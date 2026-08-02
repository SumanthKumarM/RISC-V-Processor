/*
 * trace_plugin.c - QEMU TCG plugin that records an architectural trace of an
 * RV32 program: the program counter, all 32 general-purpose registers, and
 * every store the program performs.
 *
 * This is the recording half of the second-oracle flow. qemu-riscv32 is a
 * mature implementation with no relationship to this repository, so where it
 * and rtl/risc.v agree the result is real evidence; where riscv_ref.v (our own
 * golden model) and risc.v agree, they might merely be making the same mistake
 * twice. See qemu/README.md for the full picture.
 *
 * ---------------------------------------------------------------------------
 * Why a plugin rather than -d cpu
 * ---------------------------------------------------------------------------
 * `qemu-riscv32 -one-insn-per-tb -d cpu` does print the register file before
 * every instruction, but it prints nothing about memory: QEMU has no log item
 * that reports store addresses and data. The core's data memory is a big part
 * of what we want checked, so the text log alone cannot express the trace we
 * need. The plugin API can: qemu_plugin_register_vcpu_mem_cb() delivers every
 * access with its value, and qemu_plugin_read_register() covers the registers,
 * so one mechanism produces the whole record - and produces it far faster than
 * parsing ~35 lines of pretty-printed text per instruction.
 *
 * ---------------------------------------------------------------------------
 * Output format (raw; qemu_trace.py normalises it)
 * ---------------------------------------------------------------------------
 *   S <addr> <size> <value>            one per store, hex
 *   I <pc> <r0> <r1> ... <r31>         one per executed instruction, hex
 *   E <count>                          final line, decimal instruction count
 *
 * Ordering is the subtle part. QEMU calls the instruction-execution callback
 * BEFORE the instruction runs, so the register file it sees is the state left
 * behind by the PREVIOUS instruction. Stores, by contrast, are reported while
 * their instruction runs. The plugin therefore buffers stores and flushes them
 * immediately ahead of the next `I` line, which makes each `I` line plus the
 * `S` lines above it a single coherent record:
 *
 *     "instruction N has retired; here is the resulting register file, here is
 *      the store it performed, and the next instruction to run is at <pc>"
 *
 * qemu_trace.py turns that into the canonical per-retired-instruction trace
 * that risc_tb_asm.v consumes.
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <glib.h>
#include <qemu-plugin.h>

QEMU_PLUGIN_EXPORT int qemu_plugin_version = QEMU_PLUGIN_VERSION;

#define NREG 32

/*
 * QEMU exposes RISC-V GPRs under their ABI names, but the spellings are not
 * all the obvious ones: x8 is published as "fp", not "s0". Matching on a
 * single name per register silently left x8 unresolved (and therefore reading
 * as a constant zero), which is precisely the kind of blind spot that makes a
 * verification run pass when it should fail - so every register carries its
 * full set of accepted spellings here, and an unresolved one is fatal below.
 *
 * x0 is the deliberate exception: QEMU does not publish it at all because it
 * is architecturally hardwired to zero, so reading it as zero is correct.
 */
static const char *const reg_aliases[NREG][4] = {
    { "zero", "x0",  NULL,  NULL }, { "ra",  "x1",  NULL, NULL },
    { "sp",   "x2",  NULL,  NULL }, { "gp",  "x3",  NULL, NULL },
    { "tp",   "x4",  NULL,  NULL }, { "t0",  "x5",  NULL, NULL },
    { "t1",   "x6",  NULL,  NULL }, { "t2",  "x7",  NULL, NULL },
    { "s0",   "fp",  "x8",  NULL }, { "s1",  "x9",  NULL, NULL },
    { "a0",   "x10", NULL,  NULL }, { "a1",  "x11", NULL, NULL },
    { "a2",   "x12", NULL,  NULL }, { "a3",  "x13", NULL, NULL },
    { "a4",   "x14", NULL,  NULL }, { "a5",  "x15", NULL, NULL },
    { "a6",   "x16", NULL,  NULL }, { "a7",  "x17", NULL, NULL },
    { "s2",   "x18", NULL,  NULL }, { "s3",  "x19", NULL, NULL },
    { "s4",   "x20", NULL,  NULL }, { "s5",  "x21", NULL, NULL },
    { "s6",   "x22", NULL,  NULL }, { "s7",  "x23", NULL, NULL },
    { "s8",   "x24", NULL,  NULL }, { "s9",  "x25", NULL, NULL },
    { "s10",  "x26", NULL,  NULL }, { "s11", "x27", NULL, NULL },
    { "t3",   "x28", NULL,  NULL }, { "t4",  "x29", NULL, NULL },
    { "t5",   "x30", NULL,  NULL }, { "t6",  "x31", NULL, NULL },
};

static int verbose;

static FILE *out;
static struct qemu_plugin_register *reg_handles[NREG];
static GByteArray *regbuf;

/* Stores performed by the instruction currently executing. Flushed just before
 * the next instruction's record. RV32 can only ever produce one store per
 * instruction, but the buffer holds a few so a surprise never silently drops
 * data. */
#define MAX_PENDING 8
struct store_rec {
    uint64_t addr;
    unsigned size;
    uint64_t val;
};
static struct store_rec pending[MAX_PENDING];
static unsigned n_pending;
static unsigned dropped_stores;

static uint64_t n_insn;
static uint64_t max_insn;   /* 0 = unlimited; safety net against runaway loops */

/*
 * Collect the register handles once the vCPU exists. Handles are stable for
 * the life of the vCPU, so this is done once rather than per instruction.
 */
static void vcpu_init(qemu_plugin_id_t id, unsigned int vcpu_index)
{
    GArray *regs = qemu_plugin_get_registers();

    (void)id;
    (void)vcpu_index;

    if (!regs) {
        return;
    }

    for (guint i = 0; i < regs->len; i++) {
        qemu_plugin_reg_descriptor *d =
            &g_array_index(regs, qemu_plugin_reg_descriptor, i);

        if (verbose) {
            fprintf(stderr, "trace_plugin: register '%s' (feature %s)\n",
                    d->name, d->feature ? d->feature : "-");
        }

        for (int r = 0; r < NREG; r++) {
            if (reg_handles[r]) {
                continue;
            }
            for (int a = 0; a < 4 && reg_aliases[r][a]; a++) {
                if (!g_strcmp0(d->name, reg_aliases[r][a])) {
                    reg_handles[r] = d->handle;
                    break;
                }
            }
        }
    }

    g_array_free(regs, TRUE);

    /* Bail out rather than trace a register as a constant zero: a silently
     * unresolved register would make every divergence in it invisible, so the
     * run would "pass" without ever having checked it. Index 0 is exempt -
     * x0 really is always zero. */
    for (int r = 1; r < NREG; r++) {
        if (!reg_handles[r]) {
            fprintf(stderr,
                    "trace_plugin: FATAL - QEMU exposes no register matching "
                    "x%d (tried '%s'). Re-run with the plugin argument "
                    "'verbose=1' to list the names this QEMU publishes, then "
                    "add the right spelling to reg_aliases[] in %s.\n",
                    r, reg_aliases[r][0], __FILE__);
            exit(1);
        }
    }
}

/*
 * qemu_plugin_read_register() fills the buffer in target byte order, which for
 * RV32 is little-endian, so the low byte comes first.
 */
static uint32_t read_reg(int r)
{
    uint32_t v = 0;

    if (!reg_handles[r]) {
        return 0;
    }
    g_byte_array_set_size(regbuf, 0);
    if (!qemu_plugin_read_register(reg_handles[r], regbuf)) {
        return 0;
    }
    for (guint i = 0; i < regbuf->len && i < 4; i++) {
        v |= (uint32_t)regbuf->data[i] << (8 * i);
    }
    return v;
}

static void flush_pending(void)
{
    for (unsigned i = 0; i < n_pending; i++) {
        fprintf(out, "S %" PRIx64 " %u %" PRIx64 "\n",
                pending[i].addr, pending[i].size, pending[i].val);
    }
    n_pending = 0;
}

/*
 * Fires before the instruction at @udata executes, so the registers read here
 * are the ones the previous instruction left behind - which is exactly the
 * post-retire state we want to compare against the DUT.
 */
static void insn_exec(unsigned int vcpu_index, void *udata)
{
    uint64_t pc = (uint64_t)(uintptr_t)udata;

    (void)vcpu_index;

    flush_pending();

    fprintf(out, "I %08" PRIx64, pc);
    for (int r = 0; r < NREG; r++) {
        fprintf(out, " %08x", read_reg(r));
    }
    fputc('\n', out);

    n_insn++;
    if (max_insn && n_insn >= max_insn) {
        /* A program that never reaches its halt would otherwise fill the disk.
         * Stop here; qemu_trace.py reports the truncation rather than letting
         * a short trace look like a clean early finish. */
        fprintf(out, "# max-insn cap (%" PRIu64 ") reached\n", max_insn);
        fflush(out);
        exit(0);
    }
}

static void mem_access(unsigned int vcpu_index, qemu_plugin_meminfo_t info,
                       uint64_t vaddr, void *udata)
{
    qemu_plugin_mem_value v;
    uint64_t val;
    unsigned size;

    (void)vcpu_index;
    (void)udata;

    if (!qemu_plugin_mem_is_store(info)) {
        return;
    }

    v = qemu_plugin_mem_get_value(info);
    switch (v.type) {
    case QEMU_PLUGIN_MEM_VALUE_U8:
        size = 1; val = v.data.u8;  break;
    case QEMU_PLUGIN_MEM_VALUE_U16:
        size = 2; val = v.data.u16; break;
    case QEMU_PLUGIN_MEM_VALUE_U32:
        size = 4; val = v.data.u32; break;
    case QEMU_PLUGIN_MEM_VALUE_U64:
        size = 8; val = v.data.u64; break;
    default:
        return;
    }

    if (n_pending >= MAX_PENDING) {
        dropped_stores++;
        return;
    }
    pending[n_pending].addr = vaddr;
    pending[n_pending].size = size;
    pending[n_pending].val  = val;
    n_pending++;
}

static void tb_trans(qemu_plugin_id_t id, struct qemu_plugin_tb *tb)
{
    size_t n = qemu_plugin_tb_n_insns(tb);

    (void)id;

    for (size_t i = 0; i < n; i++) {
        struct qemu_plugin_insn *insn = qemu_plugin_tb_get_insn(tb, i);
        uint64_t vaddr = qemu_plugin_insn_vaddr(insn);

        /* R_REGS is what makes qemu_plugin_read_register() legal in the cb. */
        qemu_plugin_register_vcpu_insn_exec_cb(insn, insn_exec,
                                               QEMU_PLUGIN_CB_R_REGS,
                                               (void *)(uintptr_t)vaddr);
        qemu_plugin_register_vcpu_mem_cb(insn, mem_access,
                                         QEMU_PLUGIN_CB_NO_REGS,
                                         QEMU_PLUGIN_MEM_W, NULL);
    }
}

static void plugin_exit(qemu_plugin_id_t id, void *p)
{
    (void)id;
    (void)p;

    /* The final instruction's store has no following record to ride along
     * with, so flush it here. */
    flush_pending();
    if (dropped_stores) {
        fprintf(out, "# WARNING: %u store(s) dropped, MAX_PENDING too small\n",
                dropped_stores);
    }
    fprintf(out, "E %" PRIu64 "\n", n_insn);
    fflush(out);
    if (out != stdout) {
        fclose(out);
    }
}

QEMU_PLUGIN_EXPORT int qemu_plugin_install(qemu_plugin_id_t id,
                                           const qemu_info_t *info,
                                           int argc, char **argv)
{
    const char *path = NULL;

    for (int i = 0; i < argc; i++) {
        char **tok = g_strsplit(argv[i], "=", 2);

        if (tok[0] && tok[1] && !g_strcmp0(tok[0], "outfile")) {
            path = g_strdup(tok[1]);
        } else if (tok[0] && tok[1] && !g_strcmp0(tok[0], "max")) {
            max_insn = g_ascii_strtoull(tok[1], NULL, 0);
        } else if (tok[0] && tok[1] && !g_strcmp0(tok[0], "verbose")) {
            verbose = g_ascii_strtoull(tok[1], NULL, 0) != 0;
        } else {
            fprintf(stderr, "trace_plugin: unknown argument '%s'\n", argv[i]);
            g_strfreev(tok);
            return -1;
        }
        g_strfreev(tok);
    }

    if (info && info->target_name && !g_str_has_prefix(info->target_name, "riscv")) {
        fprintf(stderr, "trace_plugin: target is '%s', expected riscv\n",
                info->target_name);
        return -1;
    }

    out = path ? fopen(path, "w") : stdout;
    if (!out) {
        fprintf(stderr, "trace_plugin: cannot open '%s' for writing\n", path);
        return -1;
    }

    regbuf = g_byte_array_new();

    qemu_plugin_register_vcpu_init_cb(id, vcpu_init);
    qemu_plugin_register_vcpu_tb_trans_cb(id, tb_trans);
    qemu_plugin_register_atexit_cb(id, plugin_exit, NULL);

    return 0;
}
