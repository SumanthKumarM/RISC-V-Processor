# =============================================================================
# Build the QEMU TCG trace plugin.
#
#   make            build trace_plugin.so
#   make clean      remove it
#
# Normally driven by `make qemu_plugin` (or `make run_qemu`) in sim/Makefile,
# which is where the rest of the second-oracle flow lives. Building here
# directly is fine too.
#
# QEMU_INCLUDE must hold the qemu-plugin.h that matches the qemu-riscv32 binary
# actually used; the plugin API is versioned (QEMU_PLUGIN_VERSION) and QEMU
# refuses to load a plugin built against a version it does not support. Both
# ship in the same toolchain tree here, so the default keeps them in step.
# =============================================================================

QEMU_PREFIX  ?= /opt/riscv-gnu/riscv
QEMU_INCLUDE ?= $(QEMU_PREFIX)/include

CC      ?= gcc
PKGCONF ?= pkg-config

GLIB_CFLAGS := $(shell $(PKGCONF) --cflags glib-2.0)
GLIB_LIBS   := $(shell $(PKGCONF) --libs   glib-2.0)

CFLAGS  := -O2 -fPIC -Wall -Wextra -std=gnu11 \
           -I$(QEMU_INCLUDE) $(GLIB_CFLAGS)
LDFLAGS := -shared
LDLIBS  := $(GLIB_LIBS)

PLUGIN := trace_plugin.so

.PHONY: all clean
all: $(PLUGIN)

$(PLUGIN): trace_plugin.c
	@test -f $(QEMU_INCLUDE)/qemu-plugin.h || \
		{ echo "*** qemu-plugin.h not found under $(QEMU_INCLUDE);"; \
		  echo "*** override with: make QEMU_INCLUDE=/path/to/include ***"; \
		  exit 1; }
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $< $(LDLIBS)
	@echo "==> built $(PLUGIN) against $(QEMU_INCLUDE)/qemu-plugin.h"

clean:
	rm -f $(PLUGIN)
