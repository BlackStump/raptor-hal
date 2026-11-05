/*
 * fhan_v2/fh_common.h -- shared declarations for the Fullhan FH885xV200 backend.
 *
 * The FH ABI as this backend calls it. Field layouts are transcribed from
 * OpenIPC divinus src/hal/fhan/f2_*.h (MIT) and are to be checked against the
 * board's libraries, the same way the HiSilicon and SigmaStar headers were:
 * a plausible layout that is wrong does not fail to compile, it corrupts
 * memory.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef FH_COMMON_H
#define FH_COMMON_H

#include "hal_internal.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * Soft-float, and that is a compile error rather than a note.
 *
 * Measured on the PB4 image (fh8856v200, 2026-11-05): the vendor libraries
 * carry no Tag_ABI_VFP_args (Tag_CPU_arch v4T, no FP attributes at all), and
 * OpenIPC's toolchain is --with-float=softfp, __ARM_PCS. A hard-float daemon
 * links, loads and runs, and hands garbage to every float argument crossing
 * into the vendor libraries.
 */
#if defined(__arm__) && defined(__ARM_PCS_VFP)
#error "Fullhan vendor libraries are soft-float: build with arm-openipc-linux-musleabi (softfp), not a hard-float toolchain"
#endif

/* The libraries, in the order they are opened.
 *
 * libvmm first: libdsp needs the allocator libvmm defines (buffer_malloc_*),
 * resolved from the global scope, so both are RTLD_GLOBAL. RTLD_NOW because
 * musl implements no lazy binding; the flag only states what happens anyway.
 */
#define FH_LIB_VMM "libvmm.so"
#define FH_LIB_DSP "libdsp.so"

/*
 * Where the package name comes from, measured on M2 (upstream OpenIPC kernel,
 * 2026-11-05):
 *
 *   /proc/cpuinfo      "Hardware        : FH8856V200"      <- used first
 *   /proc/driver/chip  exists, but holds no "FH8..." token this backend found
 *   PMU 0xf0000000     0x19112201  (the V200 die id)
 *   PMU 0xf0000004     0x0007850b  (ip version; divinus wants 0xc/0xd in the
 *                                   low nibble, this board has 0xb)
 *   PMU 0xf000000c     0x11000001  (& 0xffffff = 0x000001, which divinus's
 *                                   table decodes as FH8852V200 -- wrong for
 *                                   this FH8856V200 board)
 *
 * So the PMU package field does not identify the part here, and this backend
 * does not decode it. cpuinfo is the kernel's own board name.
 */
/*
 * FH_LOCAL -- every function this backend shares between its own files.
 *
 * rvd is linked --export-dynamic (the vendor libraries need mmap and
 * __assert from it), which exports every global symbol it has. A vendor
 * library that calls one of its *own* functions through the PLT then binds
 * to the executable's function of the same name instead, if there is one.
 * Hidden visibility keeps this backend's names out of the dynamic symbol
 * table; only the bridge (mmap, __assert) is exported on purpose.
 */
#define FH_LOCAL __attribute__((visibility("hidden")))

#define FH_CPUINFO "/proc/cpuinfo"
#define FH_CHIP_PROC "/proc/driver/chip"

/*
 * FH_SYS_GetVersion / FH_SYS_GetChipID (libdsp). Both are exported; neither
 * layout is verified yet. divinus passes a four-word struct to GetVersion and
 * an unsigned int * to GetChipID. Until the libraries are read, both are
 * called with an oversized zeroed buffer and the raw words are logged, so a
 * wrong guess costs a log line rather than a stack.
 */
#define FH_PROBE_WORDS 16

#endif /* FH_COMMON_H */
