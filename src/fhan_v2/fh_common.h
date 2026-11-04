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

/* /proc/driver/chip names the package on every Fullhan kernel seen so far
 * ("FH8856V200" on the PB4). divinus prefers the PMU registers at 0xf0000000;
 * the PB4's package id 0x7850b is not in its table and it falls back to
 * this file, so this backend starts here. */
#define FH_CHIP_PROC "/proc/driver/chip"

/* FH_SYS_GetVersion (libdsp). Layout from divinus f2_sys.h -- unverified. */
typedef struct {
    unsigned int date;
    unsigned int commit; /* git hash */
    unsigned int package;
    unsigned int reserved;
} fh_sys_ver;

#endif /* FH_COMMON_H */
