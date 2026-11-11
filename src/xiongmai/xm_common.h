/*
 * xiongmai/xm_common.h -- shared declarations for the Xiongmai XM530-family
 * backend.
 *
 * The MPI is openxiongmai's open reimplementation (libraries/libmpi, built
 * into libxmhost.so, which rvd links), declared by its own re_*.h
 * headers through SDK_INCLUDE. Unlike the HiSilicon and Fullhan backends
 * there is no dlopen and no ABI transcription here: the headers and the
 * implementation come from the same tree.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef XM_COMMON_H
#define XM_COMMON_H

#include "hal_internal.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * Soft-float. OpenIPC's toolchain.xiongmai-xm530 is --with-float=soft, and
 * openxiongmai's libraries are built with it; a hard-float daemon would hand
 * garbage to every float argument crossing into them.
 */
#if defined(__arm__) && defined(__ARM_PCS_VFP)
#error "Xiongmai libraries are soft-float: build with arm-openipc-linux-musleabi, not a hard-float toolchain"
#endif

/*
 * The device nodes XM_MPI_SYS_Init opens. It does not report all of them:
 * openxiongmai's XM_MPI_VI_INIT logs "open error" and still returns success
 * when /dev/vi is missing, so the backend checks the nodes itself first and
 * names the one that is absent (the kernel module load_xiongmai did not
 * load).
 */
#define XM_DEV_MMZ "/dev/mmz"
#define XM_DEV_VI  "/dev/vi"
#define XM_DEV_RGN "/dev/rgn"

/*
 * XM_MPI_SYS_Init starts with XM_MPI_SYS_MmzReset (ioctl 0x4D00 on /dev/mmz),
 * which drops every MMZ allocation in the system. Run under a live majestic
 * it pulls the buffers out from under it. hal_init refuses to start while a
 * process with this name exists.
 */
#define XM_RIVAL_STREAMER "majestic"

#endif /* XM_COMMON_H */
