/*
 * hal_internal_hisilicon.h -- HiSilicon platform identity and vendor selection.
 *
 * Included from the #else arm of hal_internal_sigmastar.h's platform-name
 * chain, which is why it also owns the unknown-platform #error: reaching here
 * means no Ingenic and no SigmaStar PLATFORM_* matched, so either this is a
 * HiSilicon part or it is nothing this tree knows.
 *
 * Chained off the SigmaStar header rather than off hal_internal.h so that
 * adding a third vendor costs the shared file nothing at all; see FORK.md.
 *
 * ═══════════════════════════════════════════════════════════════════════
 * GENERATION, NOT PART
 *
 * Every guard in a backend keys on HAL_HISI_GEN4 or HAL_HISI_GEN5, never on
 * PLATFORM_HI3516*. That is a measurement, not a style preference: an EV300
 * board reports "Hi3516EV200_MPP_V1.0.1.2 B030" on every /proc/umap node, so
 * EV200, EV300, DV200 and 3518EV300 are one MPP build with one library set and
 * one ABI; and OpenIPC's hi3516cv6xx libraries answer
 * "HI3516CV610_MPP_V1.0.2.0 B051 Release" on a CV608 die as readily as on a
 * CV610. A part macro would therefore be guarding a distinction that does not
 * exist, and the fourth gen4 part would have to find and edit every one of them.
 *
 * What the part macro is legitimately for is the capability tables, where the
 * parts genuinely differ, and the runtime chip-ID check.
 *
 *   src/hisi_v4/   HiMPP V4.0 -- VI -> VPSS -> VENC, ISP keyed on vi_pipe,
 *                  sensor drivers as userspace libsns_*.so.
 *   src/hisi_v5/   HiMPP V5.0 -- the hi3516cv6xx family (CV610, CV608). Same
 *                  VI -> VPSS -> VENC spine, different everything else: one
 *                  libss_mpi.so where gen4 had libmpi plus libisp, ss_mpi_
 *                  entry points over ot_-prefixed types, MMZ moved back out of
 *                  OSAL, and open_*.ko on OpenIPC's 5.10 kernel rather than the
 *                  vendor's ot_*.ko. It gets its own directory rather than
 *                  #ifdefs here because sharing a translation unit would mean
 *                  hiding an argument list behind a macro at nearly every call
 *                  site -- which is exactly how src/star/ and src/infinity6c/
 *                  ended up separate. Being built out one phase at a time; see
 *                  PLAN-hi3516cv610.md in the parent tree.
 *
 * ═══════════════════════════════════════════════════════════════════════
 * VENDOR SDK INCLUDES -- there are none, by design
 *
 * As with SigmaStar, and for a different reason. HiSilicon's MPP headers are
 * not redistributable, but openhisilicon's GPL reimplementation is; the
 * backend still declares its own ABI in src/hisi_v4/v4_*.h and dlopens
 * libmpi/libisp, because the board's libraries are the ABI and a header can
 * disagree with them. See PLAN-hi3516ev200.md, "Where the ABI structs come
 * from".
 * ═══════════════════════════════════════════════════════════════════════
 *
 * SPDX-License-Identifier: MIT
 */

#if defined(PLATFORM_HI3516EV200)
#define HAL_PLATFORM_NAME "HI3516EV200"
#define HAL_HISILICON_SDK
#define HAL_HISI_GEN4
#elif defined(PLATFORM_HI3516EV300)
#define HAL_PLATFORM_NAME "HI3516EV300"
#define HAL_HISILICON_SDK
#define HAL_HISI_GEN4
#elif defined(PLATFORM_HI3516CV610)
/*
 * The ABI's name, not the die's: this platform is every hi3516cv6xx part, and
 * the die arrives separately as soc_model (hi3516cv608, hi3516cv610). See the
 * GENERATION, NOT PART block above.
 */
#define HAL_PLATFORM_NAME "HI3516CV610"
#define HAL_HISILICON_SDK
#define HAL_HISI_GEN5
#else
/* Fullhan, and the unknown-platform #error from here on; chained the same way
 * this file is chained off the SigmaStar one. */
#include "hal_internal_fullhan.h"
#endif
