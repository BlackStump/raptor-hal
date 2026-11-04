/*
 * hal_internal_fullhan.h -- Fullhan platform identity and vendor selection.
 *
 * Included from the #else arm of hal_internal_hisilicon.h's platform chain,
 * so reaching here means no Ingenic, SigmaStar or HiSilicon PLATFORM_*
 * matched; this file therefore owns the unknown-platform #error.
 *
 * PLATFORM_FH8852V200 names the ABI, not the package: FH8852V200, FH8856V200
 * and FH8858V200 (and the V210/V201 respins) share one die and one vendor
 * library set. The package reaches raptor separately as SOC_MODEL, and at
 * run time from the PMU registers or /proc/driver/chip.
 *
 * VENDOR SDK INCLUDES -- none, by design, as for SigmaStar and HiSilicon.
 * The backend declares the parts of the FH ABI it calls in src/fhan_v2/fh_*.h
 * (transcribed from OpenIPC divinus src/hal/fhan/f2_*.h, MIT) and dlopens the
 * board's libraries, so the build needs no SDK.
 *
 * SPDX-License-Identifier: MIT
 */

#if defined(PLATFORM_FH8852V200)
#define HAL_PLATFORM_NAME "FH8852V200"
#define HAL_FULLHAN_SDK
#define HAL_FH_V2
#else
#error "No PLATFORM_* defined"
#endif
