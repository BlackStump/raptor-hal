/*
 * hal_internal_xiongmai.h -- Xiongmai platform identity and vendor selection.
 *
 * Included from the #else arm of hal_internal_fullhan.h's platform chain, so
 * reaching here means no Ingenic, SigmaStar, HiSilicon or Fullhan PLATFORM_*
 * matched; this file therefore owns the unknown-platform #error.
 *
 * PLATFORM_XM530 names the family, not the package: the XM530 and XM550
 * share OpenIPC's xm530 kernel, toolchain and library set. The package
 * reaches raptor separately as SOC_MODEL.
 *
 * SDK INCLUDES -- openxiongmai's libraries/include (re_*.h), the open
 * XM_MPI_* API, found through SDK_INCLUDE from mk/xiongmai.mk.
 *
 * SPDX-License-Identifier: MIT
 */

#if defined(PLATFORM_XM530)
#define HAL_PLATFORM_NAME "XM530"
#define HAL_XIONGMAI_SDK
#else
#error "No PLATFORM_* defined"
#endif
