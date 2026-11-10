# mk/xiongmai.mk -- Xiongmai (XM530 family) build settings.
#
# Fork-local, -included from the top of the Makefile beside sigmastar.mk,
# hisilicon.mk and fullhan.mk; see FORK.md. Nothing here runs for any other
# PLATFORM.
#
# Usage:
#   make PLATFORM=XM530 SOC_MODEL=xm550 \
#        CROSS_COMPILE=arm-openipc-linux-musleabi- \
#        XM_SDK=/path/to/openxiongmai/libraries
#
# ONE PLATFORM, SEVERAL PACKAGES. XM530 is the family as OpenIPC spells it
# (toolchain.xiongmai-xm530, the xm530 firmware build that also runs the
# XM550). The package reaches raptor as SOC_MODEL, as for hi3516cv6xx and
# fh8852v200.
#
# THE TOOLCHAIN, read from OpenIPC's toolchain.xiongmai-xm530 (gcc 13.3,
# 2026-11-11): --with-cpu=cortex-a5 --with-float=soft (__ARM_ARCH 7,
# __ARM_PCS). Fully soft-float, not softfp.
#
# THE SDK is openxiongmai (github.com/OpenIPC/openxiongmai, GPL-3.0), not a
# vendor blob. Unlike the HiSilicon and Fullhan backends nothing is
# dlopened: the stock image has no libmpi.so to open (majestic links the
# XM_MPI_* layer statically), so rvd links openxiongmai's libopenmpi.a,
# built from source with this toolchain. raptor-hal needs only its headers;
# raptor's Makefile links the archive (XM_MPI_LIB there).
XIONGMAI_PLATFORMS := XM530

ifneq ($(filter $(PLATFORM),$(XIONGMAI_PLATFORMS)),)
VENDOR := xiongmai

# Default: openxiongmai cloned beside the XM550 tree, i.e. Cams/openxiongmai
# when this is Cams/XM550/raptor/raptor-hal. Override on the command line.
XM_SDK ?= $(abspath $(CURDIR)/../../../openxiongmai/libraries)
ifeq ($(filter clean,$(MAKECMDGOALS)),)
ifeq ($(wildcard $(XM_SDK)/include/re_sys.h),)
$(error XM_SDK=$(XM_SDK) has no include/re_sys.h; point it at openxiongmai/libraries)
endif
endif
SDK_INCLUDE := $(XM_SDK)/include

BACKEND_DIR := src/xiongmai
HAL_COMMON_SRC := $(BACKEND_DIR)/hal_common.c

# Phase 1: the skeleton. hal_common.c is HAL_COMMON_SRC (compiled into both
# archives); hal_gpio.c is vendor-neutral sysfs. Each later phase adds the
# files it writes, and only files that exist.
VIDEO_SRCS := src/hal_gpio.c
AUDIO_SRCS :=
endif
