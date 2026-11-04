# mk/fullhan.mk -- Fullhan (FH885xV200) build settings.
#
# Fork-local, -included from the top of the Makefile beside sigmastar.mk and
# hisilicon.mk; see FORK.md. Nothing here runs for any other PLATFORM.
#
# Usage:
#   make PLATFORM=FH8852V200 SOC_MODEL=fh8856v200 \
#        CROSS_COMPILE=arm-openipc-linux-musleabi-
#
# ONE PLATFORM, SEVERAL PACKAGES. FH8852V200 is the ABI's name, spelled the
# way OpenIPC spells the family (toolchain.fullhan-fh8852v200): the
# FH8852/8856/8858 V200/V210/V201 packages share one die and one vendor
# library set, and differ by package and DRAM. The package reaches raptor as
# SOC_MODEL, the same split hisilicon.mk makes for hi3516cv6xx.
#
# THE TOOLCHAIN, measured on the PB4 build (2026-11-05):
#   gcc --target=arm-openipc-linux-musleabi --with-cpu=arm1176jzf-s
#       --with-fpu=vfp --with-float=softfp   (__ARM_ARCH 6, __ARM_PCS)
#   vendor libraries: Tag_CPU_arch v4T, no FP attributes, NEEDED libc.so.0
# So: ARMv6 (ARM11, no NEON), soft-float calling convention, uClibc-built
# blobs on a musl userland. src/fhan_v2/fh_common.h carries a #error on
# __ARM_PCS_VFP, as the HiSilicon backends do.
FULLHAN_PLATFORMS := FH8852V200

ifneq ($(filter $(PLATFORM),$(FULLHAN_PLATFORMS)),)
VENDOR := fullhan

# No vendor headers, no vendor libraries to link: the backend declares the
# ABI it uses in src/fhan_v2/fh_*.h and dlopens the board's libraries.
SDK_INCLUDE :=

BACKEND_DIR := src/fhan_v2
HAL_COMMON_SRC := $(BACKEND_DIR)/hal_common.c

# Phase 2a adds fh_pipeline.c (sensor, ISP, VPSS channel 0).
# Phase 1: the skeleton. hal_common.c is HAL_COMMON_SRC (compiled into both
# archives); hal_gpio.c is vendor-neutral sysfs and the only video op that
# owes nothing to the vendor SDK. Each later phase adds the files it writes,
# and only files that exist -- an unwritten file named here fails the build.
VIDEO_SRCS := $(BACKEND_DIR)/fh_pipeline.c \
              src/hal_gpio.c
AUDIO_SRCS :=
endif
