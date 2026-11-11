/*
 * caps_xiongmai.inc -- capability tables for the Xiongmai backend.
 *
 * Included from the #else arm of caps_fullhan.inc's platform chain, and so
 * owns the unknown-platform #error. Fork-local; see FORK.md.
 *
 * Phase 2a: two framesources (VI channel 0 at the sensor window, and
 * extended channel 1, scaled), no encoder yet, so rvd stops at its first
 * encoder. The hardware ceilings, from openxiongmai's re_defines.h:
 * VENC_MAX_CHN_NUM 4, VIU_MAX_CHN_NUM 3 (chn 0 plus extended channels),
 * RGN_MAX_CHN_NUM 3.
 */

#if defined(PLATFORM_XM530)
const rss_hal_caps_t g_hal_caps = {
    .soc_name = HAL_PLATFORM_NAME,
    .sdk_version = "openxiongmai libmpi",

    .has_h265 = false,
    .has_rotation = false,
    .has_set_bitrate = false,
    .has_gop_attr = false,
    .has_capped_rc = false,
    .has_smart_rc = false,

    .has_framesource = true,
    .has_osd = false,
    .has_ivs = false,

    .has_jpeg = false,
    .has_jpeg_qp = false,
    .jpeg_pulse = false,

    .has_color2grey = false,
    .has_defog = false,
    .has_sinter = false,
    .has_drc = false,
    .has_temper = false,
    .has_ae_comp = false,

    .max_sensors = 1,
    .max_enc_channels = 0,
    .max_fs_channels = 2,
    .max_enc_pixel_rate = 0,

    .max_osd_regions = 0,
    .max_osd_groups = 0,

    .uses_xburst2 = false,
    .uses_new_sdk = false,
    .uses_impvi = false,
};

#else
#error                                                                                             \
    "No PLATFORM_* defined. Set one of: PLATFORM_T10 T20 T21 T23 T30 T31 T32 T33 T40 T41 A1 INFINITY6E INFINITY6B0 INFINITY6C HI3516EV200 HI3516EV300 HI3516CV610 FH8852V200 XM530"
#endif
