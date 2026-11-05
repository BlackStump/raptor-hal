/*
 * caps_fullhan.inc -- capability tables for the Fullhan backend.
 *
 * Included from the #else arm of caps_hisilicon.inc's platform chain, and so
 * owns the unknown-platform #error. Fork-local; see FORK.md.
 *
 * A capability flag is a promise that an op exists to service it. Phase 2b
 * publishes the encoder: H.264, H.265 and MJPEG-backed JPEG, bitrate changes,
 * 8 VENC and 3 VPSS output channels (divinus's counts for the V200). ISP
 * tuning flags stay false until their ops exist.
 */

#if defined(PLATFORM_FH8852V200)
const rss_hal_caps_t g_hal_caps = {
    .soc_name = HAL_PLATFORM_NAME,
    .sdk_version = "Fullhan MPI",

    .has_h265 = true,
    .has_rotation = false,
    .has_set_bitrate = true,
    .has_gop_attr = false,
    .has_capped_rc = false,
    .has_smart_rc = false,

    .has_jpeg = true,
    .has_jpeg_qp = false,
    .jpeg_pulse = false,

    .has_color2grey = false,
    .has_defog = false,
    .has_sinter = false,
    .has_drc = false,
    .has_temper = false,
    .has_ae_comp = false,

    .max_sensors = 1,
    .max_enc_channels = 8,
    .max_fs_channels = 3,
    .max_enc_pixel_rate = 0,

    .max_osd_regions = 0,
    .max_osd_groups = 0,

    .uses_xburst2 = false,
    .uses_new_sdk = false,
    .uses_impvi = false,
};

#else
#error                                                                                             \
    "No PLATFORM_* defined. Set one of: PLATFORM_T10 T20 T21 T23 T30 T31 T32 T33 T40 T41 A1 INFINITY6E INFINITY6B0 INFINITY6C HI3516EV200 HI3516EV300 HI3516CV610 FH8852V200"
#endif
