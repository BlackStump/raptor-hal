/*
 * caps_fullhan.inc -- capability tables for the Fullhan backend.
 *
 * Included from the #else arm of caps_hisilicon.inc's platform chain, and so
 * owns the unknown-platform #error. Fork-local; see FORK.md.
 *
 * A capability flag is a promise that an op exists to service it. Phase 1
 * publishes no pipeline ops, so every encoder, JPEG and ISP flag is false and
 * every count is 0 -- rvd reads that as "this platform cannot", which is
 * exactly true until Phase 2. Each flag rises in the commit that publishes its
 * op, and each number only once it has been measured on the PB4.
 */

#if defined(PLATFORM_FH8852V200)
const rss_hal_caps_t g_hal_caps = {
    .soc_name = HAL_PLATFORM_NAME,
    .sdk_version = "Fullhan MPI",

    .has_h265 = false,
    .has_rotation = false,
    .has_set_bitrate = false,
    .has_gop_attr = false,
    .has_capped_rc = false,
    .has_smart_rc = false,

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
    .max_fs_channels = 0,
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
