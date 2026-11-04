/*
 * caps_hisilicon.inc -- capability tables for the HiSilicon backends.
 *
 * Included from the #else arm of caps_sigmastar.inc's platform chain, which is
 * itself the #else arm of hal_caps.c's. Chaining here rather than adding a
 * second hook to hal_caps.c keeps the shared file at one include; this file
 * therefore inherits the duty caps_sigmastar.inc used to hold, and owns the
 * unknown-platform #error. Fork-local; see FORK.md.
 *
 * ═══════════════════════════════════════════════════════════════════════
 * WHAT IS PUBLISHED HERE, AND WHY SO LITTLE
 *
 * A capability flag is a promise that an op exists to service it. Phase 2
 * brings up the video pipeline -- VI, the ISP, VPSS and VENC -- so the
 * encoder flags whose ops now exist are true and the rest stay false by
 * design rather than by omission. rvd reads a false flag as "this platform
 * cannot", which is exactly true. Phases 3-5 raise each remaining flag in the
 * commit that publishes its op, never ahead of it.
 *
 * The numbers that are non-zero are the ones measured on hardware.
 *
 * ═══════════════════════════════════════════════════════════════════════
 * MEASURED ON A HI3516EV300, 2026-08-31
 *
 * OpenIPC hi3516ev300_lite, IMX335 at 2592x1944, live two-stream pipeline.
 * Raw dumps in board-dumps/hi3516ev300-196-20260831/ of the parent tree; the
 * write-up is PROBE-hi3516ev300-phase-minus-1.md.
 *
 *   max_enc_channels   3   /proc/umap/venc reports VencMaxChnNum = 3 as a
 *                          module parameter. That is a driver-level cap, well
 *                          under RVD_MAX_STREAMS, and it is a real number
 *                          rather than a header constant. It is also the
 *                          number OpenIPC's load_hisilicon passed, so hal_init
 *                          reads the parameter back from sysfs and publishes
 *                          what this image loaded; 3 is the fallback.
 *
 *   max_fs_channels    2   A floor, not a ceiling, and the one number here
 *                          that is expected to rise. Two VPSS channels were
 *                          observed enabled on a live pipeline; the driver
 *                          reports u32VpssSplitNodeNum = 3, and divinus's
 *                          V4_VPSS_CHN_NUM 6 is a constant in its source
 *                          rather than a measurement. Publishing the observed
 *                          number keeps the promise honest until the
 *                          create-channels-0..5 experiment runs (risk R6).
 *
 *                          Phase 2 added a prediction to test rather than a
 *                          measurement to publish: the SDK's own header for
 *                          this chip says VPSS_MAX_PHY_CHN_NUM is 3 and
 *                          VPSS_MAX_CHN_NUM is 7 (three physical channels
 *                          plus four extension channels), which is what
 *                          V4_VPSS_MAX_PHY_CHN_NUM in v4_vpss.h sizes the
 *                          channel array by. A header constant is still not
 *                          a measurement, so this number does not move until
 *                          somebody creates channels 0..5 on a board and
 *                          records which ones accept SetChnAttr.
 *
 *   max_enc_pixel_rate 0   Unmeasured, which rvd reads as "do not test
 *                          against". The board runs 5M@20 plus a substream,
 *                          but a working configuration is not a rated ceiling.
 *
 * Per-part differences are expected here eventually -- EV200 and EV300 differ
 * in encoder rating even though they share an MPP build -- which is what the
 * part-level #if chain below is for, even though both arms are identical now.
 * ═══════════════════════════════════════════════════════════════════════
 *
 * SPDX-License-Identifier: MIT
 */

#if defined(PLATFORM_HI3516EV200) || defined(PLATFORM_HI3516EV300)
const rss_hal_caps_t g_hal_caps = {
    .soc_name = HAL_PLATFORM_NAME,
    /*
     * A family name, not a release. The running MPP answers with its own
     * version string -- "Hi3516EV200_MPP_V1.0.1.2 B030 Release" on the EV300
     * board, which is also why the string here cannot be a release number --
     * and hal_sys_get_version passes that through, so anything wanting the
     * exact drop asks the op rather than reading this.
     */
    .sdk_version = "HiMPP",

    /*
     * Encoder.
     *
     * has_h265 was already true on evidence rather than on the datasheet --
     * the board's VENC channel 0 runs H.265 at 2592x1944. has_set_bitrate is
     * true as of Phase 2 because enc_set_bitrate exists and works the only
     * way HiMPP allows: a read-modify-write of the whole channel attribute.
     *
     * has_rotation is true and per channel: HI_MPI_VPSS_SetChnRotation
     * turns one VPSS channel's output by 90, 180 or 270, so each stream can
     * be turned on its own (no SigmaStar backend turns one at all: the 6C's
     * scaler answers MI_ERR_SCL_NOT_SUPPORT and star publishes no such op). The
     * channel attribute keeps the configured size and the picture leaves
     * with width and height swapped, which rvd sizes the encoder for.
     *
     * The three that stay false are decisions, not gaps:
     *
     *   has_gop_attr   The op would be enc_set_chn_gop_attr, which takes an
     *                  opaque vendor struct. gen4's is VENC_GOP_ATTR_S with a
     *                  per-mode union, and publishing it means committing to
     *                  a layout callers pass through unexamined.
     *   has_capped_rc  gen4 has AVBR, QVBR and CVBR, and each needs
     *   has_smart_rc   parameters rvd does not carry -- u32ChangePos, the
     *                  quality-level table, the long-term statistics window.
     *                  enc_set_rc_mode maps them onto VBR with a maximum
     *                  bitrate, which is the honest approximation; claiming
     *                  the capability would promise a quality target nobody
     *                  supplied.
     */
    .has_h265 = true,
    .has_rotation = true,
    .has_set_bitrate = true,
    .has_gop_attr = false,
    .has_capped_rc = false,
    .has_smart_rc = false,

    /*
     * JPEG. Snapshot channels are PT_MJPEG -- PT_JPEG refuses this
     * attribute set outright (0xa0088003 at CreateChn, measured), divinus
     * never creates one either, and an MJPEG channel emits the same JPEG
     * bytes. Quality is the FIXQP rc's qfactor, which is what
     * enc_set_jpeg_qp drives -- despite the op's name, the value is rvd's
     * 1..100 quality, mapped onto the driver's 1..99 Qfactor. jpeg_pulse
     * because a FIXQP MJPEG channel has no rate limiter: without rvd's
     * start-one-frame-stop pacing, a held snapshot consumer would have the
     * channel encoding full-size JPEGs at the pipeline's full rate.
     * max_jpeg_pixels stays 0 -- unmeasured, per the rule above.
     */
    .has_jpeg = true,
    .has_jpeg_qp = true,
    .jpeg_pulse = true,

    /*
     * ISP -- Phase 3. The two flags that are true are the two knobs
     * hal_knob.c publishes beyond brightness and contrast: DRC strength
     * (ISP_DRC_ATTR_S, 0..1023, pinned in manual mode) and AE compensation
     * (stAuto.u8Compensation, 0..255). The rest have no op.
     */
    .has_color2grey = false,
    .has_defog = false,
    .has_sinter = false,
    .has_drc = true,
    .has_temper = false,
    .has_ae_comp = true,

    /*
     * One sensor. Gen4 has a second VI pipe, but sensor identity here is
     * whichever libsns_*.so gets dlopened and the ISP is keyed on vi_pipe, so
     * a second sensor is a design question and not a spare register.
     */
    .max_sensors = 1,

    .max_enc_channels = 3,
    .max_fs_channels = 2,
    .max_enc_pixel_rate = 0,

    /*
     * OSD -- Phase 5, over HI_MPI_RGN.
     *
     * max_osd_regions is HISI_OSD_REGION_MAX from src/hisi_v4/hisi_state.h,
     * the backend's own tracking bound. Not the vendor's RGN_HANDLE_MAX
     * (128): the limit a caller runs into is OVERLAY_MAX_NUM_VENC, 8
     * overlays per encoder channel (hi_defines.h), and 16 is two channels'
     * worth. Not referenced symbolically because this file is compiled for
     * every platform and must not pull in the gen4 headers.
     *
     * max_osd_groups is 3 because a group here *is* an encoder channel --
     * regions attach to VENC, which is the only stage with a non-zero
     * overlay budget on gen4 -- so it matches max_enc_channels above rather
     * than being a number of its own.
     *
     * The has_osd_* flags stay false. gen4's RGN does have MOSAIC regions
     * and an invert-colour sub-struct, but rss_osd_region_t has no field to
     * drive either, and there is no group callback at all; claiming a
     * capability raptor cannot ask for would promise nothing.
     */
    .max_osd_regions = 16,
    .max_osd_groups = 3,

    .uses_xburst2 = false,
    .uses_new_sdk = false,
    .uses_impvi = false,
};

#elif defined(PLATFORM_HI3516CV610)
/*
 * ═══════════════════════════════════════════════════════════════════════
 * MEASURED THROUGH PHASE 2 -- HiMPP V5, hi3516cv6xx family
 *
 * The video pipeline is up, so the numbers below are what a CV608 bench
 * board (192.168.1.238, os04d10) actually did: four encoder channels
 * across two framesources, H.264 and JPEG, streaming at once. Everything
 * Phase 3 and later owns is still false and still 0. Each flag rises in
 * the commit that publishes its op, never ahead of it -- the same rule the
 * gen4 block above was built under.
 *
 * ONE PLATFORM, TWO DIES. soc_name is the ABI's name (see
 * hal_internal_hisilicon.h); the die reaches raptor as soc_model, which the
 * build passes through from OpenIPC's BR2_OPENIPC_SOC_MODEL -- hi3516cv608 on
 * the bench board, hi3516cv610 elsewhere. When the parts turn out to differ,
 * this arm splits on soc_model the way the SigmaStar families do, not into two
 * PLATFORMs. One difference is already known: OpenIPC's load_hisilicon
 * modprobes the AI-ISP modules (open_aiisp, open_svac3e) only for certain
 * socmodel values, so AI-BNR is not a family-wide capability (plan, risk R11).
 *
 * PREDICTIONS TO TEST, NOT NUMBERS TO TRUST. From the 1.0.2.0 headers
 * (openhisilicon/kernel/include/hi3516cv6xx/ot_defines.h) -- a header constant
 * is not a measurement, which is exactly why none of them is published below:
 *
 *   OT_VPSS_MAX_PHYS_CHN_NUM  3 (+2 extension)   OT_VI_MAX_DEV_NUM      2
 *   OT_VENC_MAX_CHN_NUM      16, but OpenIPC's load_hisilicon passes
 *                            g_venc_max_chn_num=8, and gen4 taught us the
 *                            module parameter is the real cap -- hal_init
 *                            reads it back from sysfs there and should here
 *                            OT_VI_MAX_PHYS_PIPE_NUM  2   OT_RGN_HANDLE_MAX  128
 *                            OT_AI_MAX_CHN_NUM        4
 *
 * BENCH ITEM 5 IS ANSWERED: physical VPSS channel 0 is a usable output on
 * V5, where gen4 reserves it because HI_MPI_SYS_Bind forces a VPSS
 * destination's channel to 0. Here the group's input port and the output
 * channels are numbered separately, so HISI_VPSS_CHN_BASE is 0 and
 * framesource n is physical channel n. Measured: with channels 0 and 1
 * enabled at 1920x1080 and 640x360, /proc/umap/vpss shows send_ok climbing
 * on both.
 * ═══════════════════════════════════════════════════════════════════════
 */
const rss_hal_caps_t g_hal_caps = {
    .soc_name = HAL_PLATFORM_NAME,
    /* A family name, not a release; hal_sys_get_version answers with the
     * running MPP's own string once Phase 1 can ask it. */
    .sdk_version = "HiMPP",

    /*
     * Encoder.
     *
     * has_rotation stays false: ot_vpss_chn_attr does carry a rotation
     * field, but hal_fs_set_rotation accepts 0 and answers NOTSUP to
     * anything else, and a cap for an op that refuses the only interesting
     * arguments would be a lie.
     *
     * has_gop_attr, has_capped_rc and has_smart_rc stay false for gen4's
     * reasons: SMART_P changes which arm of the GOP union is read, and
     * AVBR/QVBR/CVBR each need parameters rvd does not carry.
     */
    .has_h265 = true,
    .has_rotation = false,
    .has_set_bitrate = true,
    .has_gop_attr = false,
    .has_capped_rc = false,
    .has_smart_rc = false,

    /*
     * JPEG. Every JPEG-class channel here is PT_MJPEG -- see
     * hisi_enc_payload -- so quality is the FIXQP rc's qfactor, reached
     * through the ordinary reconfigure, and rvd's 1..100 quality maps onto
     * the driver's 1..99 the same way up. jpeg_pulse says rvd owns the
     * channel's duty cycle: it starts the channel when a snapshot consumer
     * appears and stops it when the last one leaves, and the backend
     * defers the bind across that stop.
     *
     * max_jpeg_pixels stays 0 -- unmeasured, per the rule above.
     */
    .has_jpeg = true,
    .has_jpeg_qp = true,
    .jpeg_pulse = true,

    /* ISP. Phase 3, and this is where V5 diverges most from gen4: the tuning
     * is a 46-section .ini dialect and several of these knobs have no direct
     * equivalent. */
    .has_color2grey = false,
    .has_defog = false,
    .has_sinter = false,
    .has_drc = false,
    .has_temper = false,
    .has_ae_comp = false,

    /* One sensor on VI device 0, pipe 0, which is the topology
     * src/hisi_v5/hisi_state.h fixes. */
    .max_sensors = 1,

    /*
     * Four encoder channels, which is what ran together on the bench: two
     * H.264 and two MJPEG, all bound and all delivering. Not the eight
     * OpenIPC's load_hisilicon gives the driver (g_venc_max_chn_num=8, read
     * back from /sys/module/open_venc/parameters) and not the header's 16
     * -- a channel needs a stream buffer out of a 32 MB MMZ that is already
     * 30 MB full with four, so the module's number is a ceiling rather than
     * a capability. Phase 7's memory work is where that gets revisited.
     *
     * Two framesources, for the same reason: the part has three physical
     * VPSS channels and channel 0 is usable (see above), but two is what
     * rvd builds for one sensor and two is what was measured.
     *
     * max_enc_pixel_rate stays 0: what the bench measured -- 1920x1080 and
     * 640x360 at 16.7 fps each -- is VB starvation on a 32 MB MMZ, not the
     * encoder's ceiling, and publishing a number the encoder was never
     * asked for would be publishing a guess.
     */
    .max_enc_channels = 4,
    .max_fs_channels = 2,
    .max_enc_pixel_rate = 0,

    /* OSD -- Phase 5, over ss_mpi_rgn. */
    .max_osd_regions = 0,
    .max_osd_groups = 0,

    .uses_xburst2 = false,
    .uses_new_sdk = false,
    .uses_impvi = false,
};

#else
/* Fullhan, which owns the unknown-platform #error from here on. */
#include "caps_fullhan.inc"
#endif
