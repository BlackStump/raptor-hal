/*
 * hisi_v5/hal_framesource.c -- VPSS channels, HiMPP V5.0
 *
 * A raptor framesource is one VPSS channel. The group is the pipeline's
 * and belongs to hal_common.c; everything per-stream -- geometry, frame
 * rate, rotation, the userspace frame queue -- is here.
 *
 * WHAT A CHANNEL IS ON V5, and the one place it differs from gen4:
 * physical channel 0 is usable. gen4 reserves it because HI_MPI_SYS_Bind
 * forces a VPSS destination's channel to 0, so the group's input occupies
 * it; V5 numbers the group's input port and the output channels
 * separately, and the vendor's own sample binds VPSS channel 0 to VENC
 * channel 0. See HISI_VPSS_CHN_BASE.
 *
 * THERE IS NO DestroyChn. ss_mpi_vpss_disable_chn is the whole of channel
 * teardown; the channel keeps its attribute and comes back with
 * enable_chn. So hal_fs_destroy_channel disables and forgets, and a later
 * create writes the attribute again rather than assuming what is there.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#define _POSIX_C_SOURCE 200809L /* nanosleep */

#include "hisi_state.h"

#include <time.h>

#include <stdio.h>
#include <stdlib.h>

/*
 * The channel index rvd uses, checked and turned into the physical one.
 *
 * Every op starts here, so the range check is written once. Returning the
 * state rather than a bool lets the caller do both in one line.
 */
static hisi_vpss_chn_t *hisi_fs_chn(void *ctx, int chn)
{
    hisi_state_t *st = hisi_state(ctx);

    if (!st || chn < 0 || chn >= HISI_VPSS_CHN_NUM)
        return NULL;
    return &st->fs[chn];
}

/*
 * hisi_fs_frame_rate -- rvd's fps_num/fps_den into ot_frame_rate_ctrl.
 *
 * The vendor's pair is (source rate, destination rate) in whole frames,
 * with -1 meaning "do not control". So a stream at the sensor's own rate
 * asks for no control at all rather than for src == dst: the latter is
 * accepted and makes VPSS run a divider that always passes, which is a
 * frame-timing decision taken for no reason.
 *
 * A non-integer rate is rounded up, not down. Rounding down turns 29.97
 * into 29 and quietly drops a frame a second on a stream whose whole point
 * was to match a 30 fps source.
 */
static void hisi_fs_frame_rate(const hisi_state_t *st, uint32_t num, uint32_t den,
                               v5_frame_rate_ctrl *out)
{
    unsigned int src = (unsigned int)(st->mode.frame_rate + 0.5f);
    unsigned int dst;

    out->src_frame_rate = -1;
    out->dst_frame_rate = -1;

    if (!num || !den || !src)
        return;

    dst = (num + den - 1u) / den;
    if (!dst || dst >= src)
        return;

    out->src_frame_rate = (int)src;
    out->dst_frame_rate = (int)dst;
}

/*
 * The VPSS channels' share of [image]'s orientation: all of it, unless the
 * sensor is turning the picture itself, in which case none -- turning it
 * twice would turn it back. See hal_knob.c's orientation section.
 */
static int hisi_fs_vpss_mirror(const hisi_state_t *st)
{
    return hisi_snr_orien_at_sensor(st) ? 0 : st->mirror;
}

static int hisi_fs_vpss_flip(const hisi_state_t *st)
{
    return hisi_snr_orien_at_sensor(st) ? 0 : st->flip;
}

/*
 * hisi_fs_compress -- what this channel's output is compressed with.
 *
 * SEG_COMPACT on physical channel 0 and nothing anywhere else. Two
 * separate reasons, and both are narrow:
 *
 *   - The part. The tuning guide's VPSS table says CV610 supports YUV
 *     output compression on CHN0 only, where its larger siblings do CHN0
 *     and CHN1. The vendor's own samples agree -- sample_venc.c's
 *     get_default_vpss_chn_attr writes SEG_COMPACT for i == 0 and NONE
 *     for the rest.
 *
 *   - The reader. A compressed block is a format VENC understands and
 *     userspace does not. depth is nonzero exactly when rvd intends to
 *     pull frames off this channel by hand, so compression is dropped for
 *     as long as that is true and comes back when the depth returns to 0.
 *     Callers do not have to know: hal_fs_set_frame_depth rewrites the
 *     attribute and resizes the pool on the same transition.
 *
 *   - Orientation. The VPSS reference is explicit that SEG_COMPACT output
 *     and mirror/flip cannot both be on, and the driver enforces it:
 *     measured on a CV608, ss_mpi_vpss_set_chn_attr answers 0xa007800d
 *     (NOT_PERM) to an attribute carrying mirror_en while compress_mode is
 *     SEG_COMPACT, and 0 to the same attribute with compress_mode NONE.
 *     So a turned picture costs the compression for as long as it is
 *     turned, and orientation is a whole-pipeline setting rather than a
 *     per-channel one, which is why this reads the backend's state.
 *
 * Worth about 31% of the block -- 3,110,400 bytes down to roughly 2.13 MB
 * at 1080p.
 */
static v5_compress_mode hisi_fs_compress(const hisi_state_t *st, const hisi_vpss_chn_t *fs, int phy)
{
    if (phy != 0 || fs->depth)
        return V5_COMPRESS_MODE_NONE;
    if (hisi_fs_vpss_mirror(st) || hisi_fs_vpss_flip(st))
        return V5_COMPRESS_MODE_NONE;
    return V5_COMPRESS_MODE_SEG_COMPACT;
}

/* ================================================================
 * PER-CHANNEL VB POOLS
 *
 * A channel with a pool of its own draws blocks cut to its own frame; a
 * channel without one draws from the common pools, whose blocks are the
 * sensor's size because that is what hal_init knew. On this board the
 * difference is a 640x360 sub-stream frame costing 345,600 bytes instead
 * of 4,478,976 -- and with a 32 MB MMZ that difference is the whole
 * pipeline.
 *
 * V5 spells this differently from gen4 and the names are the only real
 * trap: ss_mpi_vpss_attach_chn_vb_pool, not AttachVbPool, and it does
 * nothing on its own until ss_mpi_vpss_set_chn_vb_src has moved the
 * channel to OT_VB_SRC_USER. A channel set to USER with no pool attached
 * has no blocks at all, so the two calls are made together or not at all.
 * ================================================================ */

/*
 * How many blocks one channel's pool holds. (Channel 0 normally holds
 * none: it streams through the wrap ring below, and takes a pool only
 * while a snapshot has its depth up.)
 *
 * One being written, one in flight, one held by the bound encoder. gen4
 * uses four; three is chosen here because this part's MMZ is the binding
 * constraint and a channel that turns out to want more says so plainly --
 * /proc/umap/vb reports min_free per pool, and the tuning guide's own
 * advice is to read it and subtract.
 */
#define HISI_FS_POOL_BLK_CNT 3u

/*
 * hisi_fs_pool_want -- the block size this channel needs, or 0 for none.
 *
 * Zero for a channel with no geometry yet, and zero when the frame is at
 * least as large as a common block: a pool that saves nothing is a pool
 * whose only effect is to fragment the zone.
 */
static unsigned long long hisi_fs_pool_want(const hisi_state_t *st, const hisi_vpss_chn_t *fs)
{
    unsigned long long want, common;

    if (!fs->width || !fs->height)
        return 0;
    /* A wrapped channel writes into the ring, not into frames. */
    if (fs->wrapped)
        return 0;

    want = hisi_vb_yuv_size(fs->width, fs->height, fs->compress_mode);
    common = hisi_vb_nv12_size(st->mode.dev_rect.width, st->mode.dev_rect.height);

    {
        /* local experiment: RSS_FS_POOL_MAX=<bytes> keeps bigger channels on the common pool */
        const char *ov = getenv("RSS_FS_POOL_MAX");
        if (ov && want > strtoull(ov, NULL, 10))
            return 0;
    }

    return want < common ? want : 0;
}

/*
 * hisi_fs_pool_acquire -- cut a pool to this channel's frame and attach it.
 *
 * Nothing here fails the caller, deliberately: a channel with no pool of
 * its own draws from the common pool, which is correct everywhere and
 * merely wasteful. That keeps the optimisation's failure mode the same
 * shape as the thing it replaced rather than introducing a new way for a
 * pipeline to refuse to start.
 */
static void hisi_fs_pool_acquire(hisi_state_t *st, int chn, hisi_vpss_chn_t *fs)
{
    v5_vb_pool_cfg cfg;
    unsigned long long size = hisi_fs_pool_want(st, fs);
    unsigned int pool;
    int phy = hisi_vpss_phy(chn);
    int ret;

    if (!st->vb_private_pools || !size)
        return;

    memset(&cfg, 0, sizeof(cfg));
    cfg.blk_size = size;
    cfg.blk_cnt = HISI_FS_POOL_BLK_CNT;
    cfg.remap_mode = V5_VB_REMAP_NONE;
    /* mmz_name left empty: the anonymous zone, which is where the common
     * pools live and the only one this board declares. */

    pool = st->vb.fnCreatePool(&cfg);
    if (pool == V5_VB_INVALID_POOL) {
        HAL_LOG_WARN("fs%d: ss_mpi_vb_create_pool(%llu x %u) failed; the channel will use the "
                     "common pool",
                     chn, cfg.blk_size, cfg.blk_cnt);
        return;
    }

    ret = st->vpss.fnSetChnVbSrc(HISI_VPSS_GRP, phy, V5_VB_SRC_USER);
    if (ret) {
        st->vb.fnDestroyPool(pool);
        /*
         * NOT_SUPPORT is the driver saying it has no such operation, not
         * this call being wrong, so stop asking: every later channel would
         * get the same answer and log the same line.
         */
        if (V5_ERR_ID(ret) == V5_ERR_NOT_SUPPORT) {
            st->vb_private_pools = false;
            HAL_LOG_INFO("vpss: this driver has no per-channel vb source (0x%x); channels take "
                         "their blocks from the common pool",
                         ret);
        } else {
            HAL_LOG_WARN("ss_mpi_vpss_set_chn_vb_src(%d, %d, USER) failed: 0x%x; the channel "
                         "will use the common pool",
                         HISI_VPSS_GRP, phy, ret);
        }
        return;
    }

    ret = st->vpss.fnAttachChnVbPool(HISI_VPSS_GRP, phy, pool);
    if (ret) {
        /* Put the channel back on common before dropping the pool, or it
         * is left asking a source that has nothing to give. */
        st->vpss.fnSetChnVbSrc(HISI_VPSS_GRP, phy, V5_VB_SRC_COMMON);
        st->vb.fnDestroyPool(pool);
        HAL_LOG_WARN("ss_mpi_vpss_attach_chn_vb_pool(%d, %d, %u) failed: 0x%x; the channel will "
                     "use the common pool",
                     HISI_VPSS_GRP, phy, pool, ret);
        return;
    }

    fs->vb_pool_owned = true;
    fs->vb_pool = pool;
    fs->vb_pool_blk_size = size;

    HAL_LOG_INFO("fs%d: vb pool %u, %llu B x%u for %ux%u%s", chn, pool, size, HISI_FS_POOL_BLK_CNT,
                 fs->width, fs->height,
                 fs->compress_mode == V5_COMPRESS_MODE_SEG_COMPACT ? " compressed" : "");
}

/*
 * hisi_fs_pool_release -- detach, put the source back, then free.
 *
 * In that order. The vendor's note on attach_chn_vb_pool is explicit that
 * the blocks must not still be in use by whatever VPSS is bound to when
 * the pool is destroyed, which is why every caller disables the channel
 * first.
 */
static void hisi_fs_pool_release(hisi_state_t *st, int chn, hisi_vpss_chn_t *fs)
{
    int phy = hisi_vpss_phy(chn);
    int ret;

    if (!fs->vb_pool_owned)
        return;

    if (st->vpss.fnDetachChnVbPool)
        st->vpss.fnDetachChnVbPool(HISI_VPSS_GRP, phy);
    if (st->vpss.fnSetChnVbSrc)
        st->vpss.fnSetChnVbSrc(HISI_VPSS_GRP, phy, V5_VB_SRC_COMMON);

    ret = st->vb.fnDestroyPool(fs->vb_pool);
    if (ret)
        HAL_LOG_WARN("fs%d: ss_mpi_vb_destroy_pool(%u) failed: 0x%x", chn, fs->vb_pool, ret);

    fs->vb_pool_owned = false;
    fs->vb_pool = 0;
    fs->vb_pool_blk_size = 0;
}

/* ================================================================
 * THE CHANNEL 0 WRAP RING
 *
 * VPSS physical channel 0 can feed its encoder from a ring of buf_line
 * lines instead of whole frames: ss_mpi_vpss_set_chn_buf_wrap, CV610
 * only, set between the channel attribute and the enable. While it is on
 * the channel takes one block from the common pools -- the one hal_init
 * cut in pool 1 for exactly this -- and needs no pool of its own, which
 * on this board is 6.2 MB of a 32 MB MMZ at 1080p, and the room the VI
 * pipe's 3DNR reference frames were refused for.
 *
 * What it costs, from the VPSS reference's list for this call: no flip
 * on the channel -- mirror is fine, flip is not, and hisi_fs_wrap_want
 * gives the ring up for as long as [image] asks for one -- no channel
 * post-processing, no low-delay and no buffer-share on the same channel,
 * and the bound encoders may not re-encode oversize frames -- none of
 * which this backend uses. A JPEG and an H.26x channel may both hang off
 * the wrapped channel, which is what rvd binds. What it cannot do is hand
 * a frame to userspace, and the depth that would ask for one is refused
 * rather than taken off the ring -- see hal_fs_set_frame_depth for the
 * measured reason.
 *
 * The line count is the driver's (ss_mpi_sys_get_vpss_venc_wrap_buf_line)
 * and the byte count is its formula (hisi_vb_wrap_size); the sizes are
 * remembered so a geometry change knows whether the ring has to be cut
 * again. Every failure here leaves the channel streaming from whole
 * frames, the way it did before the ring existed.
 * ================================================================ */

/*
 * hisi_fs_chn0_rings -- is the ring in play for channel 0 on this pipeline?
 *
 * Asked of the pipeline rather than of a channel, because it is what
 * decides whether a flip can be had at all: the ring is exclusive with
 * flip, and giving it up is not affordable here. See hal_isp_set_vflip.
 */
bool hisi_fs_chn0_rings(const hisi_state_t *st)
{
    return st->vb_wrap_blk && st->vpss.fnSetChnBufWrap && st->sys.fnGetVpssVencWrapBufLine;
}

static bool hisi_fs_wrap_want(const hisi_state_t *st, const hisi_vpss_chn_t *fs, int phy)
{
    /*
     * Not while the picture is flipped. The VPSS reference's wrap table
     * gives mirror as supported in every wrapped coupling and flip in
     * none, and the driver agrees: measured on a CV608, an attribute
     * carrying flip_en is accepted on the disabled channel and the
     * following ss_mpi_vpss_set_chn_buf_wrap(on) is refused 0xa007800d.
     * Mirror keeps the ring; flip costs it. hal_isp_set_vflip refuses the
     * flip that would, so in practice this term never fires; it is here
     * so the two statements of the rule cannot drift apart.
     */
    return phy == 0 && !fs->depth && !hisi_fs_vpss_flip(st) && hisi_fs_chn0_rings(st);
}

/* The ring this channel would ask for now. False when it cannot be had. */
static bool hisi_fs_wrap_attr(const hisi_state_t *st, int chn, const hisi_vpss_chn_t *fs,
                              v5_vpss_chn_buf_wrap_attr *w)
{
    v5_vpss_venc_wrap_param p;
    unsigned int line = 0;
    int ret;

    memset(&p, 0, sizeof(p));
    hisi_wrap_param(&p, st->vi_all_online ? 1 : 0,
                    st->mode.frame_rate > 1.0f ? (unsigned int)(st->mode.frame_rate + 0.5f) : 30u,
                    fs->width, fs->height);

    ret = st->sys.fnGetVpssVencWrapBufLine(&p, &line);
    if (ret || !line) {
        HAL_LOG_WARN("fs%d: ss_mpi_sys_get_vpss_venc_wrap_buf_line -> 0x%x, %u lines; streaming "
                     "from whole frames",
                     chn, ret, line);
        return false;
    }
    if (line > fs->height)
        line = fs->height;

    memset(w, 0, sizeof(*w));
    w->enable = 1;
    w->buf_line = line;
    w->buf_size = (unsigned int)hisi_vb_wrap_size(fs->width, fs->height, line, fs->compress_mode);
    if (w->buf_size > st->vb_wrap_blk) {
        HAL_LOG_WARN("fs%d: a %u-line ring is %u B and pool 1's block is %llu; streaming from "
                     "whole frames",
                     chn, line, w->buf_size, st->vb_wrap_blk);
        return false;
    }
    return true;
}

/*
 * hisi_fs_wrap_apply -- put the channel on the ring, or take it off.
 *
 * The channel has to be disabled and its attribute already set; every
 * caller is on that path. A failed enable is a warning and an unwrapped
 * channel, never an error to the caller.
 */
static void hisi_fs_wrap_apply(hisi_state_t *st, int chn, hisi_vpss_chn_t *fs, bool want)
{
    v5_vpss_chn_buf_wrap_attr w;
    int phy = hisi_vpss_phy(chn);
    int ret;

    /* local experiment: RSS_NO_WRAP=1 keeps every channel off the wrap ring */
    if (want && getenv("RSS_NO_WRAP"))
        want = false;

    if (want && !hisi_fs_wrap_attr(st, chn, fs, &w))
        want = false;

    if (!want) {
        if (fs->wrapped && st->vpss.fnSetChnBufWrap) {
            memset(&w, 0, sizeof(w));
            ret = st->vpss.fnSetChnBufWrap(HISI_VPSS_GRP, phy, &w);
            if (ret)
                HAL_LOG_WARN("fs%d: ss_mpi_vpss_set_chn_buf_wrap(off) failed: 0x%x", chn, ret);
            else
                HAL_LOG_INFO("fs%d: wrap off; streaming from whole frames", chn);
        }
        fs->wrapped = false;
        fs->wrap_line = 0;
        fs->wrap_size = 0;
        return;
    }

    /*
     * Measured: a ring turned off and back on in the same call --
     * which a geometry change on channel 0 has to do, the attribute
     * being refused while the ring is on -- answers 0xa0078014 (NO_MEM)
     * the first time and succeeds 20 ms later. The block is back in the
     * pool by then (/proc/umap/vb shows it free); whatever the driver
     * tears down on the way out is not quite gone. So: a few tries, a
     * blink apart.
     */
    {
        int tries = 10;

        for (;;) {
            ret = st->vpss.fnSetChnBufWrap(HISI_VPSS_GRP, phy, &w);
            if (!ret || V5_ERR_ID(ret) != V5_ERR_NO_MEM || --tries <= 0)
                break;
            {
                struct timespec blink = {0, 20000000L};

                nanosleep(&blink, NULL);
            }
        }
    }
    if (ret) {
        HAL_LOG_WARN("fs%d: ss_mpi_vpss_set_chn_buf_wrap(%u lines, %u B) failed: 0x%x; streaming "
                     "from whole frames",
                     chn, w.buf_line, w.buf_size, ret);
        fs->wrapped = false;
        fs->wrap_line = 0;
        fs->wrap_size = 0;
        return;
    }
    fs->wrapped = true;
    fs->wrap_line = w.buf_line;
    fs->wrap_size = w.buf_size;
    HAL_LOG_INFO("fs%d: wrapped: a %u-line ring of %u B for %ux%u%s, out of common pool 1; no "
                 "private pool",
                 chn, w.buf_line, w.buf_size, fs->width, fs->height,
                 fs->compress_mode == V5_COMPRESS_MODE_SEG_COMPACT ? " compressed" : "");
}

/*
 * hisi_fs_pool_refresh -- keep the pool matched to a geometry that changed.
 *
 * A channel that grew past its own pool's blocks would otherwise stay
 * attached to a pool that can never satisfy it and stop producing without
 * logging anything. Only an actual change in what the channel needs cycles
 * it, so the fps-only reconfigures rvd issues while streaming do not touch
 * the channel.
 *
 * The channel has to be down while its pool is swapped, so this disables
 * and re-enables around the swap and leaves the channel as it found it.
 * `attr` is the channel attribute to set inside that window, for the
 * caller that could not set it live; NULL when it already has.
 */
static void hisi_fs_pool_refresh(hisi_state_t *st, int chn, hisi_vpss_chn_t *fs,
                                 const v5_vpss_chn_attr *attr)
{
    unsigned long long want;
    bool was_enabled, want_wrap, wrap_change;
    int phy = hisi_vpss_phy(chn);
    int ret;

    /*
     * The ring first: whether the channel should be on one now, and if it
     * already is, whether the geometry it was cut for is still the
     * geometry. Only then can the pool question be asked, because a
     * wrapped channel wants no pool at all.
     */
    want_wrap = hisi_fs_wrap_want(st, fs, phy);
    wrap_change = want_wrap != fs->wrapped;
    if (want_wrap && fs->wrapped) {
        v5_vpss_chn_buf_wrap_attr w;

        wrap_change = !hisi_fs_wrap_attr(st, chn, fs, &w) || w.buf_size != fs->wrap_size ||
                      w.buf_line != fs->wrap_line;
    }

    /* An attribute to set is always a reason to cycle: it is the fps-only
     * reconfigure on a wrapped channel, which changes no block. */
    if (!attr && !st->vb_private_pools && !wrap_change)
        return;

    want = wrap_change ? 0 : hisi_fs_pool_want(st, fs);
    if (!attr && !wrap_change && want == (fs->vb_pool_owned ? fs->vb_pool_blk_size : 0ull))
        return;

    was_enabled = fs->enabled;
    if (was_enabled) {
        ret = st->vpss.fnDisableChn(HISI_VPSS_GRP, phy);
        if (ret) {
            HAL_LOG_WARN("fs%d: disable_chn failed 0x%x; keeping the old vb pool", chn, ret);
            return;
        }
        fs->enabled = false;
    }

    /*
     * Ring off, attribute, ring on -- in that order. A channel on the ring
     * refuses a new attribute whether enabled or not (0xa007800d,
     * NOT_PERM, measured both ways), and the reference wants the ring set
     * after the attribute; so the ring comes off first whenever there is
     * an attribute to set or a ring to re-cut, and goes back on after.
     */
    if (fs->wrapped && (wrap_change || attr))
        hisi_fs_wrap_apply(st, chn, fs, false);
    if (attr) {
        ret = st->vpss.fnSetChnAttr(HISI_VPSS_GRP, phy, attr);
        if (ret)
            HAL_LOG_ERR("fs%d: ss_mpi_vpss_set_chn_attr while cycling failed: 0x%x", chn, ret);
    }
    if (want_wrap && !fs->wrapped)
        hisi_fs_wrap_apply(st, chn, fs, true);
    hisi_fs_pool_release(st, chn, fs);
    hisi_fs_pool_acquire(st, chn, fs);

    if (was_enabled) {
        ret = st->vpss.fnEnableChn(HISI_VPSS_GRP, phy);
        if (ret) {
            HAL_LOG_ERR("fs%d: enable_chn after a vb pool change failed: 0x%x", chn, ret);
            return;
        }
        fs->enabled = true;
    }
}

/*
 * hisi_fs_fill_attr -- the channel attribute from raptor's config.
 *
 * USER mode, always. AUTO takes the channel's geometry from the group,
 * which would give three streams at the sensor's size and no error to say
 * so.
 *
 * YVU_SEMIPLANAR_420 (NV21) matches the group and the VI channel; the
 * neighbouring YUV_SEMIPLANAR_420 (NV12) differs only in chroma order, so
 * choosing wrong costs swapped colours rather than a failure.
 */
static void hisi_fs_fill_attr(const hisi_state_t *st, hisi_vpss_chn_t *fs, int phy,
                              v5_vpss_chn_attr *attr)
{
    memset(attr, 0, sizeof(*attr));

    /* Cached on the channel because the pool's block size has to agree
     * with it; hisi_fs_pool_want reads it back. */
    fs->compress_mode = hisi_fs_compress(st, fs, phy);

    attr->width = fs->width;
    attr->height = fs->height;
    attr->depth = fs->depth;
    attr->chn_mode = V5_VPSS_CHN_MODE_USER;
    attr->video_format = V5_VIDEO_FORMAT_LINEAR;
    attr->dynamic_range = V5_DYNAMIC_RANGE_SDR8;
    attr->pixel_format = V5_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    attr->compress_mode = fs->compress_mode;
    attr->frame_rate = fs->frame_rate;
    attr->border_en = 0;
    attr->aspect_ratio.mode = V5_ASPECT_RATIO_NONE;

    /*
     * Mirror and flip are the channel's share of the orientation -- the
     * whole of it when the sensor cannot turn the picture, none of it
     * when it can (hisi_fs_vpss_mirror). Not the VI channel's, which in
     * the all-online coupling writes nothing to DDR and so has no
     * write-out to reverse: measured on a CV608, the VI channel takes
     * mirror_en and reports it in /proc/umap/vi and the picture does not
     * move.
     *
     * They are per-stream here, which is what raptor wants, and they
     * follow the backend's orientation state rather than the caller's
     * config: [image] hflip turns the camera, not one stream.
     *
     * Two costs, both handled above rather than here. hisi_fs_compress
     * drops channel 0's SEG_COMPACT while either is on, because the driver
     * refuses an oriented attribute on a compressed channel; and
     * hisi_fs_wrap_want gives up the ring while flip is on, because the
     * ring refuses to come back on over a flipped channel.
     */
    attr->mirror_en = hisi_fs_vpss_mirror(st);
    attr->flip_en = hisi_fs_vpss_flip(st);
}

/*
 * hal_fs_create_channel -- configure and enable one VPSS channel.
 *
 * Enable is part of create rather than only of hal_fs_enable_channel, and
 * that is deliberate: rvd's pipeline calls create then bind then start,
 * and never calls enable for a stream it is about to bind. A channel that
 * is configured but not enabled produces nothing, with the group running
 * and /proc/umap/vpss showing the channel at 0x0.
 */
int hal_fs_create_channel(void *ctx, int chn, const rss_fs_config_t *cfg)
{
    hisi_state_t *st = hisi_state(ctx);
    hisi_vpss_chn_t *fs = hisi_fs_chn(ctx, chn);
    v5_vpss_chn_attr attr;
    int phy;
    int ret;

    if (!st || !fs || !cfg)
        return RSS_ERR_INVAL;
    if (!st->vpss_grp_created) {
        HAL_LOG_ERR("fs%d: VPSS group %d is not up", chn, HISI_VPSS_GRP);
        return RSS_ERR_NOENT;
    }
    if (!cfg->width || !cfg->height) {
        HAL_LOG_ERR("fs%d: %ux%u is not a size", chn, cfg->width, cfg->height);
        return RSS_ERR_INVAL;
    }
    hisi_fs_orien_guard(st);
    if (cfg->width > st->mode.dev_rect.width || cfg->height > st->mode.dev_rect.height) {
        /*
         * VPSS scales down, not up: the group was created for the sensor's
         * output and a channel cannot ask for more. Rejecting here rather
         * than letting set_chn_attr fail makes the message name both
         * numbers.
         */
        HAL_LOG_ERR("fs%d: %ux%u is larger than the group's %ux%u", chn, cfg->width, cfg->height,
                    st->mode.dev_rect.width, st->mode.dev_rect.height);
        return RSS_ERR_INVAL;
    }

    fs->width = cfg->width;
    fs->height = cfg->height;
    hisi_fs_frame_rate(st, cfg->fps_num, cfg->fps_den, &fs->frame_rate);
    phy = hisi_vpss_phy(chn);
    hisi_fs_fill_attr(st, fs, phy, &attr);

    ret = st->vpss.fnSetChnAttr(HISI_VPSS_GRP, phy, &attr);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_vpss_set_chn_attr(grp %d, chn %d) failed: 0x%x", HISI_VPSS_GRP, phy,
                    ret);
        return RSS_ERR_IO;
    }
    fs->configured = true;

    /* Before the enable: the ring has to be set while the channel is down,
     * and a channel that comes up on the common pool and is moved to its
     * own afterwards has to be cycled to do it. The ring first, because a
     * wrapped channel wants no pool. */
    hisi_fs_wrap_apply(st, chn, fs, hisi_fs_wrap_want(st, fs, phy));
    hisi_fs_pool_acquire(st, chn, fs);

    ret = st->vpss.fnEnableChn(HISI_VPSS_GRP, phy);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_vpss_enable_chn(grp %d, chn %d) failed: 0x%x", HISI_VPSS_GRP, phy, ret);
        return RSS_ERR_IO;
    }
    fs->enabled = true;

    HAL_LOG_INFO("fs%d: VPSS %d/%d %ux%u, rate %d/%d, depth %u", chn, HISI_VPSS_GRP, phy, fs->width,
                 fs->height, fs->frame_rate.src_frame_rate, fs->frame_rate.dst_frame_rate,
                 fs->depth);
    return RSS_OK;
}

/*
 * hisi_fs_apply_orien -- put st->mirror / st->flip onto the channels.
 *
 * The fallback path, for a sensor without orientation registers the
 * backend knows (hal_knob.c's orientation section has the whole account
 * of where a turn can live on this part). Set on all the channels
 * together because [image] hflip means the camera rather than one stream.
 *
 * WHAT IT COSTS. Channel 0 alone is expensive, and only because of how it
 * streams: it is SEG_COMPACT compressed and it feeds its encoder from the
 * wrap ring. Compression is mutually exclusive with orientation, so that
 * goes (hisi_fs_compress) and the ring is simply re-cut a little larger --
 * which is all a mirror costs. The ring is mutually exclusive with flip
 * alone, and giving it up is not affordable: see hal_isp_set_vflip, which
 * refuses that flip outright, and hisi_fs_orien_guard, which drops one
 * that arrived before the ring was settled.
 */
/*
 * hisi_fs_orien_guard -- drop a flip channel 0 cannot have.
 *
 * hal_isp_set_vflip refuses it, so this is the case that got past that:
 * an [image] vflip remembered before hal_init had settled whether channel
 * 0 would ride the ring. Dropping it here rather than at bring-up is the
 * difference between a mirrored picture and no pipeline at all -- channel
 * 0 leaving the ring takes three sensor-sized frames out of the zone, and
 * the encoders are created after the framesources, so what it starves
 * fails later and fatally.
 */
void hisi_fs_orien_guard(hisi_state_t *st)
{
    if (!hisi_fs_vpss_flip(st) || !hisi_fs_chn0_rings(st))
        return;
    HAL_LOG_ERR("vflip dropped: channel 0's wrap ring is exclusive with flip, and the frames it "
                "would need instead do not fit; see hal_isp_set_vflip");
    st->flip = 0;
}

int hisi_fs_apply_orien(hisi_state_t *st)
{
    int chn;
    int rc = RSS_OK;

    if (!st)
        return RSS_ERR_INVAL;
    if (!st->vpss_grp_created || !st->vpss.fnSetChnAttr) {
        HAL_LOG_DBG("orientation: mirror %d, flip %d noted for bring-up", st->mirror, st->flip);
        return RSS_OK;
    }

    for (chn = 0; chn < HISI_VPSS_CHN_NUM; chn++) {
        hisi_vpss_chn_t *fs = &st->fs[chn];
        v5_vpss_chn_attr attr;
        v5_compress_mode was;
        int phy = hisi_vpss_phy(chn);
        int ret;

        if (!fs->configured)
            continue;

        was = fs->compress_mode;
        hisi_fs_fill_attr(st, fs, phy, &attr);

        /*
         * A wrapped channel refuses a new attribute outright, and a
         * channel whose compression or ring just changed has a block size
         * that no longer matches its pool. Either way the cycle is the
         * path that can do it; a plain channel takes the attribute live,
         * and keeps streaming across the change.
         */
        if (fs->wrapped || fs->compress_mode != was ||
            hisi_fs_wrap_want(st, fs, phy) != fs->wrapped) {
            hisi_fs_pool_refresh(st, chn, fs, &attr);
            continue;
        }

        ret = st->vpss.fnSetChnAttr(HISI_VPSS_GRP, phy, &attr);
        if (ret) {
            HAL_LOG_ERR("fs%d: ss_mpi_vpss_set_chn_attr(mirror %d, flip %d) failed: 0x%x", chn,
                        st->mirror, st->flip, ret);
            if (rc == RSS_OK)
                rc = RSS_ERR_IO;
        }
    }

    if (rc == RSS_OK)
        HAL_LOG_INFO("orientation: mirror %d, flip %d on the VPSS channels",
                     hisi_fs_vpss_mirror(st), hisi_fs_vpss_flip(st));
    return rc;
}

/*
 * hal_fs_set_channel_attr -- geometry or rate on a live channel.
 *
 * VPSS takes a new channel attribute while the group is running, which is
 * what makes a resolution change cheap here: no destroy, no rebind, and
 * the encoder on the far side of the bind sees a differently sized frame
 * on its next one.
 */
int hal_fs_set_channel_attr(void *ctx, int chn, const rss_fs_config_t *cfg)
{
    hisi_state_t *st = hisi_state(ctx);
    hisi_vpss_chn_t *fs = hisi_fs_chn(ctx, chn);
    v5_vpss_chn_attr attr;
    int phy;
    int ret;

    if (!st || !fs || !cfg)
        return RSS_ERR_INVAL;
    if (!fs->configured)
        return hal_fs_create_channel(ctx, chn, cfg);
    if (!cfg->width || !cfg->height)
        return RSS_ERR_INVAL;
    if (cfg->width > st->mode.dev_rect.width || cfg->height > st->mode.dev_rect.height)
        return RSS_ERR_INVAL;

    fs->width = cfg->width;
    fs->height = cfg->height;
    hisi_fs_frame_rate(st, cfg->fps_num, cfg->fps_den, &fs->frame_rate);
    phy = hisi_vpss_phy(chn);
    hisi_fs_fill_attr(st, fs, phy, &attr);

    if (fs->wrapped) {
        /* Refused live on a wrapped channel; set inside the cycle, where
         * the ring is re-cut for the new geometry as well. */
        hisi_fs_pool_refresh(st, chn, fs, &attr);
        return RSS_OK;
    }

    ret = st->vpss.fnSetChnAttr(HISI_VPSS_GRP, phy, &attr);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_vpss_set_chn_attr(grp %d, chn %d) failed: 0x%x", HISI_VPSS_GRP, phy,
                    ret);
        return RSS_ERR_IO;
    }

    /* A new geometry is a new block size. No-op when the size did not
     * actually change, which is the fps-only reconfigure. */
    hisi_fs_pool_refresh(st, chn, fs, NULL);

    HAL_LOG_INFO("fs%d: now %ux%u, rate %d/%d", chn, fs->width, fs->height,
                 fs->frame_rate.src_frame_rate, fs->frame_rate.dst_frame_rate);
    return RSS_OK;
}

int hal_fs_enable_channel(void *ctx, int chn)
{
    hisi_state_t *st = hisi_state(ctx);
    hisi_vpss_chn_t *fs = hisi_fs_chn(ctx, chn);
    int phy;
    int ret;

    if (!st || !fs)
        return RSS_ERR_INVAL;
    if (!fs->configured)
        return RSS_ERR_NOENT;
    if (fs->enabled)
        return RSS_OK;

    phy = hisi_vpss_phy(chn);
    ret = st->vpss.fnEnableChn(HISI_VPSS_GRP, phy);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_vpss_enable_chn(grp %d, chn %d) failed: 0x%x", HISI_VPSS_GRP, phy, ret);
        return RSS_ERR_IO;
    }
    fs->enabled = true;
    return RSS_OK;
}

/*
 * hal_fs_disable_channel -- stop delivering, keep the attribute.
 *
 * A frame checked out through fs_get_frame is released first. Disabling a
 * channel with one outstanding leaves the block accounted to a channel
 * that no longer runs, and VB then reports it as in use until the group is
 * destroyed.
 */
int hal_fs_disable_channel(void *ctx, int chn)
{
    hisi_state_t *st = hisi_state(ctx);
    hisi_vpss_chn_t *fs = hisi_fs_chn(ctx, chn);
    int phy;
    int ret;

    if (!st || !fs)
        return RSS_ERR_INVAL;
    if (!fs->enabled)
        return RSS_OK;

    phy = hisi_vpss_phy(chn);

    if (fs->frame_held) {
        if (st->vpss.fnReleaseChnFrame)
            st->vpss.fnReleaseChnFrame(HISI_VPSS_GRP, phy, &fs->frame);
        fs->frame_held = false;
    }

    ret = st->vpss.fnDisableChn(HISI_VPSS_GRP, phy);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_vpss_disable_chn(grp %d, chn %d) failed: 0x%x", HISI_VPSS_GRP, phy,
                    ret);
        return RSS_ERR_IO;
    }
    fs->enabled = false;
    return RSS_OK;
}

/*
 * hal_fs_destroy_channel -- there is no ss_mpi_vpss_destroy_chn.
 *
 * Disable is the whole of it. The bookkeeping is cleared so a later create
 * writes the attribute again rather than trusting what the driver still
 * holds.
 */
int hal_fs_destroy_channel(void *ctx, int chn)
{
    hisi_state_t *st = hisi_state(ctx);
    hisi_vpss_chn_t *fs = hisi_fs_chn(ctx, chn);
    int ret;

    if (!st || !fs)
        return RSS_ERR_INVAL;

    ret = hal_fs_disable_channel(ctx, chn);
    if (ret)
        return ret;

    /* After the disable, so the pool's blocks are back before it is
     * destroyed; the ring's block goes back to common the same way. */
    hisi_fs_wrap_apply(st, chn, fs, false);
    hisi_fs_pool_release(st, chn, fs);

    memset(fs, 0, sizeof(*fs));
    return RSS_OK;
}

/*
 * hal_fs_set_rotation -- turn one channel's output.
 *
 * ss_mpi_vpss_set_chn_rotation takes an ot_rotation_attr the backend does
 * not transcribe (Phase 3 owns rotation properly), so the op reports what
 * it can do rather than pretending: 0 is accepted because it is the state
 * the channel is already in, and anything else is RSS_ERR_NOTSUP with a
 * line saying why.
 */
int hal_fs_set_rotation(void *ctx, int chn, int degrees)
{
    hisi_vpss_chn_t *fs = hisi_fs_chn(ctx, chn);

    if (!fs)
        return RSS_ERR_INVAL;
    if (degrees == 0) {
        fs->rotation = 0;
        return RSS_OK;
    }

    HAL_LOG_WARN("fs%d: rotation %d needs ot_rotation_attr, which this backend does not yet "
                 "transcribe",
                 chn, degrees);
    return RSS_ERR_NOTSUP;
}

/*
 * hal_fs_set_frame_depth -- how many frames the channel queues for us.
 *
 * Zero is the streaming case: the channel feeds its bound VENC and queues
 * nothing. rvd raises it to take a snapshot by hand, and with depth 0
 * get_chn_frame would block to its timeout on a channel that is working
 * perfectly -- which is why this is a real op and not bookkeeping.
 *
 * Every frame of depth is a frame of VB held by this channel, so the
 * bound is the vendor's 8 and the default stays 0.
 */
#define HISI_FS_MAX_DEPTH 8

int hal_fs_set_frame_depth(void *ctx, int chn, int depth)
{
    hisi_state_t *st = hisi_state(ctx);
    hisi_vpss_chn_t *fs = hisi_fs_chn(ctx, chn);
    v5_vpss_chn_attr attr;
    int phy;
    int ret;

    if (!st || !fs)
        return RSS_ERR_INVAL;
    if (depth < 0 || depth > HISI_FS_MAX_DEPTH)
        return RSS_ERR_INVAL;
    if ((unsigned int)depth == fs->depth)
        return RSS_OK;

    if (!fs->configured) {
        fs->depth = (unsigned int)depth;
        return RSS_OK;
    }

    fs->depth = (unsigned int)depth;
    phy = hisi_vpss_phy(chn);
    hisi_fs_fill_attr(st, fs, phy, &attr);

    /*
     * A depth on the wrapped channel used to be refused here. It is not
     * any more, because the all-online coupling changed the arithmetic
     * behind the refusal: the three uncompressed 1080p frames a depth
     * needs are 9.3 MB, and where the offline coupling left 4.8 MB of the
     * zone free the all-online one leaves 17.9. What the refusal was
     * protecting against -- the channel falling back to sensor-sized
     * blocks and VI failing 73 allocations in two seconds -- cannot happen
     * from a pool that fits.
     *
     * So the ring comes off for the duration instead. hisi_fs_pool_refresh
     * is the path that can do it: a wrapped channel refuses a new
     * attribute (0xa007800d), and refresh knows to disable, unwrap, set,
     * re-pool and enable in that order. hisi_fs_wrap_want already answers
     * false for a channel with a depth, so this direction takes the ring
     * off and the return to depth 0 puts it back.
     */
    if (fs->wrapped || hisi_fs_wrap_want(st, fs, phy) != fs->wrapped) {
        hisi_fs_pool_refresh(st, chn, fs, &attr);
        HAL_LOG_INFO("fs%d: depth %u, %s", chn, fs->depth,
                     fs->wrapped ? "back on the wrap ring" : "off the wrap ring, on its own pool");
        return RSS_OK;
    }

    ret = st->vpss.fnSetChnAttr(HISI_VPSS_GRP, phy, &attr);
    if (ret) {
        HAL_LOG_ERR("fs%d: depth %d rejected: 0x%x", chn, depth, ret);
        return RSS_ERR_IO;
    }

    /*
     * Raising the depth turns compression off on channel 0 and lowering it
     * turns it back on -- see hisi_fs_compress -- and either way the
     * block size just changed underneath the pool. (A channel that was
     * never wrapped, that is; one on the ring was refused above.)
     */
    hisi_fs_pool_refresh(st, chn, fs, NULL);

    return RSS_OK;
}

int hal_fs_get_frame_depth(void *ctx, int chn, int *depth)
{
    hisi_vpss_chn_t *fs = hisi_fs_chn(ctx, chn);

    if (!fs || !depth)
        return RSS_ERR_INVAL;
    *depth = (int)fs->depth;
    return RSS_OK;
}

/* Give back whatever hal_fs_get_frame mapped, if it mapped anything. */
static void hisi_fs_frame_unmap(hisi_state_t *st, hisi_vpss_chn_t *fs)
{
    if (!fs->frame_map)
        return;
    if (st->sys.fnMunmap)
        st->sys.fnMunmap(fs->frame_map, fs->frame_map_size);
    fs->frame_map = NULL;
    fs->frame_map_phys = 0;
    fs->frame_map_size = 0;
}

/*
 * hal_fs_get_frame -- check out one YUV frame.
 *
 * MPP wants the same descriptor back, so it lives in the channel's state
 * and the caller is handed a pointer into it. One outstanding frame per
 * channel: rvd's snapshot path takes one, uses it and gives it back, and a
 * second concurrent take would need a second descriptor for no caller that
 * exists.
 */
#define HISI_FS_FRAME_TIMEOUT_MS 2000

int hal_fs_get_frame(void *ctx, int chn, void **frame_data, rss_frame_info_t *info)
{
    hisi_state_t *st = hisi_state(ctx);
    hisi_vpss_chn_t *fs = hisi_fs_chn(ctx, chn);
    int phy;
    int ret;

    if (!st || !fs || !frame_data)
        return RSS_ERR_INVAL;
    if (!st->vpss.fnGetChnFrame)
        return RSS_ERR_NOTSUP;
    if (!fs->enabled)
        return RSS_ERR_NOENT;
    if (fs->frame_held) {
        HAL_LOG_ERR("fs%d: a frame is already checked out", chn);
        return RSS_ERR_BUSY;
    }
    if (!fs->depth) {
        HAL_LOG_ERR("fs%d: depth is 0, so no frame is ever queued for userspace -- raise it with "
                    "fs_set_frame_depth first",
                    chn);
        return RSS_ERR_INVAL;
    }

    phy = hisi_vpss_phy(chn);
    memset(&fs->frame, 0, sizeof(fs->frame));
    ret = st->vpss.fnGetChnFrame(HISI_VPSS_GRP, phy, &fs->frame, HISI_FS_FRAME_TIMEOUT_MS);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_vpss_get_chn_frame(grp %d, chn %d) failed: 0x%x", HISI_VPSS_GRP, phy,
                    ret);
        return RSS_ERR_IO;
    }
    fs->frame_held = true;

    *frame_data = &fs->frame;
    if (info) {
        const v5_video_frame *f = &fs->frame.video_frame;
        unsigned int luma = f->stride[0] * f->height;
        unsigned int chroma = f->stride[1] ? f->stride[1] * (f->height / 2u) : luma / 2u;

        memset(info, 0, sizeof(*info));
        info->width = (uint16_t)f->width;
        info->height = (uint16_t)f->height;
        info->pixfmt = f->pixel_format == V5_PIXEL_FORMAT_YVU_SEMIPLANAR_420 ? RSS_PIXFMT_NV21
                                                                             : RSS_PIXFMT_NV12;
        info->timestamp = (int64_t)f->pts;
        info->phys_addr = f->phys_addr[0];

        /*
         * The two planes are one allocation and the chroma plane's own
         * physical address says where in it; a stride-derived guess would
         * be wrong the moment the driver pads between them.
         */
        info->size = f->phys_addr[1] > f->phys_addr[0]
                         ? (f->phys_addr[1] - f->phys_addr[0]) + chroma
                         : luma + chroma;

        /*
         * REMAP_NONE pools hand back a null virt_addr, so the block is
         * mapped here for the one read the caller is about to do and
         * unmapped in hal_fs_release_frame. Cached, then flushed: the
         * snapshot copies megabytes and an uncached read of that is slow
         * enough to notice.
         */
        info->virt_addr = f->virt_addr[0];
        if (!info->virt_addr && st->sys.fnMmapCached && info->size) {
            void *virt = st->sys.fnMmapCached(info->phys_addr, info->size);

            if (virt) {
                if (st->sys.fnFlushCache)
                    st->sys.fnFlushCache(info->phys_addr, virt, info->size);
                fs->frame_map = virt;
                fs->frame_map_phys = info->phys_addr;
                fs->frame_map_size = info->size;
                info->virt_addr = virt;
            } else {
                HAL_LOG_WARN("fs%d: ss_mpi_sys_mmap_cached(0x%08x, %u) failed; the caller gets a "
                             "frame it cannot read",
                             chn, info->phys_addr, info->size);
            }
        }
    }

    return RSS_OK;
}

int hal_fs_release_frame(void *ctx, int chn, void *frame_data)
{
    hisi_state_t *st = hisi_state(ctx);
    hisi_vpss_chn_t *fs = hisi_fs_chn(ctx, chn);
    int phy;
    int ret;

    if (!st || !fs)
        return RSS_ERR_INVAL;
    if (!fs->frame_held)
        return RSS_OK;
    /* The pointer is the caller's proof it holds this channel's frame; a
     * mismatch is a caller bug and releasing anyway would hide it. */
    if (frame_data && frame_data != &fs->frame)
        return RSS_ERR_INVAL;

    hisi_fs_frame_unmap(st, fs);

    phy = hisi_vpss_phy(chn);
    ret =
        st->vpss.fnReleaseChnFrame ? st->vpss.fnReleaseChnFrame(HISI_VPSS_GRP, phy, &fs->frame) : 0;
    fs->frame_held = false;

    if (ret) {
        HAL_LOG_ERR("ss_mpi_vpss_release_chn_frame(grp %d, chn %d) failed: 0x%x", HISI_VPSS_GRP,
                    phy, ret);
        return RSS_ERR_IO;
    }
    return RSS_OK;
}

/*
 * hisi_fs_release_all -- teardown's entry point.
 *
 * Called from the pipeline teardown before the group is stopped, because
 * ss_mpi_vpss_stop_grp with a channel still enabled returns busy and the
 * whole teardown then unwinds in the wrong order.
 */
void hisi_fs_release_all(hisi_state_t *st)
{
    int chn;

    if (!st)
        return;

    for (chn = 0; chn < HISI_VPSS_CHN_NUM; chn++) {
        hisi_vpss_chn_t *fs = &st->fs[chn];
        int phy = hisi_vpss_phy(chn);

        if (fs->frame_held) {
            hisi_fs_frame_unmap(st, fs);
            if (st->vpss.fnReleaseChnFrame)
                st->vpss.fnReleaseChnFrame(HISI_VPSS_GRP, phy, &fs->frame);
            fs->frame_held = false;
        }
        if (fs->enabled && st->vpss.fnDisableChn)
            st->vpss.fnDisableChn(HISI_VPSS_GRP, phy);
        fs->enabled = false;
        hisi_fs_pool_release(st, chn, fs);
        memset(fs, 0, sizeof(*fs));
    }
}
