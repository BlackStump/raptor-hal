/*
 * xiongmai/xm_isp.c -- the ISP library's attributes: exposure and saturation.
 *
 * Its own file because it speaks the ISP library's headers
 * (openxiongmai/libraries/isp/include/isp, added for this object only by
 * mk/xiongmai.mk), which carry their own copies of the XM_* types and are
 * not meant to meet the MPI's re_*.h in one translation unit.
 *
 * EXPOSURE. The open AE (libisp_XM530, LIBISP_OPEN_AE=1) reads its limits
 * and target from XM_MPI_ISP_GetExposureAttr, which libxmhost answers from
 * a store. Majestic used to fill that store; here the backend does:
 *   - automatic, every zone weighted 1 (libxmhost's own start);
 *   - the system gain at most 128x. That is where the vendor AE stopped on
 *     the XM550 (openxiongmai #94); left open, the SC5332's tables reach
 *     488x and the night picture is all noise;
 *   - the compensation 0, which the open AE reads as the sensor's own
 *     target (0x39 = 57 on the SC5332, where majestic held 55 in a lit
 *     room). [image] ae_comp sets it, 1..255; auto puts 0 back.
 * Majestic also lowered the compensation as it got dark (55 to 33). That
 * needs the AE's gain read back and belongs with day/night.
 *
 * SATURATION. The open ISP's tuning pass turns ISP_SATURATION_ATTR_S into
 * the Cb/Cr gain: 0x100 + 6 x (s - 128) above 128, unity at or below. So
 * [image] saturation, 0..255 as on every raptor platform, raises colour
 * above 128 and is neutral below it -- the hardware path has no
 * desaturation short of greyscale. Auto hands back the sensor's per-gain
 * table. Majestic's camera_set_color worked on the chroma block
 * (0x30071000) instead; matching that is the next step.
 *
 * MIRROR AND FLIP. The sensor's: XM_MPI_ISP_SetChnAttr in sensor mode hands
 * them to the sensor library (the SC5332's sets 0x3221), which moves the
 * ISP's input crop a Bayer step with them (libispapp's VI_WinSet: 8 pixels
 * across, 4 down) and refreshes the MIPI frame. The output window is cut
 * after the ISP, so it is the same with them as without. Nothing is called
 * while both stay off: VI_WinSet's first call also sets the pixel-clock
 * edge, which nothing has needed while the picture is the right way up.
 *
 * ANTI-FLICKER. The open AE's own: with stAntiflicker set it holds the
 * exposure to whole mains periods. isp_set_antiflicker sets it, kept
 * across every exposure write.
 *
 * DAY AND NIGHT. On the IVG-83X50PSA-B the LED board's photosensor moves
 * the IR-cut filter and lights the IR LEDs itself; ric only follows with
 * the picture. Night is black and white -- CSC saturation 0, which the
 * open ISP turns into Cb/Cr gains of 0 at once and keeps there -- and day
 * puts back the CSC saturation that was there. ric decides from
 * isp_get_exposure: lines x total gain, the AE's average luminance (by day
 * only: at night it is the IR LEDs' light, not the scene's) and the
 * white-balance gains, read from libisp_XM530's AE state and AWB.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "hal_internal.h"

#include "mpi_ae.h"
#include "mpi_awb.h"
#include "mpi_isp.h"

#include <string.h>

/* XM_MPI_ISP_Set/GetSaturationAttr live in libisp_XM530; the exposure
 * attributes in libxmhost. Both are in the global scope at run time. */

#define XM_SYS_GAIN_MAX (128u << 10) /* 128x, 10-bit fraction */

static int ae_comp_asked = -1;  /* -1: the sensor's own target */
static int flicker_hz = 0;      /* 0 off, 50 or 60: exposure in whole mains periods */
static int mirror_on, flip_on;  /* what the sensor was last set to (0, 0 at start) */
static double line_us = 0;      /* one sensor line, for exposure_time */
static rss_isp_mode_t isp_mode = RSS_ISP_DAY;
static int day_csc_sat = -1;    /* the CSC saturation night replaced */
static int saturation_asked = -1; /* -1: the sensor's table */

static int xm_exposure_write(int comp)
{
    ISP_EXPOSURE_ATTR_S a;

    memset(&a, 0, sizeof(a));
    if (XM_MPI_ISP_GetExposureAttr(0, &a) != XM_SUCCESS) {
        HAL_LOG_ERR("isp: XM_MPI_ISP_GetExposureAttr failed");
        return RSS_ERR_IO;
    }
    a.enOpType = OP_TYPE_AUTO;
    a.stAuto.stSysGainRange.u32Min = 1u << 10;
    a.stAuto.stSysGainRange.u32Max = XM_SYS_GAIN_MAX;
    a.stAuto.u8Compensation = (XM_U8)(comp < 0 ? 0 : comp);
    /* the open AE reads this every frame and keeps the exposure to whole
     * flicker periods (10 ms at 50 Hz, 8.3 at 60), at least one */
    a.stAuto.stAntiflicker.bEnable = flicker_hz ? XM_TRUE : XM_FALSE;
    a.stAuto.stAntiflicker.u8Frequency = (XM_U8)flicker_hz;
    if (XM_MPI_ISP_SetExposureAttr(0, &a) != XM_SUCCESS) {
        HAL_LOG_ERR("isp: XM_MPI_ISP_SetExposureAttr failed");
        return RSS_ERR_IO;
    }
    return RSS_OK;
}

int xm_isp_exposure_defaults(void)
{
    int ret = xm_exposure_write(ae_comp_asked);

    if (ret == RSS_OK)
        HAL_LOG_INFO("isp: exposure automatic, gain to 128x, target %s", "the sensor's own");
    return ret;
}

int hal_isp_set_ae_comp(void *ctx, int val)
{
    int ret;

    if (!ctx)
        return RSS_ERR_INVAL;
    if (val == RSS_ISP_AUTO)
        val = -1;
    else if (val < 1 || val > 255) {
        HAL_LOG_WARN("ae_comp: %d is outside 1..255 (or auto for the sensor's own)", val);
        return RSS_ERR_INVAL;
    }
    ret = xm_exposure_write(val);
    if (ret)
        return ret;
    ae_comp_asked = val;
    if (val < 0)
        HAL_LOG_INFO("ae_comp: the sensor's own target");
    else
        HAL_LOG_INFO("ae_comp: target %d", val);
    return RSS_OK;
}

int hal_isp_get_ae_comp(void *ctx, int *val)
{
    ISP_EXPOSURE_ATTR_S a;

    if (!ctx || !val)
        return RSS_ERR_INVAL;
    memset(&a, 0, sizeof(a));
    if (XM_MPI_ISP_GetExposureAttr(0, &a) != XM_SUCCESS)
        return RSS_ERR_IO;
    if (!a.stAuto.u8Compensation)
        return RSS_ERR_BUSY; /* the sensor's own, not a number we hold */
    *val = a.stAuto.u8Compensation;
    return RSS_OK;
}

int hal_isp_set_saturation(void *ctx, int val)
{
    ISP_SATURATION_ATTR_S a;

    if (!ctx)
        return RSS_ERR_INVAL;
    if (val != RSS_ISP_AUTO && (val < 0 || val > 255)) {
        HAL_LOG_WARN("saturation: %d is outside 0..255", val);
        return RSS_ERR_INVAL;
    }
    memset(&a, 0, sizeof(a));
    if (XM_MPI_ISP_GetSaturationAttr(&a) != XM_SUCCESS) {
        HAL_LOG_ERR("isp: XM_MPI_ISP_GetSaturationAttr failed");
        return RSS_ERR_IO;
    }
    if (val == RSS_ISP_AUTO) {
        a.enOpType = OP_TYPE_AUTO;
    } else {
        a.enOpType = OP_TYPE_MANUAL;
        a.stManual.u8Saturation = (XM_U8)val;
    }
    if (XM_MPI_ISP_SetSaturationAttr(&a) != XM_SUCCESS) {
        HAL_LOG_ERR("isp: XM_MPI_ISP_SetSaturationAttr failed");
        return RSS_ERR_IO;
    }
    saturation_asked = val == RSS_ISP_AUTO ? -1 : val;
    if (saturation_asked < 0)
        HAL_LOG_INFO("saturation: the sensor's table");
    else if (val <= 128)
        HAL_LOG_INFO("saturation: %d (at or below 128 the open ISP leaves colour as it is)", val);
    else
        HAL_LOG_INFO("saturation: %d (Cb/Cr gain %.2fx)", val, (256 + 6 * (val - 128)) / 256.0);
    return RSS_OK;
}

int hal_isp_get_saturation(void *ctx, int *val)
{
    ISP_SATURATION_ATTR_S a;

    if (!ctx || !val)
        return RSS_ERR_INVAL;
    if (saturation_asked >= 0) {
        *val = saturation_asked;
        return RSS_OK;
    }
    memset(&a, 0, sizeof(a));
    if (XM_MPI_ISP_GetSaturationAttr(&a) != XM_SUCCESS)
        return RSS_ERR_IO;
    *val = a.enOpType == OP_TYPE_MANUAL ? a.stManual.u8Saturation : a.stAuto.au8Sat[0];
    return RSS_OK;
}

/* The sensor's line time, from its timing (VTS lines a frame at fps) */
void xm_isp_set_timing(unsigned fps, unsigned vts)
{
    line_us = fps && vts ? 1e6 / ((double)fps * vts) : 0;
}

int hal_isp_get_exposure(void *ctx, rss_exposure_t *exposure)
{
    ISP_INNER_STATE_INFO_S in;
    ISP_WB_INFO_S wb;
    uint64_t prod;

    if (!ctx || !exposure)
        return RSS_ERR_INVAL;
    memset(exposure, 0, sizeof(*exposure));
    memset(&in, 0, sizeof(in));
    if (XM_MPI_ISP_QueryInnerStateInfo(0, &in) != XM_SUCCESS)
        return RSS_ERR_IO;
    if (!in.u32ExposureTime || !in.u32AllGain)
        return RSS_ERR_BUSY; /* before the AE's first frame */

    /* lines x total gain (1024 = 1x): 1 line at 1x is 1. On the SC5332 a
     * lit room reads in the hundreds, the dark with the IR LEDs off up to
     * 3296 lines x 124x, about 409000 */
    prod = (uint64_t)in.u32ExposureTime * in.u32AllGain / 1024;
    exposure->total_gain = prod > 0xffffffffu ? 0xffffffffu : (uint32_t)prod;
    exposure->exposure_time = (uint32_t)(in.u32ExposureTime * line_us + 0.5);
    exposure->ae_luma = in.u8AveLum;
    exposure->valid_mask = RSS_EXPOSURE_VALID_TOTAL_GAIN |
                           (line_us > 0 ? RSS_EXPOSURE_VALID_TIME : 0);
    if (isp_mode != RSS_ISP_NIGHT)
        exposure->valid_mask |= RSS_EXPOSURE_VALID_AE_LUMA;

    memset(&wb, 0, sizeof(wb));
    if (XM_MPI_ISP_QueryWBInfo(&wb) == XM_SUCCESS && wb.u16Ggain) {
        /* 256 = 1x, green held at 1x by the open AWB */
        exposure->wb_rgain = wb.u16Rgain;
        exposure->wb_ggain = wb.u16Ggain;
        exposure->wb_bgain = wb.u16Bgain;
        exposure->valid_mask |= RSS_EXPOSURE_VALID_WB_RGAIN | RSS_EXPOSURE_VALID_WB_GGAIN |
                                RSS_EXPOSURE_VALID_WB_BGAIN;
    }
    return RSS_OK;
}

int hal_isp_set_running_mode(void *ctx, rss_isp_mode_t mode)
{
    ISP_CSC_ATTR_S c;

    if (!ctx || (mode != RSS_ISP_DAY && mode != RSS_ISP_NIGHT))
        return RSS_ERR_INVAL;
    if (mode == isp_mode)
        return RSS_OK;
    memset(&c, 0, sizeof(c));
    if (XM_MPI_ISP_GetCSCAttr(&c) != XM_SUCCESS) {
        HAL_LOG_ERR("isp: XM_MPI_ISP_GetCSCAttr failed");
        return RSS_ERR_IO;
    }
    if (mode == RSS_ISP_NIGHT) {
        if (c.stManual.u8SatuVal)
            day_csc_sat = c.stManual.u8SatuVal;
        c.bEnable = XM_TRUE;
        c.stManual.u8SatuVal = 0;
    } else {
        c.stManual.u8SatuVal = (XM_U8)(day_csc_sat > 0 ? day_csc_sat : 50);
    }
    if (XM_MPI_ISP_SetCSCAttr(&c) != XM_SUCCESS) {
        HAL_LOG_ERR("isp: XM_MPI_ISP_SetCSCAttr failed");
        return RSS_ERR_IO;
    }
    isp_mode = mode;
    if (mode == RSS_ISP_NIGHT)
        HAL_LOG_INFO("isp: night, black and white");
    else
        HAL_LOG_INFO("isp: day, colour (CSC saturation back to %d)", c.stManual.u8SatuVal);
    return RSS_OK;
}

int hal_isp_set_antiflicker(void *ctx, rss_antiflicker_t mode)
{
    int old = flicker_hz, rc;

    if (!ctx)
        return RSS_ERR_INVAL;
    switch (mode) {
    case RSS_ANTIFLICKER_OFF:
        flicker_hz = 0;
        break;
    case RSS_ANTIFLICKER_50HZ:
        flicker_hz = 50;
        break;
    case RSS_ANTIFLICKER_60HZ:
        flicker_hz = 60;
        break;
    default:
        return RSS_ERR_INVAL;
    }
    rc = xm_exposure_write(ae_comp_asked);
    if (rc) {
        flicker_hz = old;
        return rc;
    }
    if (flicker_hz)
        HAL_LOG_INFO("isp: anti-flicker %d Hz (exposure in whole %.1f ms periods)", flicker_hz,
                     500.0 / flicker_hz);
    else
        HAL_LOG_INFO("isp: anti-flicker off");
    return RSS_OK;
}

static int xm_mirror_flip(int mirror, int flip)
{
    ISP_CHN_ATTR_S c;

    mirror = !!mirror;
    flip = !!flip;
    if (mirror == mirror_on && flip == flip_on)
        return RSS_OK;
    memset(&c, 0, sizeof(c));
    XM_MPI_ISP_GetChnAttr(&c);
    c.bMode = XM_FALSE; /* the sensor does it */
    c.bMirror = mirror ? XM_TRUE : XM_FALSE;
    c.bFlip = flip ? XM_TRUE : XM_FALSE;
    if (XM_MPI_ISP_SetChnAttr(&c) != XM_SUCCESS) {
        HAL_LOG_ERR("isp: XM_MPI_ISP_SetChnAttr (mirror %d, flip %d) failed", mirror, flip);
        return RSS_ERR_IO;
    }
    mirror_on = mirror;
    flip_on = flip;
    HAL_LOG_INFO("isp: mirror %s, flip %s", mirror ? "on" : "off", flip ? "on" : "off");
    return RSS_OK;
}

/* hflip is the sensor's mirror (left-right), vflip its flip (upside down) */
int hal_isp_set_hflip(void *ctx, int enable)
{
    return ctx ? xm_mirror_flip(enable, flip_on) : RSS_ERR_INVAL;
}

int hal_isp_set_vflip(void *ctx, int enable)
{
    return ctx ? xm_mirror_flip(mirror_on, enable) : RSS_ERR_INVAL;
}
