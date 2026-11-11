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
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "hal_internal.h"

#include "mpi_ae.h"
#include "mpi_isp.h"

#include <string.h>

/* XM_MPI_ISP_Set/GetSaturationAttr live in libisp_XM530; the exposure
 * attributes in libxmhost. Both are in the global scope at run time. */

#define XM_SYS_GAIN_MAX (128u << 10) /* 128x, 10-bit fraction */

static int ae_comp_asked = -1;  /* -1: the sensor's own target */
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
