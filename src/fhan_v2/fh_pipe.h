/*
 * fhan_v2/fh_pipe.h -- the Fullhan video pipeline: sensor, ISP and VPSS.
 *
 * Function signatures transcribed from OpenIPC divinus src/hal/fhan/f2_*.h
 * (MIT) as carried on the PB4 branch (pr61). The library exports were listed
 * on the PB4 image (2026-11-05) and every name used here is among them. The
 * struct layouts the ISP and the sensor library exchange (the sensor ops
 * table, the VI attribute) are deliberately NOT declared: this backend only
 * passes them through as opaque pointers, so a wrong transcription cannot
 * hurt it.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef FH_PIPE_H
#define FH_PIPE_H

#include "fh_common.h"

#include <pthread.h>

/* VPSS output channels on the V200 (divinus F2_VPSS_CHN_NUM). */
#define FH_VPSS_CHN_MAX 3

/* FH_VPSS_SetViAttr / SetChnAttr / GetChnAttr payload: two words. */
typedef struct {
    unsigned int width;
    unsigned int height;
} fh_dim;

/* FH_VPSS_SetFramectrl payload (divinus f2_vpss_framectrl). */
typedef struct {
    unsigned short src_fps;
    unsigned short dst_fps;
} fh_vpss_framectrl;

/*
 * One sensor mode: the ISP's sensor-format word for a size and rate. The
 * GC4653 rows are the PB4 measurements from divinus pr61 (2560x1440@25 is
 * 0x603 and is what the stock firmware and divinus both stream). Sensors with
 * fps 0 have only divinus's single default format and are unmeasured here.
 */
typedef struct {
    const char *lib;     /* sensor library stem, "gc4653_mipi" */
    unsigned int width;
    unsigned int height;
    unsigned int fps;    /* 0 = not known */
    unsigned int format; /* API_ISP_SetSensorFmt */
} fh_snr_mode;

typedef struct {
    /* libraries */
    void *h_ispcore, *h_isp, *h_mipi, *h_adv, *h_advisp, *h_snr;

    /* VPSS (libdsp) */
    int (*fnVpssSetViAttr)(fh_dim *dim);
    int (*fnVpssEnable)(unsigned int grp);
    int (*fnVpssDisable)(unsigned int grp);
    int (*fnVpssGetChnCap)(unsigned int chn, void *cap);
    int (*fnVpssChnInitMem)(unsigned int chn, unsigned int width, unsigned int height);
    int (*fnVpssSetChnAttr)(unsigned int chn, fh_dim *dim);
    int (*fnVpssSetFramectrl)(unsigned int chn, fh_vpss_framectrl *ctrl);
    int (*fnVpssOpenChn)(unsigned int chn);
    int (*fnVpssCloseChn)(unsigned int chn);

    /* ISP (libisp, libadvapi_isp) */
    int (*fnIspMemInit)(unsigned int width, unsigned int height);
    int (*fnIspSensorRegCb)(unsigned int index, void *ops);
    int (*fnIspSensorUnRegCb)(unsigned int index);
    int (*fnIspSensorInit)(void);
    int (*fnIspSetSensorFmt)(unsigned int format);
    int (*fnIspInit)(void);
    int (*fnIspSensorKick)(void);
    int (*fnIspRun)(void);
    int (*fnIspExit)(void);
    int (*fnIspLoadParam)(void *param);
    int (*fnIspSetMirrorAndflipEx)(unsigned int mirror, unsigned int flip, unsigned int bayer);
    int (*fnIspGetVIState)(void *state);
    int (*fnIspGetAeInfo)(void *info); /* optional */
    int (*fnAdvIspInit)(void);
    int (*fnAdvIspSensorInit)(void);

    /* sensor library */
    void *snr_ops; /* <lib>_sensor_if, opaque, handed to API_ISP_SensorRegCb */
    /* Returns the driver's object pointer, not a status (bench, 2026-11-05:
     * 0xb695a740 one run, 0xb699a740 the next). divinus ignores it. */
    void *(*fnSnrCreate)(void);
    int (*fnSnrDestroy)(void);
} fh_pipe_impl;

#endif /* FH_PIPE_H */
