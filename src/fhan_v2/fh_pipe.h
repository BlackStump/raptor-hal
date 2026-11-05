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

/* Encoder channels (divinus F2_VENC_CHN_NUM). */
#define FH_VENC_CHN_MAX 8

/* Codec bits, as FH_VENC_CreateChn's capability mask and the channel
 * attribute's first word (divinus f2_common.h). */
#define FH_CODEC_JPEG 0x01u
#define FH_CODEC_MJPEG 0x02u
#define FH_CODEC_H264 0x04u
#define FH_CODEC_H265 0x10u

/* Rate-control modes (divinus f2_common.h). */
#define FH_RC_H264VBR 3u
#define FH_RC_H264CBR 4u
#define FH_RC_H264FIXQP 5u
#define FH_RC_H265VBR 7u
#define FH_RC_H265CBR 8u

/* FH_VENC_CreateChn payload. */
typedef struct {
    unsigned int types;
    unsigned int width;
    unsigned int height;
    unsigned int b_frames;
} fh_venc_cap;

/* Rate control, the tail of the channel attribute and FH_VENC_SetRCAttr's
 * payload. Layouts per mode are in fh_encoder.c. */
typedef struct {
    unsigned int mode;
    unsigned int param[21];
} fh_venc_rc;

/* FH_VENC_SetChnAttr payload for H.264/H.265, 0xac bytes. */
typedef struct {
    unsigned int codec;
    unsigned int profile; /* H.264 66/77/100; H.265 1 */
    unsigned int gop;
    unsigned int width;
    unsigned int height;
    unsigned int reserved[16];
    fh_venc_rc rate;
} fh_venc_attr;

/* The same buffer for MJPEG (divinus f2_venc_mjpegattr, driver-confirmed). */
typedef struct {
    unsigned int codec;
    unsigned int width;
    unsigned int height;
    unsigned int rotate;
    unsigned int enc_speed;
    unsigned int reserved[16];
    unsigned int mode; /* 0 = fixed QP */
    unsigned int qp;
    unsigned short frame_num;
    unsigned short frame_time;
    unsigned int reserved2[40];
} fh_venc_mjpeg_attr;

/* FH_VENC_GetStream output, V200 (divinus f2_venc_strm). field04 is the
 * channel; MJPEG puts its picture at field08/field0c instead of a NALU
 * list. The other words are logged on each channel's first frame. */
typedef struct {
    unsigned int type;
    unsigned int length;
    unsigned int addr;
} fh_venc_nalu;

typedef struct {
    unsigned int type;
    unsigned int field04;
    unsigned int field08;
    unsigned int field0c;
    unsigned int field10;
    unsigned int field14;
    unsigned int field18;
    unsigned int nalu_cnt;
    fh_venc_nalu nalu[40];
    unsigned int tail[48];
} fh_venc_strm;

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

    int (*fnVpssSetVOMode)(unsigned int chn, unsigned int mode);

    /* SYS binding and VENC (libdsp) */
    int (*fnBind)(unsigned int vpss_chn, unsigned int venc_chn);
    int (*fnUnbind)(unsigned int venc_chn);
    int (*fnVencCreate)(unsigned int chn, fh_venc_cap *cap);
    int (*fnVencGetStream)(unsigned int type_mask, fh_venc_strm *strm);
    int (*fnVencRelease)(unsigned int chn);
    int (*fnVencRequestIdr)(unsigned int chn);
    int (*fnVencSetAttr)(unsigned int chn, void *attr);
    int (*fnVencSetRc)(unsigned int chn, fh_venc_rc *rc);
    int (*fnVencStart)(unsigned int chn);
    int (*fnVencStop)(unsigned int chn);

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
    int (*fnIspGetAeInfo)(void *info);   /* optional */
    int (*fnIspGetAeInfo2)(void *info);  /* optional, probed only */
    int (*fnIspGetAeStatus)(void *info); /* optional, probed only */
    int (*fnIspGetAeDefaultCfg)(void *cfg); /* diagnostic */
    int (*fnIspGetAlgCtrl)(void *cfg);      /* diagnostic */
    int (*fnIspGetAeRouteCfg)(void *cfg);   /* diagnostic */
    int (*fnIspSetAlgCtrl)(void *cfg);      /* night AE experiment */
    int (*fnIspSetAeDefaultCfg)(void *cfg); /* night AE experiment */
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
