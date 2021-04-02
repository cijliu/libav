/*
 * @Author: cijliu
 * @Date: 2020-10-03 01:15:05
 * @LastEditTime: 2020-11-13 09:26:10
 */
#ifndef HI3516EV200_H
#define HI3516EV200_H
#include "sample_comm.h"
#include <signal.h>
struct hisilicon_vi {
	SAMPLE_VI_CONFIG_S stViConfig;
	VPSS_GRP           VpssGrp;
	VPSS_CHN           VpssChn; 
	VIDEO_FRAME_INFO_S stFrame;
};

void HISILICON_VIO_Sensor(int type);
int HISILICON_VIO_Start(int width, int height);
int HISILICON_VIO_GetFrame(unsigned char *buf, int *len);
void HISILICON_VIO_Stop(void);
#endif