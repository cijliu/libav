#ifndef HI3516EV200_VI_H
#define HI3516EV200_VI_H
#include "hi3516ev200.h"

HI_S32 _SAMPLE_COMM_SYS_Init(VB_CONFIG_S* pstVbConfig);
static struct hisilicon_vi HisiDev = {
	.VpssGrp = 0, 
	.VpssChn = VPSS_CHN1,	
};

static void yuv_8bit_dump(VIDEO_FRAME_S* pVBuf, unsigned char* dst,int *len)
{
    unsigned int w, h;
    char* pVBufVirt_Y;
    char* pVBufVirt_C;
    char* pMemContent;
	unsigned char *yuv = dst;
    unsigned char TmpBuff[1920*1080*3/2]; //If this value is too small and the image is big, this memory may not be enough
    HI_U64 phy_addr;
    PIXEL_FORMAT_E  enPixelFormat = pVBuf->enPixelFormat;
    HI_U32 u32UvHeight = 0;/*When the storage format is a planar format, this variable is used to keep the height of the UV component */
    HI_BOOL bUvInvert;
	HI_U32 u32Size = 0;
	HI_CHAR* pUserPageAddr[2] = {HI_NULL, HI_NULL};
    bUvInvert = (PIXEL_FORMAT_YUV_SEMIPLANAR_420 == enPixelFormat
        || PIXEL_FORMAT_YUV_SEMIPLANAR_422 == enPixelFormat) ? HI_TRUE : HI_FALSE;

    if (PIXEL_FORMAT_YVU_SEMIPLANAR_420 == enPixelFormat || PIXEL_FORMAT_YUV_SEMIPLANAR_420 == enPixelFormat)
    {
        u32Size = (pVBuf->u32Stride[0]) * (pVBuf->u32Height) * 3 / 2;
        //u32Size = 	(pVBuf->u32Width) * (pVBuf->u32Height) * 3 / 2;
        u32UvHeight = pVBuf->u32Height / 2;
		//printf("size:%d %d\n", pVBuf->u32Stride[0], pVBuf->u32Height);
    }
    else if (PIXEL_FORMAT_YVU_SEMIPLANAR_422 == enPixelFormat || PIXEL_FORMAT_YUV_SEMIPLANAR_422 == enPixelFormat)
    {
        u32Size = (pVBuf->u32Stride[0]) * (pVBuf->u32Height) * 2;
        u32UvHeight = pVBuf->u32Height;
    }
    else if (PIXEL_FORMAT_YUV_400 == enPixelFormat)
    {
        u32Size = (pVBuf->u32Stride[0]) * (pVBuf->u32Height);
        u32UvHeight = pVBuf->u32Height;
    }
    *len = u32Size;
	//printf("enPixelFormat %d u32Size :%d \nu32UvHeight :%d\n",enPixelFormat,u32Size ,u32UvHeight);
    phy_addr = pVBuf->u64PhyAddr[0];

    pUserPageAddr[0] = (HI_CHAR*) HI_MPI_SYS_Mmap(phy_addr, u32Size);

    if (HI_NULL == pUserPageAddr[0])
    {
        return;
    }

    pVBufVirt_Y = pUserPageAddr[0];
    pVBufVirt_C = pVBufVirt_Y + (pVBuf->u32Stride[0]) * (pVBuf->u32Height);

    /* save Y ----------------------------------------------------------------*/
    //fprintf(stderr, "saving......Y......");
    //fflush(stderr);

	#if 0
	FILE *f = fopen("y.yuv", "wb+");
	fwrite(pVBufVirt_Y, 1, pVBuf->u32Height*pVBuf->u32Width, f);
	fclose(f);
	#endif
    for (h = 0; h < pVBuf->u32Height; h++)
    {
        pMemContent = pVBufVirt_Y + h * pVBuf->u32Stride[0];
        memcpy(yuv, pMemContent, pVBuf->u32Width);
		yuv += pVBuf->u32Width;
    }

    if (PIXEL_FORMAT_YUV_400 != enPixelFormat)
    {
        //fflush(pfd);
        /* save U ----------------------------------------------------------------*/
        //fprintf(stderr, "U......");
        //fflush(stderr);

        for (h = 0; h < u32UvHeight; h++)
        {
            pMemContent = pVBufVirt_C + h * pVBuf->u32Stride[1];

            if(!bUvInvert) pMemContent += 1;

            for (w = 0; w < pVBuf->u32Width / 2; w++)
            {
                TmpBuff[w] = *pMemContent;
                pMemContent += 2;
            }

            memcpy(yuv, TmpBuff, pVBuf->u32Width / 2);
			yuv += pVBuf->u32Width / 2;
        }

        //fflush(pfd);

        /* save V ----------------------------------------------------------------*/
        //fprintf(stderr, "V......");
        //fflush(stderr);

        for (h = 0; h < u32UvHeight; h++)
        {
            pMemContent = pVBufVirt_C + h * pVBuf->u32Stride[1];

            if(bUvInvert) pMemContent += 1;

            for (w = 0; w < pVBuf->u32Width / 2; w++)
            {
                TmpBuff[w] = *pMemContent;
                pMemContent += 2;
            }

            memcpy(yuv,TmpBuff, pVBuf->u32Width / 2);
			yuv += pVBuf->u32Width / 2;
        }
    }

    //fflush(pfd);

    //fprintf(stderr, "done %d!\n", pVBuf->u32TimeRef);
    //fflush(stderr);

    HI_MPI_SYS_Munmap(pUserPageAddr[0], u32Size);
    pUserPageAddr[0] = HI_NULL;
}
#if 0
static void HISILICON_VIO_HandleSig(HI_S32 signo)
{
    signal(SIGINT, SIG_IGN);
    signal(SIGTERM, SIG_IGN);

    if (SIGINT == signo || SIGTERM == signo)
    {
		HISILICON_VIO_Stop();
		return;
		#if 0
		SAMPLE_COMM_VENC_StopGetStream();
        SAMPLE_COMM_All_ISP_Stop();
        SAMPLE_COMM_SYS_Exit();
        SAMPLE_PRT("\033[0;31mprogram termination abnormally!\033[0;39m\n");
		#endif
    }
    exit(-1);
}
#endif
HI_S32 _SAMPLE_COMM_SYS_Init(VB_CONFIG_S* pstVbConfig)
{
    HI_S32 s32Ret = HI_FAILURE;
    int u32BlkSize;
    VB_CONFIG_S stVbConf;
    // if(g_sys_init) {
    //     g_sys_init ++;
    //     return HI_SUCCESS;
    // }
    // g_sys_init ++;
    HI_MPI_SYS_Exit();
    HI_MPI_VB_Exit();

    if (NULL == pstVbConfig)
    {
        SAMPLE_PRT("input parameter is null, it is invaild!\n");
        return HI_FAILURE;
    }
    #if 1
    /*config vb*/
    hi_memset(&stVbConf, sizeof(VB_CONFIG_S), 0, sizeof(VB_CONFIG_S));
    stVbConf.u32MaxPoolCnt              = 2;

    u32BlkSize = COMMON_GetPicBufferSize(1920, 1080, PIXEL_FORMAT_YVU_SEMIPLANAR_420, 
        DATA_BITWIDTH_8, COMPRESS_MODE_NONE, DEFAULT_ALIGN);
    stVbConf.astCommPool[0].u64BlkSize  = u32BlkSize;
    stVbConf.astCommPool[0].u32BlkCnt   = 3;
	u32BlkSize = COMMON_GetPicBufferSize(1280, 720, PIXEL_FORMAT_YVU_SEMIPLANAR_420, 
        DATA_BITWIDTH_8, COMPRESS_MODE_NONE, DEFAULT_ALIGN);
    stVbConf.astCommPool[1].u64BlkSize  = u32BlkSize;
    stVbConf.astCommPool[1].u32BlkCnt   = 3;
    s32Ret = HI_MPI_VB_SetConfig(&stVbConf);
    #else
    s32Ret = HI_MPI_VB_SetConfig(pstVbConfig);
    #endif
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("HI_MPI_VB_SetConf failed!\n");
        return HI_FAILURE;
    }

    s32Ret = HI_MPI_VB_Init();

    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("HI_MPI_VB_Init failed!\n");
        return HI_FAILURE;
    }

    s32Ret = HI_MPI_SYS_Init();

    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("HI_MPI_SYS_Init failed!\n");
        HI_MPI_VB_Exit();
        return HI_FAILURE;
    }

    return HI_SUCCESS;
}
int HISILICON_VIO_Start(int width, int height)
{
    HI_S32             s32Ret = HI_SUCCESS;

    HI_S32             s32ViCnt       = 1;
    VI_DEV             ViDev          = 0;
    VI_PIPE            ViPipe         = 0;
    VI_CHN             ViChn          = 0;
    HI_S32             s32WorkSnsId   = 0;


    SIZE_S             stSize;
    VB_CONFIG_S        stVbConf;
    PIC_SIZE_E         enPicSize;
    HI_U32             u32BlkSize;



    WDR_MODE_E         enWDRMode      = WDR_MODE_NONE;
    DYNAMIC_RANGE_E    enDynamicRange = DYNAMIC_RANGE_SDR8;
    PIXEL_FORMAT_E     enPixFormat    = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    VIDEO_FORMAT_E     enVideoFormat  = VIDEO_FORMAT_LINEAR;
    COMPRESS_MODE_E    enCompressMode = COMPRESS_MODE_NONE;
    VI_VPSS_MODE_E     enMastPipeMode = VI_ONLINE_VPSS_OFFLINE;//VI_ONLINE_VPSS_ONLINE;

    VPSS_GRP           VpssGrp        = HisiDev.VpssGrp;
    VPSS_GRP_ATTR_S    stVpssGrpAttr;
    VPSS_CHN           VpssChn        = HisiDev.VpssChn;
    HI_BOOL            abChnEnable[VPSS_MAX_PHY_CHN_NUM] = {0};
    VPSS_CHN_ATTR_S    astVpssChnAttr[VPSS_MAX_PHY_CHN_NUM];
    VI_USERPIC_ATTR_S stUserPicAttr;
    /*config vi*/
    SAMPLE_COMM_VI_GetSensorInfo(&HisiDev.stViConfig);

    HisiDev.stViConfig.s32WorkingViNum                                   = s32ViCnt;
    HisiDev.stViConfig.as32WorkingViId[0]                                = 0;
    HisiDev.stViConfig.astViInfo[s32WorkSnsId].stSnsInfo.MipiDev         = ViDev;
    HisiDev.stViConfig.astViInfo[s32WorkSnsId].stSnsInfo.s32BusId        = 0;
    HisiDev.stViConfig.astViInfo[s32WorkSnsId].stDevInfo.ViDev           = ViDev;
    HisiDev.stViConfig.astViInfo[s32WorkSnsId].stDevInfo.enWDRMode       = enWDRMode;
    HisiDev.stViConfig.astViInfo[s32WorkSnsId].stPipeInfo.enMastPipeMode = enMastPipeMode;
    HisiDev.stViConfig.astViInfo[s32WorkSnsId].stPipeInfo.aPipe[0]       = ViPipe;
    HisiDev.stViConfig.astViInfo[s32WorkSnsId].stPipeInfo.aPipe[1]       = -1;
    HisiDev.stViConfig.astViInfo[s32WorkSnsId].stChnInfo.ViChn           = ViChn;
    HisiDev.stViConfig.astViInfo[s32WorkSnsId].stChnInfo.enPixFormat     = enPixFormat;
    HisiDev.stViConfig.astViInfo[s32WorkSnsId].stChnInfo.enDynamicRange  = enDynamicRange;
    HisiDev.stViConfig.astViInfo[s32WorkSnsId].stChnInfo.enVideoFormat   = enVideoFormat;
    HisiDev.stViConfig.astViInfo[s32WorkSnsId].stChnInfo.enCompressMode  = enCompressMode;

    /*get picture size*/
    s32Ret = SAMPLE_COMM_VI_GetSizeBySensor(
    									HisiDev.stViConfig.astViInfo[s32WorkSnsId].stSnsInfo.enSnsType, 
    									&enPicSize);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("get picture size by sensor failed!\n");
        return s32Ret;
    }

    s32Ret = SAMPLE_COMM_SYS_GetPicSize(enPicSize, &stSize);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("get picture size failed!\n");
        return s32Ret;
    }

    /*config vb*/
    hi_memset(&stVbConf, sizeof(VB_CONFIG_S), 0, sizeof(VB_CONFIG_S));
    stVbConf.u32MaxPoolCnt              = 2;
	//SAMPLE_PRT("stSize %d x %d\n", stSize.u32Width, stSize.u32Height);
    u32BlkSize = COMMON_GetPicBufferSize(stSize.u32Width, stSize.u32Height, SAMPLE_PIXEL_FORMAT, DATA_BITWIDTH_8, COMPRESS_MODE_NONE, DEFAULT_ALIGN);
	stVbConf.astCommPool[0].u64BlkSize	= u32BlkSize;
	stVbConf.astCommPool[0].u32BlkCnt	= 2;
    u32BlkSize = COMMON_GetPicBufferSize(stSize.u32Width, stSize.u32Height, PIXEL_FORMAT_YVU_SEMIPLANAR_420, DATA_BITWIDTH_8, COMPRESS_MODE_NONE, DEFAULT_ALIGN);
    stVbConf.astCommPool[1].u64BlkSize  = u32BlkSize;
    stVbConf.astCommPool[1].u32BlkCnt   = 2;
    s32Ret = SAMPLE_COMM_SYS_Init(&stVbConf);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("system init failed with %d!\n", s32Ret);
        return s32Ret;
    }

    /*start vi*/
    s32Ret = SAMPLE_COMM_VI_StartVi(&HisiDev.stViConfig);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("start vi failed.s32Ret:0x%x !\n", s32Ret);
        goto EXIT;
    }
	///////////////////////////////
    
    stUserPicAttr.enUsrPicMode = VI_USERPIC_MODE_BGC;
    stUserPicAttr.unUsrPic.stUsrPicBg.u32BgColor = 0xff0000;
    if (HI_MPI_VI_SetUserPic(ViPipe, &stUserPicAttr))
    {
        SAMPLE_PRT("HI_MPI_VI_SetUserPic failed.\n");
        //return -1;
    }
    if (HI_MPI_VI_EnableUserPic(ViPipe))
    {
        SAMPLE_PRT("HI_MPI_VI_EnableUserPic failed.\n");
        //return -1;
    }
    
    ////////////////////////////////
    /*config vpss*/
    hi_memset(&stVpssGrpAttr, sizeof(VPSS_GRP_ATTR_S), 0, sizeof(VPSS_GRP_ATTR_S));
    stVpssGrpAttr.stFrameRate.s32SrcFrameRate    = -1;
    stVpssGrpAttr.stFrameRate.s32DstFrameRate    = -1;
    stVpssGrpAttr.enDynamicRange                 = DYNAMIC_RANGE_SDR8;
    stVpssGrpAttr.enPixelFormat                  = enPixFormat;
    stVpssGrpAttr.u32MaxW                        = stSize.u32Width;
    stVpssGrpAttr.u32MaxH                        = stSize.u32Height;
    stVpssGrpAttr.bNrEn                          = HI_TRUE;
    stVpssGrpAttr.stNrAttr.enCompressMode        = COMPRESS_MODE_FRAME;
    stVpssGrpAttr.stNrAttr.enNrMotionMode        = NR_MOTION_MODE_NORMAL;

    astVpssChnAttr[VpssChn].u32Width                    = width;
    astVpssChnAttr[VpssChn].u32Height                   = height;
    astVpssChnAttr[VpssChn].enChnMode                   = VPSS_CHN_MODE_USER;
    astVpssChnAttr[VpssChn].enCompressMode              = enCompressMode;
    astVpssChnAttr[VpssChn].enDynamicRange              = enDynamicRange;
    astVpssChnAttr[VpssChn].enVideoFormat               = enVideoFormat; 
    astVpssChnAttr[VpssChn].enPixelFormat               = enPixFormat;
    astVpssChnAttr[VpssChn].stFrameRate.s32SrcFrameRate = -1;
    astVpssChnAttr[VpssChn].stFrameRate.s32DstFrameRate = -1;
    astVpssChnAttr[VpssChn].u32Depth                    = 2;
    astVpssChnAttr[VpssChn].bMirror                     = HI_FALSE;
    astVpssChnAttr[VpssChn].bFlip                       = HI_FALSE;
    astVpssChnAttr[VpssChn].stAspectRatio.enMode        = ASPECT_RATIO_NONE;

    /*start vpss*/
    abChnEnable[HisiDev.VpssChn] = HI_TRUE;
    s32Ret = SAMPLE_COMM_VPSS_Start(HisiDev.VpssGrp, abChnEnable, &stVpssGrpAttr, astVpssChnAttr);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("start vpss group failed. s32Ret: 0x%x !\n", s32Ret);
        goto EXIT1;
    }

	/*vi bind vpss*/
    s32Ret = SAMPLE_COMM_VI_Bind_VPSS(ViPipe, ViChn, VpssGrp);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("vi bind vpss failed. s32Ret: 0x%x !\n", s32Ret);
		SAMPLE_COMM_VI_UnBind_VPSS(ViPipe, ViChn, VpssGrp);
        goto EXIT2;
    }


	//signal(SIGINT, HISILICON_VIO_HandleSig);
    //signal(SIGTERM, HISILICON_VIO_HandleSig);
	return 0;
EXIT2:
    SAMPLE_COMM_VPSS_Stop(VpssGrp, abChnEnable);
EXIT1:
    SAMPLE_COMM_VI_StopVi(&HisiDev.stViConfig);
EXIT:
    SAMPLE_COMM_SYS_Exit();
    return s32Ret;
}

int HISILICON_VIO_GetFrame(unsigned char *buf, int *len)
{
	for(;;){
		
		int s32Ret = HI_MPI_VPSS_GetChnFrame(HisiDev.VpssGrp,HisiDev.VpssChn,&HisiDev.stFrame,1000); 
		if (HI_SUCCESS != s32Ret)
		{		  
			printf("[%d]get vpss frame err:0x%x\n",__LINE__, s32Ret); 
			return -1;
		}
		yuv_8bit_dump(&HisiDev.stFrame.stVFrame, buf, len);
		s32Ret = HI_MPI_VPSS_ReleaseChnFrame(HisiDev.VpssGrp,HisiDev.VpssChn,&HisiDev.stFrame);
		if (HI_SUCCESS != s32Ret)
		{		  
			printf("get vpss frame err:0x%x\n", s32Ret); 
		}
		break;
	}
	return 0;

}
void HISILICON_VIO_Sensor(int type)
{
    SAMPLE_COMM_VI_SetSensorType(type);
    SAMPLE_COMM_ISP_SetSensorType(type);
}
void HISILICON_VIO_Stop(void)
{
	HI_BOOL abChnEnable[VPSS_MAX_PHY_CHN_NUM] = {0};
	abChnEnable[0] = HI_TRUE;
	SAMPLE_COMM_VI_UnBind_VPSS(0, 0, HisiDev.VpssGrp);
	SAMPLE_COMM_VPSS_Stop(0, abChnEnable);
	SAMPLE_COMM_VI_StopVi(&HisiDev.stViConfig);
    SAMPLE_COMM_All_ISP_Stop();
	SAMPLE_COMM_SYS_Exit();

}
#endif 