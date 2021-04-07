/*
 * @Author: cijliu
 * @Date: 2020-09-30 19:34:27
 * @LastEditTime: 2020-11-13 09:31:46
 */
#include "config.h"
#include "libavformat/avformat.h"
#include "libavformat/internal.h"
#include <unistd.h>
#include <fcntl.h>
#include <stdatomic.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <poll.h>
#include "hi3516ev200_vi.h"
#include "hi3516ev200_venc.h"
#include "libavutil/avassert.h"
#include "libavutil/imgutils.h"
#include "libavutil/internal.h"
#include "libavutil/log.h"
#include "libavutil/opt.h"
#include "libavutil/parseutils.h"
#include "libavutil/pixdesc.h"
#include "libavutil/avstring.h"
#include "libavutil/mathematics.h"
#define _HI3516EV200_DEBUG_  1

#if _HI3516EV200_DEBUG_
#include "stdio.h"
#define _debug(format,...) printf("[\033[1m\033[40;33m"__FILE__": %05d\033[0m]: "format, __LINE__, ##__VA_ARGS__)  
#else
#define _debug(format,...)
#endif
enum pixel_enum {
    HI3516EV200_YUV420,
    HI3516EV200_H264,
    HI3516EV200_H265,
    HI3516EV200_JPEG,
};
struct video_data {
    AVClass *class;
    int fd;
    int frame_format; /* V4L2_PIX_FMT_* */
    int width, height;
    int frame_size;
    int timeout;
    int interlaced;
    int top_field_first;

    int buffers;
    atomic_int buffers_queued;
    void **buf_start;
    unsigned int *buf_len;
    char *standard;
    int channel;
    char *video_size;   /**< String describing video size,
                             set by a private option. */
    char *pixel_format; /**< Set by a private option. */
    int list_format;    /**< Set by a private option. */
    char *framerate;    /**< Set by a private option. */
};
static void video_format(struct video_data *s)
{
    if(strcmp(s->video_size, "1080p") == 0){
        s->width = 1920;
        s->height = 1080;
        s->frame_format = PIC_1080P;
    }
    else if(strcmp(s->video_size, "720p") == 0){
        s->width = 1280;
        s->height = 720;
        s->frame_format = PIC_720P;
    }
    else if(strcmp(s->video_size, "vga") == 0){
        s->width = 640;
        s->height = 480;
        s->frame_format = PIC_VGA;
    }
    else {
        s->width = 320;
        s->height = 240;
        s->frame_format = PIC_QVGA;
    }
}
static int pixel_format(struct video_data *s)
{
    enum pixel_enum en = HI3516EV200_YUV420;
    if(strcmp(s->pixel_format, "h264") == 0){
        en = HI3516EV200_H264;
    }
    else if(strcmp(s->pixel_format, "h265") == 0){
        en = HI3516EV200_H264;
        
    }
    else if(strcmp(s->pixel_format, "jpeg") == 0){
        en = HI3516EV200_JPEG;
    }
    return (int)en;
}
static int sensor_type(char *s)
{
    int type = 0;
    if(strcmp(s, "gc2053") == 0){
        type = 1; 
    }
    return type;
}
static int hi3516ev200_read_header(AVFormatContext *s1)
{
    AVStream *st;
    int ret;
    struct video_data *s = s1->priv_data;
    video_format(s);
    _debug("%s %d %s (%d,%d)\n",__func__, __LINE__,s1->filename,s->width, s->height);
    st = avformat_new_stream(s1, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    HISILICON_VIO_Sensor(sensor_type(s1->filename));
    HISILICON_VIO_Start(s->width, s->height);
    ret = HISILICON_Coder_Init(HISILICON_CODER_H264, s->frame_format, SAMPLE_RC_CBR, VENC_GOPMODE_NORMALP);
    if(ret){
        return ret;
    }
    return 0;
}
static int hi3516ev200_read_packet(AVFormatContext *s1, AVPacket *pkt)
{
    int ret,en;
    struct video_data *s = s1->priv_data;
    _debug("%s %d\n",__func__, __LINE__);
    
    if ((ret = av_new_packet(pkt, s->width*s->height*3/2)) < 0){
        _debug("%s %d\n",__func__, __LINE__);
        return ret;
    }
    ret = HISILICON_VIO_GetFrame(pkt->data, &pkt->size);    
    if(ret){
        return AVERROR(ret);
    }
    en = pixel_format(s);
    if(en == HI3516EV200_H264){
        pack_t hisi_pack;
        _debug("encoder:H264\n");
        if(HISILICON_Coder_SendYUV420Frame(pkt->data, s->frame_format)){
            _debug("encoder:H264 err\n");
        }
        hisi_pack.data = pkt->data;
        if(HISILICON_Coder_Run(&hisi_pack)){
            _debug("HISILICON_H264_Coder_Run break\n");
        }
        pkt->size = hisi_pack.len;
        //av_free_packet(pkt);
    }
    _debug("%s %d %d\n",__func__, __LINE__,pkt->size);
    return pkt->size;
}
static int hi3516ev200_read_close(AVFormatContext *s1)
{
    _debug("%s %d\n",__func__, __LINE__);
    HISILICON_VIO_Stop();
    return 0;
}
#define OFFSET(x) offsetof(struct video_data, x)
#define DEC AV_OPT_FLAG_DECODING_PARAM
static const AVOption options[] = {
    { "pixel_format",   "A string describing frame format, such as h264 or yuv420.", OFFSET(pixel_format),   AV_OPT_TYPE_STRING, {.str = "yuv420"},  0, 0,       DEC },
    { "video_size",   "A string describing frame size, such as vga or h720.", OFFSET(video_size),   AV_OPT_TYPE_STRING, {.str = "vga"},  0, 0,       DEC },
    { NULL },
};
static const AVClass hi3516ev200_class = {
    .class_name = "hi3516ev200 indev",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};
AVInputFormat ff_hi3516ev200_demuxer = {
    .name           = "hi3516ev200",
    .long_name      = NULL_IF_CONFIG_SMALL("hi3516ev200 device"),
    .priv_data_size = sizeof(struct video_data),
    .read_header    = hi3516ev200_read_header,
    .read_packet    = hi3516ev200_read_packet,
    .read_close     = hi3516ev200_read_close,
    .flags          = AVFMT_NOFILE,
    .priv_class     = &hi3516ev200_class,
};