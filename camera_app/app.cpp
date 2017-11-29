#include "app.h"
#include "../mros-lib/ros.h"

//RGB888形式でのイメージデータをROSにパブリッシュする
//ROS compressed image streamでJPEGデータを送ればきれいに処理できそう
//image_raw/theora ? めんどくさそう

//mbed library 
#include "mbed.h"
#include "DisplayBace.h"
#include "JPEG_Converter.h"
#include "SoftPWM.h"
#include "EthernetInterface.h"

#define VIDEO_CVBS             (0)                 /* Analog  Video Signal */
#define VIDEO_CMOS_CAMERA      (1)                 /* Digital Video Signal */
#define VIDEO_YCBCR422         (0)
#define VIDEO_RGB888           (1)
#define VIDEO_RGB565           (2)

/** Camera setting **/
#define VIDEO_INPUT_METHOD     (VIDEO_CMOS_CAMERA) /* Select  VIDEO_CVBS or VIDEO_CMOS_CAMERA                       */
#define VIDEO_INPUT_FORMAT     (VIDEO_RGB888)    /* Select  VIDEO_YCBCR422 or VIDEO_RGB888 or VIDEO_RGB565        */
#define USE_VIDEO_CH           (0)                 /* Select  0 or 1            If selecting VIDEO_CMOS_CAMERA, should be 0.)               */
#define VIDEO_PAL              (0)                 /* Select  0(NTSC) or 1(PAL) If selecting VIDEO_CVBS, this parameter is not referenced.) */
/*****************************/

#if USE_VIDEO_CH == (0)
#define VIDEO_INPUT_CH         (DisplayBase::VIDEO_INPUT_CHANNEL_0)
#define VIDEO_INT_TYPE         (DisplayBase::INT_TYPE_S0_VFIELD)
#else
#define VIDEO_INPUT_CH         (DisplayBase::VIDEO_INPUT_CHANNEL_1)
#define VIDEO_INT_TYPE         (DisplayBase::INT_TYPE_S1_VFIELD)
#endif

#if ( VIDEO_INPUT_FORMAT == VIDEO_YCBCR422 || VIDEO_INPUT_FORMAT == VIDEO_RGB565 )
#define DATA_SIZE_PER_PIC      (2u)
#else
#define DATA_SIZE_PER_PIC      (4u)
#endif

/*! Frame buffer stride: Frame buffer stride should be set to a multiple of 32 or 128
    in accordance with the frame buffer burst transfer mode. */
#define PIXEL_HW               (320u)  /* QVGA */
#define PIXEL_VW               (240u)  /* QVGA */

#define VIDEO_BUFFER_STRIDE    (((PIXEL_HW * DATA_SIZE_PER_PIC) + 31u) & ~31u)
#define VIDEO_BUFFER_HEIGHT    (PIXEL_VW)

#if defined(__ICCARM__)
#pragma data_alignment=16
static uint8_t FrameBuffer_Video[VIDEO_BUFFER_STRIDE * VIDEO_BUFFER_HEIGHT]@ ".mirrorram";  //16 bytes aligned!;
#pragma data_alignment=4
#else
static uint8_t FrameBuffer_Video[VIDEO_BUFFER_STRIDE * VIDEO_BUFFER_HEIGHT]__attribute((section("NC_BSS"),aligned(16)));  //16 bytes aligned!;
#endif
static volatile int32_t vsync_count = 0;
static volatile int32_t vfield_count = 1;

#if defined(__ICCARM__)
#pragma data_alignment=8
static uint8_t JpegBuffer[2][1024 * 50]@ ".mirrorram";  //8 bytes aligned!;
#pragma data_alignment=4
#else
static uint8_t JpegBuffer[2][1024 * 50]__attribute((section("NC_BSS"),aligned(8)));  //8 bytes aligned!;
#endif
static size_t jcu_encode_size[2];
static int image_change = 0;
JPEG_Converter Jcu;
static int jcu_buf_index_write = 0;
static int jcu_buf_index_write_done = 0;
static int jcu_buf_index_read = 0;
static int jcu_encoding = 0;

ros::NodeHandle n;
ros::Publisher pub;
ros_Image img;
bool pub_flag = false;

static void JcuEncodeCallBackFunc(JPEG_Converter::jpeg_conv_error_t err_code) {

    //syslog(LOG_NOTICE,"Jcucallback");
    jcu_buf_index_write_done = jcu_buf_index_write;
    image_change = 1;
    jcu_encoding = 0;
}

static void IntCallbackFunc_Vfield(DisplayBase::int_type_t int_type) {
    //Interrupt callback function
	//mROSCallback　function pub
	memcpy(img.data,FrameBuffer_Video,sizeof(FrameBuffer_Video));
	pub_flag = true;
	//ROS_INFO("size [%d][%d], [%d][%d][%d],100:[%d][%d]",strlen(img.data),strlen(FrameBuffer_Video),img.data[99],img.data[100],img.data[101],img.data[100],FrameBuffer_Video[100]);
	//pub.imgpublish(&img);

	/*もともとのサンプルコード
    syslog(LOG_NOTICE,"Jcu_buf_index :%d",jcu_buf_index_write);
    if (vfield_count != 0) {
        vfield_count = 0;
    } else {
        vfield_count = 1;

        JPEG_Converter::bitmap_buff_info_t bitmap_buff_info;
        JPEG_Converter::encode_options_t   encode_options;

        bitmap_buff_info.width          = PIXEL_HW;
        bitmap_buff_info.height         = PIXEL_VW;
        bitmap_buff_info.format         = JPEG_Converter::WR_RD_YCbCr422;
        bitmap_buff_info.buffer_address = (void *)FrameBuffer_Video;

        encode_options.encode_buff_size = sizeof(JpegBuffer[0]);
        encode_options.p_EncodeCallBackFunc = &JcuEncodeCallBackFunc;

        jcu_encoding = 1;
        if (jcu_buf_index_read == jcu_buf_index_write) {
            if (jcu_buf_index_write != 0) {
                jcu_buf_index_write = 0;
            } else {
                jcu_buf_index_write = 1;
            }
        }
        jcu_encode_size[jcu_buf_index_write] = 0;
        if (Jcu.encode(&bitmap_buff_info, JpegBuffer[jcu_buf_index_write], &jcu_encode_size[jcu_buf_index_write], &encode_options) != JPEG_Converter::JPEG_CONV_OK) {
            jcu_encode_size[jcu_buf_index_write] = 0;
            jcu_encoding = 0;
        }
    }
    */
}

static void IntCallbackFunc_Vsync(DisplayBase::int_type_t int_type) {
    //Interrupt callback function for Vsync interruption
    if (vsync_count > 0) {
        vsync_count--;
    }
}

#if 1 /* WaitVsync(const int32_t wait_count) */
static void WaitVsync(const int32_t wait_count) {
    //Wait for the specified number of times Vsync occurs
    vsync_count = wait_count;
    while (vsync_count > 0) {
        /* Do nothing */
    }
}
#endif /* WaitVsync */

#if 1 /* camera_start(void) */
DisplayBase::video_ext_in_config_t ext_in_config;
static void camera_start(void) {
    DisplayBase::graphics_error_t error;

#if VIDEO_INPUT_METHOD == VIDEO_CMOS_CAMERA
    PinName cmos_camera_pin[11] = {
        /* data pin */
        P2_7, P2_6, P2_5, P2_4, P2_3, P2_2, P2_1, P2_0,
        /* control pin */
        P10_0,      /* DV0_CLK   */
        P1_0,       /* DV0_Vsync */
        P1_1        /* DV0_Hsync */
    };
#endif 

    /* Create DisplayBase object */
    DisplayBase Display;

    /* Graphics initialization process */
    error = Display.Graphics_init(NULL);
    if (error != DisplayBase::GRAPHICS_OK) {
        printf("Line %d, error %d\n", __LINE__, error);
        while (1);
    }

#if VIDEO_INPUT_METHOD == VIDEO_CVBS
    error = Display.Graphics_Video_init( DisplayBase::INPUT_SEL_VDEC, NULL);
    if( error != DisplayBase::GRAPHICS_OK ) {
        printf("Line %d, error %d\n", __LINE__, error);
        while(1);
    }

#elif VIDEO_INPUT_METHOD == VIDEO_CMOS_CAMERA
    /* MT9V111 camera input config */
    ext_in_config.inp_format     = DisplayBase::VIDEO_EXTIN_FORMAT_BT601; /* BT601 8bit YCbCr format */
    ext_in_config.inp_pxd_edge   = DisplayBase::EDGE_RISING;              /* Clock edge select for capturing data          */
    ext_in_config.inp_vs_edge    = DisplayBase::EDGE_RISING;              /* Clock edge select for capturing Vsync signals */
    ext_in_config.inp_hs_edge    = DisplayBase::EDGE_RISING;              /* Clock edge select for capturing Hsync signals */
    ext_in_config.inp_endian_on  = DisplayBase::OFF;                      /* External input bit endian change on/off       */
    ext_in_config.inp_swap_on    = DisplayBase::OFF;                      /* External input B/R signal swap on/off         */
    ext_in_config.inp_vs_inv     = DisplayBase::SIG_POL_NOT_INVERTED;     /* External input DV_VSYNC inversion control     */
    ext_in_config.inp_hs_inv     = DisplayBase::SIG_POL_INVERTED;         /* External input DV_HSYNC inversion control     */
    ext_in_config.inp_f525_625   = DisplayBase::EXTIN_LINE_525;           /* Number of lines for BT.656 external input */
    ext_in_config.inp_h_pos      = DisplayBase::EXTIN_H_POS_CRYCBY;       /* Y/Cb/Y/Cr data string start timing to Hsync reference */
    ext_in_config.cap_vs_pos     = 6;                                     /* Capture start position from Vsync */
    ext_in_config.cap_hs_pos     = 150;                                   /* Capture start position form Hsync */
    ext_in_config.cap_width      = 640;                                   /* Capture width  */
    ext_in_config.cap_height     = 468u;                                  /* Capture height Max 468[line]
                                                                             Due to CMOS(MT9V111) output signal timing and VDC5 specification */
    error = Display.Graphics_Video_init( DisplayBase::INPUT_SEL_EXT, &ext_in_config);
    if( error != DisplayBase::GRAPHICS_OK ) {
        printf("Line %d, error %d\n", __LINE__, error);
        while(1);
    }

    /* MT9V111 camera input port setting */
    error = Display.Graphics_Dvinput_Port_Init(cmos_camera_pin, 11);
    if( error != DisplayBase::GRAPHICS_OK ) {
        printf("Line %d, error %d\n", __LINE__, error);
        while (1);
    }
#endif

    /* Interrupt callback function setting (Vsync signal input to scaler 0) */
    error = Display.Graphics_Irq_Handler_Set(DisplayBase::INT_TYPE_S0_VI_VSYNC, 0, IntCallbackFunc_Vsync);
    if (error != DisplayBase::GRAPHICS_OK) {
        printf("Line %d, error %d\n", __LINE__, error);
        while (1);
    }
    /* Video capture setting (progressive form fixed) */
    error = Display.Video_Write_Setting(
                VIDEO_INPUT_CH,
#if VIDEO_PAL == 0
                DisplayBase::COL_SYS_NTSC_358,
#else
                DisplayBase::COL_SYS_PAL_443,
#endif
                FrameBuffer_Video,
                VIDEO_BUFFER_STRIDE,
#if VIDEO_INPUT_FORMAT == VIDEO_YCBCR422
                DisplayBase::VIDEO_FORMAT_YCBCR422,
                DisplayBase::WR_RD_WRSWA_NON,
#elif VIDEO_INPUT_FORMAT == VIDEO_RGB565
                DisplayBase::VIDEO_FORMAT_RGB565,
                DisplayBase::WR_RD_WRSWA_32_16BIT,
#else
                DisplayBase::VIDEO_FORMAT_RGB888,
                DisplayBase::WR_RD_WRSWA_32BIT,
#endif
                PIXEL_VW,
                PIXEL_HW
            );
    if (error != DisplayBase::GRAPHICS_OK) {
        printf("Line %d, error %d\n", __LINE__, error);
        while (1);
    }

    /* Interrupt callback function setting (Field end signal for recording function in scaler 0) */
    error = Display.Graphics_Irq_Handler_Set(VIDEO_INT_TYPE, 0, IntCallbackFunc_Vfield);
    if (error != DisplayBase::GRAPHICS_OK) {
        printf("Line %d, error %d\n", __LINE__, error);
        while (1);
    }

    /* Video write process start */
    syslog(LOG_NOTICE,"Video start");
    error = Display.Video_Start (VIDEO_INPUT_CH);
    if (error != DisplayBase::GRAPHICS_OK) {
        printf("Line %d, error %d\n", __LINE__, error);
        while (1);
    }

    /* Video write process stop */
    syslog(LOG_NOTICE,"Video stop");
    error = Display.Video_Stop (VIDEO_INPUT_CH);
    if (error != DisplayBase::GRAPHICS_OK) {
        printf("Line %d, error %d\n", __LINE__, error);
        while (1);
    }
    /* Video write process start */
    syslog(LOG_NOTICE,"Video start");
    error = Display.Video_Start (VIDEO_INPUT_CH);
    if (error != DisplayBase::GRAPHICS_OK) {
        printf("Line %d, error %d\n", __LINE__, error);
        while (1);
    }

    /* Wait vsync to update resister */
    WaitVsync(1);
}
#endif /* camera_start(void) */

void usr_task1(){
#ifndef _USR_TASK_1_
#define _USR_TASK_1_

	syslog(LOG_NOTICE,"========Activate user task1========");
#endif

	//mROS configuration
	pub = n.advertise("image_raw",1);
	img.encoding="rgba8";
	img.is_bigendian = 0;
	img.width = 320;
	img.height = 240;
	img.step = 320*4;
	Header head;
	head.seq = 0;
	head.sec = 0;
	head.nsec = 0;
	head.frame_id = "mROScam";
	img.header = head;

	//camera start and publish loop
	camera_start();
	while(1){
		if(pub_flag){
			//ROS_INFO("[%x][%x][%x]",img.data[0],img.data[1],img.data[2]);
			pub.imgpublish(&img);
			pub_flag = false;
		}
	}
}

void Callback(){

}

void usr_task2(){
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscriber("image_raw",1,Callback);
	ros::spin();

}

