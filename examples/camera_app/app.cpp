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
#include "opencv2/opencv.hpp"


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
#endif
JPEG_Converter Jcu;


/** mROS structure variables **/
ros::NodeHandle n;
ros::Publisher pub;
sensor_msgs::Image img;
ros::Rate loop_rate(FREQ);

static void IntCallbackFunc_Vfield(DisplayBase::int_type_t int_type) {
    //Interrupt callback function
	//mROSCallback　function pub
	memcpy(img.data,FrameBuffer_Video,sizeof(FrameBuffer_Video));

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
unsigned char ibuf[320*4*240];
void usr_task1(){
#ifndef _USR_TASK_1_
#define _USR_TASK_1_

	syslog(LOG_NOTICE,"========Activate user task1========");
#endif

	//mROS configuration
	int argc = 0;
	char *argv = NULL;
	int count = 0;
	ros::init(argc,argv,"mros_camera");
	pub = n.advertise("image_raw",1);
	img.encoding="bgra8";
	img.is_bigendian = 0;
	img.width = 320;
	img.height = 240;
	img.step = img.width*4;
	Header head;
	head.seq = 0;
	head.sec = 0;
	head.nsec = 0;
	head.frame_id = "mROScam";
	img.header = head;
	img.data = &ibuf[0];
	//camera start and publish loop

	camera_start();
	dly_tsk(1000);	//pubノードの起動が間に合わない？
	ROS_INFO("USER TASK1: start data publish");
	while(1){
		ROS_INFO("USER TASK1: publishing image %d", count++);
		pub.publish(img);
		loop_rate.sleep();
		dly_tsk(200);
	}
}



/*==============================================================================*/
/*preprocess node*/
//width = 160,height = 120 に変換して画像投げる

ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub2;
sensor_msgs::Image img2;
cv::Mat resizedImg;
unsigned char img2_buf[160*120*4];

void Callback(sensor_msgs::Image& msg){
	cv::Mat mat(msg.height,msg.width,CV_8UC4,msg.data,msg.step);
	cv::resize(mat,resizedImg,cv::Size(160,120));
	memcpy(img2.data,resizedImg.data,img2.step*img2.height);
	pub2.publish(img2);
}


void usr_task2(){
#ifndef _USR_TASK_2_
#define _USR_TASK_2_
	syslog(LOG_NOTICE,"========Activate user task2========");

	int argc = 0;
	char *argv = NULL;
	ros::init(argc,argv,"mros_image_converter");
	ROS_INFO("ADVERTISE PUB2");
	pub2 = nh.advertise("input_data",1000);
	img2.width = 160;
	img2.height = 120;
	img2.encoding="bgra8";
	img2.is_bigendian = 0;
	img2.step = 160*4;
	img2.data = &img2_buf[0];
	Header head;
	head.seq = 0;
	head.sec = 0;
	head.nsec = 0;
	head.frame_id = "mROScam2";
	img2.header = head;
	sub = nh.subscriber("image_raw",1,Callback);
	ros::spin();

#endif

}
