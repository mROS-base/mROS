#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp> //あった
#include <opencv2/highgui/highgui.hpp> //イラン
#include <iostream>
#include <vector>
#include <ctime>
#include <cmath>

#include <stdio.h>
#include <time.h>
#include <sys/time.h>


static const std::string OPENCV_WINDOW = "Image window";
using namespace std;


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  int width;
  int height;
  
public:
  ImageConverter(int _width, int _height)
    : it_(nh_), width(_width), height(_height)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/image_raw", 1, 
      &ImageConverter::imageCb, this);
   
    image_pub_ = it_.advertise("/input_data", 1000);

    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    double d[7];
    int t = 0;

    d[t++] = get_dtime();

    //sleep(1);
    sleep(0.1);


    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGRA8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    d[t++] = get_dtime();

    //HSV
    //cv::Mat hsvimg, redimg;
    //cvtColor(cv_ptr->image, hsvimg, CV_BGR2HSV);

    d[t++] = get_dtime();

    //inRange(hsvimg, cv::Scalar(150, 70, 70), cv::Scalar(180, 255, 255), redimg);  //閾値を設定しrにマスクとして格納
    //default:150,70,70 ~ 180,255,255

    d[t++] = get_dtime();
    //膨張収縮
    //cv::Mat dilateimg, erodeimg;
    //cv::dilate(redimg, dilateimg, cv::Mat(), cv::Point(-1,-1), 10);
    //cv::erode(dilateimg, erodeimg, cv::Mat(), cv::Point(-1,-1), 10);

    cv::Mat resizedImg;
    //cv::resize(cv_ptr->image, resizedImg, cv::Size(32,32));
    //cv::resize(cv_ptr->image, resizedImg, cv::Size(640,480));
    cv::resize(cv_ptr->image, resizedImg, cv::Size(width,height));
	
    image_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgra8", resizedImg).toImageMsg());
    }

	double get_dtime(){
	  struct timeval tv;
	  gettimeofday(&tv, NULL);
	  return ((double)(tv.tv_sec) + (double)(tv.tv_usec) * 0.001 * 0.001);
	}
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");

  if(argc!=3) {
    cout << "usage: rosrun preprocess preprocess_node WIDTH HEIGHT" << endl;
    return -1;
  }
  int ic_width  = atoi(argv[1]);
  int ic_height = atoi(argv[2]);
  cout << "WIDTH =" << ic_width << endl;
  cout << "HEIGHT=" << ic_height << endl;
  
  ImageConverter ic(ic_width, ic_height);
  ros::spin();
  return 0;
}


/*
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub2;
sensor_msgs::Image img2;
cv::Mat resizedImg;
unsigned char img2_buf[160*120*4];

void Callback(sensor_msgs::Image& msg){
	cv::Mat mat(msg.height,msg.width,CV_8UC4,msg.data,msg.step);

	cv::resize(mat,resizedImg,cv::Size(160,120));
	//memcpy(img2.data,resizedImg.data,img2.step*img2.height);
	for(int i=0;i<img2.step*img2.height;i++){
	  img2.data[i] = resizedImg.data[i];
	}
	pub2.publish(img2);
}

int main(){
#ifndef _USR_TASK_2_
#define _USR_TASK_2_
	ROS_INFO("========Activate user task2========");

	int argc = 0;
	char **argv = NULL;
	ros::init(argc,argv,"mros_image_converter");
	ROS_INFO("ADVERTISE PUB2");
	pub2 = nh.advertise<sensor_msgs::Image>("input_data",1000);
	img2.width = 160;
	img2.height = 120;
	img2.encoding="bgra8";
	img2.is_bigendian = 0;
	img2.step = 160*4;
	std_msgs::Header head;
	head.seq = 0;
	head.frame_id = "mROScam2";
	img2.header = head;
	sub = nh.subscribe("image_raw",1,Callback);
	ros::spin();
	return 0;
#endif

}
*/
