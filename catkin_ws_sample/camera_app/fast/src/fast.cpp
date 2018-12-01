#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/contrib/features2d.hpp>
#include <iostream>
#include <vector>
#include <ctime>
#include <cmath>

#include <stdio.h>
#include <time.h>
#include <sys/time.h>

using namespace std;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher image_raw_pub_;
  int mode;
  
public:
  ImageConverter(int _mode, const char* _output_topicname)
    : it_(nh_), mode(_mode)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/input_data", 1, 
      &ImageConverter::imageCb, this);
   
    image_pub_ = it_.advertise(_output_topicname, 1000);
    image_raw_pub_ = it_.advertise("/image",1000);

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

    // sleep(20);
    
    image_raw_pub_.publish(msg);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    d[t++] = get_dtime();

    cv::Mat originalImg(cv_ptr->image);

    //HSV
    //cv::Mat hsvimg, redimg;
    cv::Mat grayImg;
    //cvtColor(cv_ptr->image, hsvimg, CV_BGR2HSV);
    cvtColor(cv_ptr->image, grayImg, CV_BGR2GRAY);




    d[t++] = get_dtime();

    vector<cv::KeyPoint> kvs;
    cv::Mat maskImg(grayImg);
    cv::Mat resultImg(grayImg);
    cv::Mat outputImg;
    cv::FAST(grayImg, kvs, 20, true);

    d[t++] = get_dtime();

    static bool drawCircle = (mode==1);
    if(drawCircle) {
      cv::drawKeypoints(grayImg, kvs, resultImg);
    } else {
      maskImg=cv::Scalar(0,0,0);
      vector<cv::KeyPoint>::iterator it;
      for(it = kvs.begin(); it != kvs.end(); it++) {
	cv::Point2f& pf= it->pt;
	cv::Point p ( pf.x, pf.y );
	cv::line(maskImg, p, p, cv::Scalar(255,255,255));
	//maskImg.at(p)=cv::Scalar(255,255,255);
      }
      cv::dilate(maskImg, resultImg, cv::Mat(), cv::Point(-1,-1), 1);
    }


    d[t++] = get_dtime();

    //cout << std::setw(5);
    for(int j = 0; j < t-1; j++){
      //cout << "time(" << j << "):" << d[j+1] - d[j] << endl;
      cout << (int) ((d[j+1]-d[j])*1000000) << " ";
    }
    cout << endl;

    
    //cvtColor(resultImg, outputImg, cv::COLOR_GRAY2BGR);
    if(drawCircle) {
      image_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", resultImg).toImageMsg());
    } else {
      image_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", resultImg).toImageMsg());
    }
  }

	double get_dtime(){
	  struct timeval tv;
	  gettimeofday(&tv, NULL);
	  return ((double)(tv.tv_sec) + (double)(tv.tv_usec) * 0.001 * 0.001);
	}
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "fast");
    if(argc!=3) {
    cout << "usage: rosrun fast fast OUTPUT_TOPICNAME MODE" << endl;
    cout << "MODE 0=FASTX(hardware) 1=drawFeature(Circle, software)" << endl;
    return -1;
  }
  int ic_mode  = atoi(argv[2]);
  cout << "MODE=" << ic_mode << endl;
  
  ImageConverter ic(ic_mode, argv[1]);
  ros::spin();
  return 0;
}

