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

#define Byte 8

ros::Publisher pub;

class processing
{
public:
    processing(int argc_input, char **argv_input);
    ~processing(){};
    static void callback(const sensor_msgs::Image msg);
    static processing* theProcess;
  void main_run(const char* input_topic_name,
		const char* output_topic_name
		  );
    int argc;
    char **argv;
    ros::Subscriber sub;
};

processing::processing(int argc_input, char **argv_input){
    processing::argc = argc_input;
    processing::argv = argv_input;
    ros::init(processing::argc, processing::argv, "mask");
}

void processing::callback(const sensor_msgs::Image msg)
{
    cout <<"msg width="<<msg.width<<" height="<<msg.height<<" step="<<msg.step<<endl;  
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
    cv::Mat maskImg(cv_ptr->image);
    cout << "maskImg.channels=" << maskImg.channels() << endl;

    vector<cv::Mat>planes_rgba, planes_rgb;
    cv::split(maskImg, planes_rgba);
    planes_rgb.push_back(planes_rgba[3]);
			 planes_rgb.push_back(planes_rgba[3]);
			 planes_rgb.push_back(planes_rgba[3]);
    
			 //cv::Mat whiteImg(maskImg.size(), CV_8UC3);
			 //cv::Mat blackImg = whiteImg.clone();
			 //cv::Mat resultImg = blackImg.clone();
			 //blackImg=cv::Scalar(0,0,0);
			 //whiteImg=cv::Scalar(255,255,255);
			 //cout << "whiteImg.channels=" << whiteImg.channels() << endl;
			 cv::Mat wbImg(maskImg.size(), CV_8UC3);
			 merge(planes_rgb, wbImg);
			 
    //cvCopy(whiteImg.data, resultImg.data, maskImg.data); 
    //cvtColor(blackImg, resultImg, CV_BGRA2BGR);
    /*
    cout << "result width=" << resultImg.width
      	 << " height=" << resultImg.height
	 << " step=" <<resultImg.step
	 <<endl;
    */
    //pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", wbImg).toImageMsg());
    pub.publish(msg);
    //cerr << "publish" << endl;
    return;
}

void processing::main_run(const char *input_topic_name,
			  const char *output_topic_name
			  ){
    ros::NodeHandle n;
    // ros::NodeHandle n_;
    sub = n.subscribe(input_topic_name, 100, processing::callback);
    pub = n.advertise<sensor_msgs::Image>(output_topic_name, 100);
    ros::spin();
}


int main(int argc, char **argv)
{
  //    if(argc!=3) {
  //  cout << "usage: rosrun mask mask INPUT_TOPIC OUTPUT_TOPIC" << endl;
  //  return -1;
  // }
    cout << "topic1:[" << argv[1] << "] topic2:[" << argv[2] << "]" << endl;
    processing ps(argc, argv);
    ps.main_run(argv[1], argv[2]);
    return 0;
}
