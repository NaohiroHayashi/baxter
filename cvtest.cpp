#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>
#include <stdio.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rsdk_xdisplay_image");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Publisher pub;
  pub = it.advertise("/robot/xdisplay", 1);
  cv::Mat img = cv::imread("/home/kaminuno/catkin_ws/devel/lib/cvtest/sample.jpg");//reading an image
  
  if(!img.data){
      printf("Cannot Open Imgfile!¥n");
      return(-1);
  }
      
  cv::namedWindow("img",CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
  cv::imshow("img",img);
  
  cv_bridge::CvImage  out_msg;
  out_msg.encoding = sensor_msgs::image_encodings::BGR8; 
  out_msg.image = img;
  pub.publish(out_msg.toImageMsg());
  
  ros::Rate loop_rate(10);
  ros::spinOnce(); //callbackyoudesu
  loop_rate.sleep(); //sleep
  cvWaitKey(100000);
  return 0;
}
