//  https://wiki.ros.org/image_transport/Tutorials/SubscribingToImages をほぼそのまま使用
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

void camera_callback(const sensor_msgs::ImageConstPtr& img_msg)
{
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(img_msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img_msg->encoding.c_str());
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_sample_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_color", 1, camera_callback);
  ros::spin();

    return 0;
}