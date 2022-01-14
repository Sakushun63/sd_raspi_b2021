#include <iostream>
#include <unistd.h>
#include <ros/ros.h>
#include <darknet_ros_msgs/ObjectCount.h>   // cunter
#include <std_msgs/Int8.h>   // velocity
using namespace std;

darknet_ros_msgs::ObjectCount msg_include_count;
//int msg_include_count;
const int loop_rate = 10;

void detectorCallback(const darknet_ros_msgs::ObjectCount& Objectmsg)
{
ROS_INFO("Object Detecting!!!%d", Objectmsg.count);
msg_include_count = Objectmsg;
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "detector");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("darknet_ros/found_object", 10, detectorCallback);
  //ros::Publisher pub = nh.advertise<std_msgs::Int8>("/detectOrNot", 10);
  ros::Publisher pub = nh.advertise<darknet_ros_msgs::ObjectCount>("/detectOrNot", 10);

  ros::Rate r(loop_rate);
  while (ros::ok())
  {
	pub.publish(msg_include_count);
	ros::spinOnce();
	r.sleep();
  }
  
  return 0;
}
