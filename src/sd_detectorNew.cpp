#include <iostream>
#include <unistd.h>
#include <ros/ros.h>
#include <darknet_ros_msgs/ObjectCount.h>   // cunter
#include <std_msgs/Int8.h>   // velocity
using namespace std;

class DetectorNode
{
private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher pub;

  int  detect_msg;
  darknet_ros_msgs::ObjectCount msg_include_count;
  const int loop_rate = 10;

public:
  DetectorNode()
  {
  sub = nh.subscribe("darknet_ros/found_object", 10, &DetectorNode::detectorCallback, this);
  pub = nh.advertise<std_msgs::Int8>("/detectOrNot", 10);
  }

  void detectorCallback(const darknet_ros_msgs::ObjectCount& Objectmsg)
  {
    msg_include_count = Objectmsg;
  }

  void mainloop()
  {
  ros::Rate r(10);
  while (ros::ok())
  {
	if (msg_include_count.count = 1){
	ROS_INFO("Object Detecting!!!");
	pub.publish(msg_include_count);
	}else{
	ROS_INFO("Object Not Detect");
	}

	ros::spinOnce();
	r.sleep();
  }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detector");
  ros::spin();
  DetectorNode detectnode = DetectorNode();  
  detectnode.mainloop();  
  return 0;
}
