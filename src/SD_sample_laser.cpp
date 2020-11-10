#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h> // scan
#include <geometry_msgs/Twist.h>   // cmd_vel

class LaserSampleNode
{
private:
  ros::NodeHandle nh;

  ros::Subscriber sub_laser;
  ros::Publisher pub_vel;

  sensor_msgs::LaserScan latest_scan;

  const int loop_rate = 2;
  bool is_recieved_scan = false;

public:
  LaserSampleNode()
  {
    sub_laser = nh.subscribe("/light_sensor/front/scan", 10, &LaserSampleNode::laser_callback, this);
    pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  }

  geometry_msgs::Twist create_vel_msg(const double vel, const double omega)
  {
    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = vel;
    cmd_msg.angular.z = omega;

    return cmd_msg;
  }

  void laser_callback(const sensor_msgs::LaserScanConstPtr &laser_msg)
  {
    latest_scan = *laser_msg;
    is_recieved_scan = true;
  }

  void mainloop()
  {
    ros::Rate r(loop_rate);
    int cnt = 0;

    while (ros::ok())
    {
      ros::spinOnce();

      cnt++;
      if(cnt<4) continue;
      if(!is_recieved_scan) continue;

      const double center_value = latest_scan.ranges[latest_scan.ranges.size() / 2];
      ROS_INFO("cernter laser value %f", center_value);

      if (center_value < 0.5)
      {
        geometry_msgs::Twist cmd_msg = create_vel_msg(0.0, 0.5);
        pub_vel.publish(cmd_msg);

        cnt=0;

        ROS_INFO("stop");
      }
      else
      {
        geometry_msgs::Twist cmd_msg = create_vel_msg(0.3, 0.0);
        pub_vel.publish(cmd_msg);

        ROS_INFO("move on");
      }

      r.sleep();
    }
  }
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "laser_sample_node");

  LaserSampleNode node = LaserSampleNode();
  node.mainloop();

  // ros::spin();

  return 0;
}