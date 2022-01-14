#include <iostream>
#include <unistd.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h> // scan 
#include <geometry_msgs/Twist.h>   // velocity
#include <darknet_ros_msgs/ObjectCount.h>   // cunter
#include <darknet_ros_msgs/BoundingBoxes.h>   // cunter
#include <darknet_ros_msgs/BoundingBox.h>   // cunter
#include <std_msgs/Int8.h>   // velocity
using namespace std;

// クラスLaserSampleNodeの定義
class LaserSampleNode
{
private:
  ros::NodeHandle nh;

  ros::Subscriber sub_laser;
  ros::Publisher pub_vel;
  ros::Subscriber sub;
  ros::Subscriber sub_box;
  darknet_ros_msgs::ObjectCount msg_include_count;
  darknet_ros_msgs::BoundingBoxes boxes;
  sensor_msgs::LaserScan latest_scan;

  geometry_msgs::Twist cmd_msg;
  const int loop_rate = 10;
  bool is_recieved_scan = false;
  //bool is_detect = false;
  int xmin;
  int xmax;

public:
  LaserSampleNode()
  {
    sub_laser = nh.subscribe("/light_sensor/front/scan",10,&LaserSampleNode::laser_callback,this);
    pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    sub = nh.subscribe("darknet_ros/found_object", 10, &LaserSampleNode::detectorCallback, this);
    sub_box = nh.subscribe("darknet_ros/bounding_boxes", 10, &LaserSampleNode::boxCallback, this);
  }

  // 速度指令値を保持する変数に値を代入する関数
  geometry_msgs::Twist create_vel_msg(const double vel, const double omega)
  {
    // http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html
    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = vel;
    cmd_msg.angular.z = omega;
    return cmd_msg;
  }
  

  void boxCallback(const darknet_ros_msgs::BoundingBoxes& Boxesmsg)
  {
    //box = Boxesmsg.bounding_boxes;
    boxes = Boxesmsg;
  }
 
  void detectorCallback(const darknet_ros_msgs::ObjectCount& Objectmsg)
  {
    ROS_INFO("Object>>>%d", Objectmsg.count);
    msg_include_count = Objectmsg;
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

      // 回転判定してから一定回数は回転司令を与え続ける
      cnt++;
      if (cnt < 3)
      {
        pub_vel.publish(cmd_msg);
        continue;
      }

      // 値を取得しないままlatest_scan変数にアクセスするとエラーを起こすので判定
      if(!is_recieved_scan) continue;

      // 複数あるセンサの値のうち、中央にある値を取得する
      // http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html
      const double center_value = latest_scan.ranges[latest_scan.ranges.size() / 2];
      const double right_value = latest_scan.ranges[latest_scan.ranges.size() / 4];
      const double left_value = latest_scan.ranges[latest_scan.ranges.size() * 3 / 4];
      //const int xmin = box.bounding_boxes;
      const int detecting = msg_include_count.count;
      
      if (detecting){//stop
	//for(darknet_ros_msgs::BoundingBox& box : boxes):
	  xmin = boxes.bounding_boxes[0].xmin;
	  xmax = boxes.bounding_boxes[0].xmax;
	if(xmax-xmin > 20){ 
          cmd_msg = create_vel_msg(0.0, 0.0);        
          ROS_INFO("GOAL!!!!! XMAX:%d XMIN*%d", xmax, xmin);
	  break;
	//}else if ((xmax+xmin)/2 < 200){
          //cmd_msg = create_vel_msg(0.2, 0.2);        
          //ROS_INFO("APROACHING LEFT XMAX:%d XMIN:%d", xmax, xmin);
	//}else if ((xmax+xmin)/2 > 500){
          //cmd_msg = create_vel_msg(0.2, -0.2);        
          //ROS_INFO("APROACHING RIGHT XMAX:%d XMIN:%d", xmax, xmin);
	}else{
          cmd_msg = create_vel_msg(0.2, 0.0);        
          ROS_INFO("APROACHING XMAX:%d XMIN:%d", xmax, xmin);
	}	  
      }else{
 	if (center_value < 1.0){//前方衝突回避
          cmd_msg = create_vel_msg(0.1, 0.5);//左回転
          pub_vel.publish(cmd_msg);
          cnt=0;
          ROS_INFO("center laser value %5f : rotate", center_value);
        }else{
          if (right_value < 0.4){
	    cmd_msg = create_vel_msg(0.3, 0.3);
	    pub_vel.publish(cmd_msg);
	    ROS_INFO("center laser value %5f : turn left", left_value);
          }else if (0.6<right_value<0.8){
	    cmd_msg = create_vel_msg(0.3, -0.3);
	    pub_vel.publish(cmd_msg);
	    ROS_INFO("center laser value %5f : turn right", left_value);
          }else{
            cmd_msg = create_vel_msg(0.3, -0.6);
            pub_vel.publish(cmd_msg);
            ROS_INFO("center laser value %8f : move on", center_value);
	  }
        }
      }
      // 指定したループ周期になるように調整
      r.sleep();
    }
  }
};

int main(int argc, char **argv)
{   
  ros::init(argc, argv, "laser_sample_node");

  // クラスLaserSampleNodeの実体となる変数nodeの定義
  LaserSampleNode node = LaserSampleNode();

  // メインループを回す
  node.mainloop();

  return 0;
}
