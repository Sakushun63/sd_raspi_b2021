#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h> // scan 
#include <geometry_msgs/Twist.h>   // velocity

// クラスLaserSampleNodeの定義
class WallTrackerNode
{
private:
  ros::NodeHandle nh;

  ros::Subscriber sub_laser;
  ros::Publisher pub_vel;

  // laser scanの測定値を保存する変数の定義
  sensor_msgs::LaserScan latest_scan;

  // 速度指令値を保存する変数の定義
  geometry_msgs::Twist cmd_msg;

  // 2Hzで制御ループを回す
  const int loop_rate = 2;

  // メッセージの初回受信を確認する変数
  bool is_recieved_scan = false;

public:
  WallTrackerNode()
  {
    sub_laser = nh.subscribe("/light_sensor/front/scan", 10, &WallTrackerNode::laser_callback, this);
    pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
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

  // laserトピックを受けたときに実行される関数
  // 最新のスキャン情報を保存しておく
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
      cmd_msg = create_vel_msg(0.3, 0.0);
 
      // 回転判定してから一定回数は回転司令を与え続ける
      pub_vel.publish(cmd_msg);
      continue;
    }

     // 指定したループ周期になるように調整
      r.sleep();
  }
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "wall_tracker_node");

  // クラスLaserSampleNodeの実体となる変数nodeの定義
  WallTrackerNode node = WallTrackerNode();

  // メインループを回す
  node.mainloop();

  return 0;
}
