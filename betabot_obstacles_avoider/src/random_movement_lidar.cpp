#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <array>
#include <algorithm>
#include <stdlib.h>     //for using the function sleep

/*

By: Mohamed Kasem
s-mohamed.kasem@zewailcity.edu.eg
https://github.com/MohamedKasem99

==================================================

This node will work with 3 range rays, namely float r_0, float r_45, float r_315 which 
hold the values of 3 mounted ultrasonic sensors. The rays will indicate whether the robot 
should be moving forward, turning left or turning right based on a certain threshold (class member)
*/
class RandomMover
{

public:
  RandomMover()
  {
    threshold = 0.25;
    speed = 0.3;
    pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, &RandomMover::msgCallback, this);

    forward_control_msg.linear.x = speed;
    forward_control_msg.angular.z = 0;
    angle_control_msg.linear.x = 0;

  }
  void msgCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    sum_l = 0; 
    sum_r = 0;
    for (int i = 0; i < 250; i++)
    {
      if (msg->ranges[i] <= threshold + msg->range_min)
      {  sum_r++;
      }
      if (msg->ranges[2000 - i] <= threshold + msg->range_min)
        sum_l++;
    }
 
        // ROS_INFO("left= %f, middle=%f, right=%f", msg->ranges[1500], msg->ranges[0], msg->ranges[500]);
  }

  void run()
  {
    while (ros::ok())
    {
      if (sum_l < 10 && sum_r < 10)
      {
        pub_.publish(forward_control_msg);
        ROS_INFO("MOVING FORWARD");
        ROS_INFO("sum_l= %i, sum_r=%i", sum_l, sum_r);
      }
      else
      {
        // Ternary operator checks the -45 degrees ray and decides on the direction of rotation accordingly
        
        angle_control_msg.angular.z = sum_r > 10? speed * -2 : speed * 2;
        pub_.publish(angle_control_msg);
        sleep(1);
        ROS_INFO("TURNING");
        ROS_INFO("sum_l= %i, sum_r=%i", sum_l, sum_r);
      }

      ros::spinOnce();
    }
  }
  void printMsg(geometry_msgs::Twist msg)
  {
    ROS_INFO("linear %f, %f, %f. angular %f, %f, %f", msg.linear.x, msg.linear.y, msg.linear.z,
             msg.angular.x, msg.angular.y, msg.angular.z);
  }
  ~RandomMover()
  {
    ROS_INFO("STOPPING!");
    forward_control_msg.linear.x = 0;
    forward_control_msg.angular.z = 0;
    pub_.publish(forward_control_msg);
  }
  int direction()
  {
    if(sum_l > 10 || sum_r > 10){
      if(sum_l > sum_r) return 1;
      if(sum_r > sum_l) return 0;
    } 
    return 2;
  }

protected:
  ros::NodeHandle nh;
  ros::Publisher pub_;
  ros::Subscriber scan_sub;
  double threshold, speed;
  int sum_r, sum_l;
  geometry_msgs::Twist forward_control_msg, angle_control_msg;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "random_movement_lidar");
  RandomMover my_random_mover;
  my_random_mover.run();
  return 0;
}