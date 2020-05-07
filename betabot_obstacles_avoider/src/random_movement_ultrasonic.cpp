#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>
#include <vector>


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

    //replace this with other 3 subscribers that get the 3 rays from 3 different ultransonic sensor topics.
    scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, &RandomMover::msgCallback, this);

    forward_control_msg.linear.x = speed;
    forward_control_msg.angular.z = 0;
    angle_control_msg.linear.x = 0;

    direction = true; // clockwise rotation
  }
  void msgCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    //Here you should assign the received ultrasonic readings.
    sensor.r_0 = msg->ranges[1500]; // corresponds to the 0 degree (shifted 3PI/2 because of the current lidar orientation)
    sensor.r_45 = msg->ranges[1750]; // 45 degrees 
    sensor.r_315 = msg->ranges[1250]; // -45 degrees 

    // new_vector = msg->ranges[1250 : 1750];
    // if any(list(map(isSubThresh, new_vector)))
    //   do_action; 
  }

  void run()
  {
    while (ros::ok())
    {
      ROS_INFO("%f ", sensor.r_0);
      if (sensor.r_0 >= threshold && sensor.r_45 >= threshold && sensor.r_315 >= threshold)
      {
        pub_.publish(forward_control_msg);
        ROS_INFO("MOVING FORWARD");
      }
      else
      {
        // Ternary operator checks the -45 degrees ray and decides on the direction of rotation accordingly
        angle_control_msg.angular.z = sensor.r_315 >= threshold ? speed * -2 : speed * 2;
        pub_.publish(angle_control_msg);
        ROS_INFO("TURNING");
      }

      ros::spinOnce();
    }
  }
  void printMsg(geometry_msgs::Twist msg)
  {
    ROS_INFO("linear %f, %f, %f. angular %f, %f, %f", msg.linear.x, msg.linear.y, msg.linear.z,
             msg.angular.x, msg.angular.y, msg.angular.z);
  }

  void stop_bot()
  {
    ROS_INFO("STOPPING!");
    forward_control_msg.linear.x = 0;
    forward_control_msg.angular.z = 0;
    pub_.publish(forward_control_msg);
  }

protected:
  ros::NodeHandle nh;
  ros::Publisher pub_;
  ros::Subscriber scan_sub;
  double threshold;
  double speed;
  float sensor_reading;
  struct
  {
    float r_0;         //0 degrees
    float r_45;        //45 degrees
    float r_315;       //-45 degrees 

  } sensor;            //holds the sensor rays
  geometry_msgs::Twist forward_control_msg;
  geometry_msgs::Twist angle_control_msg;
  bool direction;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "random_movement_ultrasonic");
  RandomMover my_random_mover;
  my_random_mover.run();
  return 0;
}