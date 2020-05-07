#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <boost/thread.hpp>
#include <move_in_shape/moveShapeAction.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

void spinThread()
{
  ros::spin();
}

geometry_msgs::Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);

    geometry_msgs::Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;

    return q;
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_moveShape");

  // create the action client
  actionlib::SimpleActionClient<move_in_shape::moveShapeAction> ac("moveShape");
  boost::thread spin_thread(&spinThread);
  bool finished_before_timeout;


  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();

  ROS_INFO("Action server started, sending goal 1.");
  // send a goal to the action
  move_in_shape::moveShapeGoal goal;
  double square_side = atof(argv[1]);


  auto forward = [&](double x, double y)
  {
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  ac.sendGoal(goal);

  //wait for the action to return
  finished_before_timeout = ac.waitForResult(ros::Duration(10));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    {
      ROS_INFO("Action did not finish before the time out.");
      ac.cancelGoal();
    }
  }; 


  auto left = [&](int corner){
  goal.pose.orientation = ToQuaternion(1.5708 * corner,0,0);
  
  ROS_INFO("Action server started, sending goal 2.");
  ac.sendGoal(goal);

  //wait for the action to return
  finished_before_timeout = ac.waitForResult(ros::Duration(10));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    {
      ROS_INFO("Action did not finish before the time out.");
      ac.cancelGoal();
    }
  };

  
  forward(square_side, 0);
  left(1);
  forward(square_side, square_side*1.05);
  left(2);
  forward(0, square_side*0.95);
  left(3);  
  forward(0, 0);
  left(4);  


  // shutdown the node and join the thread back before exiting
  ros::shutdown();
  spin_thread.join();

  //exit
  return 0;
}