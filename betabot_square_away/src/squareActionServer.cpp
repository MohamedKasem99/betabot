#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <move_in_shape/moveShapeAction.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class moveShapeAction
{
public:
  moveShapeAction(std::string name, char** argv) : 
    as_(nh_, name, false),
    threshold(atof(argv[1])),
    speed(atof(argv[2])),
    action_name_(name){
      //register the goal and feeback callbacks
      as_.registerGoalCallback(boost::bind(&moveShapeAction::goalCB, this));
      as_.registerPreemptCallback(boost::bind(&moveShapeAction::preemptCB, this));

      //subscribe to the data topic of interest
      sub_ = nh_.subscribe("/robot_pose_ekf/odom_combined", 1, &moveShapeAction::analysisCB, this);
      pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
      as_.start();
    }

  ~moveShapeAction(void){
    stop_bot("Stopping on exit!");
    } 
    
  void goalCB(){
    // reset helper variables
    // accept the new goal
    stop_bot("Stopping because of receiving a goal");
    goal_.pose = as_.acceptNewGoal()->pose;
    ROS_INFO("accepted new goal with coordinates (%f, %f, %f)",goal_.pose.position.x,goal_.pose.position.y,goal_.pose.position.z); 
  }

  void preemptCB(){
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    stop_bot();
    as_.setPreempted();
  }
 
  
  void analysisCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr& odom){

    // make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;

    feedback_.pose = odom->pose.pose;

    //get yaw angle of the robot

    double current_yaw = from_quat_to_yaw(feedback_.pose.orientation);
    double goal_yaw = from_quat_to_yaw(goal_.pose.orientation);

    //compute the std_dev and mean of the data 
    as_.publishFeedback(feedback_);

    ForwardControlMsg.linear.x = speed;
    AngleControlMsg.angular.z = speed*2;
    bool AngleAmIThereYet = std::abs(goal_yaw - current_yaw) < threshold;

    bool ForwardAmIThereYet = std::abs(goal_.pose.position.x - feedback_.pose.position.x) < threshold & 
                              std::abs(goal_.pose.position.y - feedback_.pose.position.y) < threshold;


    if(!AngleAmIThereYet)
    {
      pub_.publish(AngleControlMsg);
      ROS_INFO("Heading at (%f)", current_yaw); 
    }

    if(!ForwardAmIThereYet & AngleAmIThereYet)
    {
      pub_.publish(ForwardControlMsg);
      ROS_INFO("I'm at (%f, %f, %f)",feedback_.pose.position.x, feedback_.pose.position.y, feedback_.pose.position.z); 
    }

    else if (ForwardAmIThereYet & AngleAmIThereYet)
      {
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        stop_bot("Stopping because action succeded");
        // set the action state to succeeded
        as_.setSucceeded(result_);
        return;
      }
  }


void stop_bot(std::string stopping_reason = "NO REASON GIVEN"){
  ROS_INFO(stopping_reason.c_str());

  ForwardControlMsg.linear.x = 0;
  ForwardControlMsg.angular.z = 0; 
  pub_.publish(ForwardControlMsg);
}


protected:
    
  ros::NodeHandle nh_;

  actionlib::SimpleActionServer<move_in_shape::moveShapeAction> as_;
  std::string action_name_;
  double threshold, speed;
  geometry_msgs::Twist ForwardControlMsg;
  geometry_msgs::Twist AngleControlMsg;
  move_in_shape::moveShapeGoal goal_;
  move_in_shape::moveShapeFeedback feedback_;
  move_in_shape::moveShapeResult result_;

  ros::Subscriber sub_;
  ros::Publisher pub_;

  double from_quat_to_yaw(geometry_msgs::Quaternion q){

      double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
      double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
      double yaw = std::atan2(siny_cosp, cosy_cosp);

      return yaw;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveShape");

  moveShapeAction square(ros::this_node::getName(),argv);
  ros::spin();

  return 0;
}
