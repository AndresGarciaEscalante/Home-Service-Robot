#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  
  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the first stop
  goal.target_pose.pose.position.x = -0.5;
  goal.target_pose.pose.position.y = 3.5;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the pick up 
  ROS_INFO("Robot is traveling to the pick up zone");
  ac.sendGoal(goal);
  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached the pick up location
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Robot picked up the virtual box");
    //Wait 5 seconds to start the next task
    sleep(5);
    // Define the position to drop off the object
    goal.target_pose.pose.position.x = 1.0;
    goal.target_pose.pose.position.y = -4.0;
    goal.target_pose.pose.orientation.w = 1.0;

    // Send the position and orientation for the drop off
    ROS_INFO("Robot is traveling to the drop off zone");
    ac.sendGoal(goal);
    
    // Wait an infinite time for the results
    ac.waitForResult();
    
    // Check if robot reached the drop off location
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Robot dropped the virtual box");
    }
    
    else{
      ROS_INFO("The robot was not able to complete task 2");  
    }
  }
  else
    ROS_INFO("The robot was not able to complete task 1");
  sleep(5);
  return 0;
}