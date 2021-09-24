#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
// #include<unistd.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/*void delay(float seconds) {
    ROS_INFO("Waiting %f seconds", seconds);
    unsigned int microsecond = 1000000;
    usleep(seconds * microsecond); //sleeps for 3 second
}*/

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //Tell the action client to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // Set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Move to the pickup goal
  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 2.48455810546875;
  goal.target_pose.pose.position.y = -6.9661078453063965;
  goal.target_pose.pose.position.z = 0.0;
  
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = -0.5930581109560585;
  goal.target_pose.pose.orientation.w = 0.8051596593404511;
  

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("[PICKUP ZONE REACHED]");
  else
    ROS_INFO("The robot failed to reach the pickup goal for some reason");

  ros::Duration(5).sleep(); // Wait 5 seconds

  // // Move to the drop off zone
  // // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = -2.1428637504577637;
  goal.target_pose.pose.position.y = -2.9643027782440186;
  goal.target_pose.pose.position.z = 0.0;
  
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = -0.9198558849924803;
  goal.target_pose.pose.orientation.w = 0.3922564860454202;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("[DROP OFF ZONE REACHED]");
  else
    ROS_INFO("The robot failed to reach the drop off zone for some reason");

  getchar();
  return 0;
}