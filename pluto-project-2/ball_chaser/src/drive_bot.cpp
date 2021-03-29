#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"
#include <std_msgs/Float64.h>

// DONE : Include the ball_chaser "DriveToTarget" header file

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// done? TODO Create a handle_drive_request callback function that executes whenever a drive_robot service is requested

bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,ball_chaser::DriveToTarget::Response& res)
{
  ROS_INFO("DriveToTarget Request recieved - lx:%1.2f, az:%1.2f", (float)req.linear_x, (float)req.angular_z); 
  
  // This function should publish the requested linear x and angular velocities to the robot wheel
  std_msgs::Float64 linear_x, angular_z;
  // motor_command_publisher.publish(req.linear_x);
  // motor_command_publisher.publish(req.angular_z);

  // Create a motor_command object of type geometry_msgs::Twist
  geometry_msgs::Twist motor_command;
  // Set wheel velocities to requested velocities
  motor_command.linear.x = req.linear_x;
  motor_command.angular.z = req.angular_z;
  // publishes angles to drive the robot
  motor_command_publisher.publish(motor_command);
// After publishing the requested velocities, a messagefeedback should be returned with the requested wheel velocities 
  res.msg_feedback = "commanded velocities - lx: " + std::to_string(motor_command.linear.x) + " az: " + std::to_string(motor_command.angular.z);
  ROS_INFO_STREAM(res.msg_feedback);

  return true;

}
int main(int argc, char** argv)
{
  // Initialize a ROS node 
  ros::init(argc, argv, "drive_bot");

  // Create a ROS NodeHandle object
  ros::NodeHandle n;

  // Inform ROS master that we will be publishing a message of type geomettry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
  motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // DONE: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function //  ball_chaser_pub = n.advertise<std_msgs::Float56>("/ball_chaser/command_robot", 10);
  ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
  ROS_INFO("Ready to send drive commands");
  // DONE: Delete the loop, move the code to the inside of the callback function and make the necessary changes to publish the requested velocities instead of constant values 

  // TODO or done?: Handle ROS communication events
  ros::spin();

  return 0;

} 
