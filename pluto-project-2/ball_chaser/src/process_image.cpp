#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
  // TODO: Request a service and pass the velocities to it to drive the robot
  ROS_INFO_STREAM("Driving the robot with command_robot");

  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;

  // call the drive to target service and pass the requested velocities
  if (!client.call(srv))
    ROS_ERROR("Failed to call the service command_robot");

}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
  int white_pixel = 255;
  int column_width_sep = img.width / 3; // separate at width/3
  int side_enum = 0; // will call 1 left, 2 center, 3 right
  //TODO: loop through each pixel in the image and check if there's a bright one
  // bool white_pixel_found = false;
  for(int i = 0; i < img.height & img.step; i++) {
    if(img.data[i] == white_pixel) {
      // white_pixel_found = true;
      if(i % img.width < column_width_sep) { // left side of image
        drive_robot(0.0, 0.5); // drive robot left
      } else if (i % img.width > (img.width - column_width_sep)) { // 
        drive_robot(0.0, -0.5); // drive robot right
      } else {
        drive_robot(0.5, 0); // drive robot straight
      };
    } else {
      drive_robot(0, 0); // if no white image found, stop
    }
  }
  // Then, identify if this pixel falls in the left, mid, or right side of the image
  // depending on the ball position, call the drive_bot function and pass velocities to it
  // request a stop when there is no white ball seen by the camera
}
int main(int argc, char** argv)
{
  // Initialize the process_image node and create a handle to it
  ros::init(argc, argv, "process_image");
  ros::NodeHandle n;

  // Define a client service capable of requesting services from command_robot
  client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
  // subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
  ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

  // handle ROS communication events
  ros::spin();

  return 0;
};


