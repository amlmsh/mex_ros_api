#include "ros/ros.h"
#include "mex_ros_api/servo_motor_pos.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "servo_motor_ctrl");
  if (argc != 3)
    {
      ROS_INFO("usage: process two floats leftInp rightInp");
      return 1;
    }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<mex_ros_api::servo_motor_pos>("set_abs_servo_motor");
  mex_ros_api::servo_motor_pos srv;
  srv.request.inpLeftSrv  = atof(argv[1]);
  srv.request.inpRightSrv = atof(argv[2]);
  if (client.call(srv))
    {
      ROS_INFO("\nLeft out: %f \nRight out: %f\n",
	       (float)srv.response.outLeftSrv, (float) srv.response.outRightSrv);
    }
  else
    {
      ROS_ERROR("Failed to call service neural network processing.");
      return 1;
    }

  return 0;
}
