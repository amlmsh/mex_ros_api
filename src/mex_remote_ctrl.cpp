#include "ros/ros.h"
#include "mex_ros_api/servo_motor_pos.h"
#include <cstdlib>
#include <string>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "servo_motor_ctrl");
  if (argc != 4){
      ROS_INFO("usage: process <motorIdx int> <posUnit const char*>  <int posVal>");
      return 1;
  };

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<mex_ros_api::servo_motor_pos>("set_abs_servo_motor");
  mex_ros_api::servo_motor_pos srv;
  srv.request.inpMotorIdxSrv  = atoi(argv[1]);
  srv.request.inpPosUnit = std::string(argv[2]);
  srv.request.inpPosVal = atoi(argv[3]);


  if (client.call(srv)){
	  ROS_INFO("\nreturned pos value: %i \nunit: %s \nof device: %i\n",
			  (int) srv.response.outPosVal,
			  ((std::string) srv.response.outPosUnit).c_str(),
			  (int) srv.response.outMotorIdxSrv);
  }else{
	  ROS_ERROR("Failed to call service servo motor set position.");
	  return 1;
  }

  return 0;
}
