#include "ros/ros.h"
#include "mex_ros_api/servo_motor_pos.h"


/*
 * ROS processing functions
 *
 */
bool setAbsPosSrvMotor(mex_ros_api::servo_motor_pos::Request   &input,
					   mex_ros_api::servo_motor_pos::Response  &output);



  
int main(int argc, char **argv){
  ros::init(argc, argv, "servo_motor_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("set_abs_servo_motor", setAbsPosSrvMotor);

  ROS_INFO("Ready to process servor motor absolute positions.");
  ros::spin();

  return 0;
}


bool setAbsPosSrvMotor(mex_ros_api::servo_motor_pos::Request   &input, mex_ros_api::servo_motor_pos::Response  &output){

	output.outLeftSrv = (float)input.inpLeftSrv;
	output.outRightSrv = (float)input.inpRightSrv;

	return true;
}




