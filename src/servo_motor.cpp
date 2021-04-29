#include "ros/ros.h"
#include "mex_ros_api/servo_motor_pos.h"

#include "Pololu.hpp"
#include "SerialCom.hpp"
#include "ServoMotor.hpp"


  ServoMotor *ptrArm;

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

  const char* portName = "/dev/ttyACM0";  // Linux
  Pololu conn(portName, 9600);
  conn.openConnection();
  conn.getErrors();

  ptrArm = new ServoMotor(0, 7500, 1500	, &conn);
  ptrArm->setMinMaxDegree(-45,45);


  ROS_INFO("Ready to process servor motor absolute positions.");
  ros::spin();

  return 0;
}


bool setAbsPosSrvMotor(mex_ros_api::servo_motor_pos::Request   &input, mex_ros_api::servo_motor_pos::Response  &output){


	output.outLeftSrv = (float)input.inpLeftSrv;
	output.outRightSrv = (float)input.inpRightSrv;

	try{
		ptrArm->setPositionInDeg((int)input.inpLeftSrv);
	}catch(ExceptionServoMotor *msg){
		std::cout << msg->getMsg() << endl;
	}

	return true;
}




