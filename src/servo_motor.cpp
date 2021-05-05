#include "ros/ros.h"
#include "mex_ros_api/servo_motor_pos.h"

#include "Pololu.hpp"
#include "SerialCom.hpp"
#include "ServoMotor.hpp"


Pololu      *conn;
ServoMotor  *ptrArm;
ServoMotor* *fieldOMotors;
int nmbMotors = 2;
int glbArmIdx = 0; // for demo only

/*
 * ROS processing functions
 *
 */
bool setPosSrvMotor(mex_ros_api::servo_motor_pos::Request   &input,
					   mex_ros_api::servo_motor_pos::Response  &output);


void shutDown();

  
int main(int argc, char **argv){
  ros::init(argc, argv, "servo_motor_server");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("set_abs_servo_motor", setPosSrvMotor);

  // init serial connection
  const char* portName = "/dev/ttyACM0";  // Linux
  conn = new Pololu(portName, 9600);
  try{
	  conn->openConnection();
	  conn->getErrors();
  }catch(ExceptionPololu *msg){
	  std::cout << "Error Pololu: " << msg->getMsg() << endl;
	  return 0;
  }catch(...){
	  std::cout << "Unknown error, while initialising Pololu serial connection.\n";
	  return 0;
  }

  // init servomotors
  try{
	  fieldOMotors = new ServoMotor*[nmbMotors];
	  for(int armIdx=0; armIdx < nmbMotors; armIdx++){
		  ptrArm = new ServoMotor(armIdx, 6000, 4000	, conn);
		  ptrArm->setMinMaxDegree(-90,90);
		  fieldOMotors[armIdx] = ptrArm;
	  }
  }catch(ExceptionPololu *msg){
	  std::cout << "Error Pololu: " << msg->getMsg() << endl;
	  shutDown();
	  return 0;
  }catch(...){
	  std::cout << "Unknown error, while initialising Pololu controlled servo motors.\n";
	  shutDown();
	  return 0;
  }

  ROS_INFO("Ready to process servo motor absolute positions.");
  ros::spin();

  shutDown();
  return 0;
}


bool setPosSrvMotor(mex_ros_api::servo_motor_pos::Request   &input, mex_ros_api::servo_motor_pos::Response  &output){

	glbArmIdx = (glbArmIdx + 1) % nmbMotors;
	std::string unitStr;

	output.outMotorIdxSrv = (int)input.inpMotorIdxSrv;
	output.outPosVal      = (int)input.inpPosVal;
	unitStr               = (std::string)input.inpPosUnit;
	output.outPosUnit     = unitStr;

	// check values
	if((int)input.inpMotorIdxSrv > (nmbMotors-1)){
		std::cout << "Motor index out of range, max. index value is " << (nmbMotors-1) << "\n";
		return false;
	}

	// check motor index
	if((int)input.inpMotorIdxSrv < 0){ // neg. value then sending motor active state
		if(conn->getMovingState()){
			output.outPosUnit = string("ACTIVE");
			output.outPosVal = 1;
		}else{
			output.outPosUnit = string("INACTIVE");
			output.outPosVal = 0;
		}
		return true; // finish, no position data to be set
	}



	// set position values values
	try{

		ptrArm = fieldOMotors[glbArmIdx];
		if(unitStr.compare(0,3,"ABS") == 0 ){
			ptrArm->setPositionInAbs((int)input.inpPosVal);
		}else if (unitStr.compare(0,3,"DEG") == 0 ){
			ptrArm->setPositionInDeg((int)input.inpPosVal);
		}else{
			std::cout << "Unknown unit data: " << unitStr << " but looked for 'ABS' or 'DEG'\n";
			return false;
		}
	}catch(ExceptionServoMotor *msg){
		std::cout << msg->getMsg() << std::endl;
		return false;
	}catch(IException *msg){
		std::cout << msg->getMsg() << std::endl;
		return false;
	}catch(...){
		std::cout << "Unknown error while set position of robot arm.\n";
		return false;
	}

	return true;
}


void shutDown(){ // free memory and close serial connection
	  for(int armIdx=0; armIdx < nmbMotors; armIdx++){
		  ptrArm = fieldOMotors[armIdx];
		  delete ptrArm;
	  }
	  conn->closeConnection();
	  delete conn;

	  return;
}

