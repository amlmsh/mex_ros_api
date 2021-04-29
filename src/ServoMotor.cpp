//============================================================================
// Name        : ServoMotor.cpp
// Author      : Willi Penner
//
// Description : ServoMotor source file. It contains the definition of the
//               functions of the ServoMotor class.
//============================================================================
#include "ServoMotor.hpp"
#include "Pololu.hpp"
#include <iostream>
#include <sstream>
#include <cmath>









ServoMotorPololuBase::ServoMotorPololuBase(unsigned short servoID,
										   unsigned short neutralPos,
										   unsigned short delta,
										   IPololu  *pololuController){

	if(servoID < 0){
		string msg("ServoMotorPololuBase::servo number is negative.");
		throw new ExceptionServoMotorBase(msg);
	}
	servoNmb_ = servoID;

	if(neutralPos < 0){
		string msg("ServoMotorPololuBase:: neutral position is negative.");
		throw new ExceptionServoMotorBase(msg);
	}
	neutralPosition_ = neutralPos;

	if(neutralPosition_ <= delta){
		string msg("ServoMotorPololuBase:: delta range is larger than neutral position.");
		throw new ExceptionServoMotorBase(msg);
	}
	delta_ = delta;

	if(pololuController == NULL){
		string msg("ServoMotorPololuBase:: controller reference is NULL pointer.");
		throw new ExceptionServoMotorBase(msg);
	}
	pololuCtrl_ = pololuController;

	try{
		pololuCtrl_->getPosition(servoNmb_);
	}catch(IException *e){
		stringstream ss;
		ss << "ServoMotorPololuBase::constructor: no acces to servo motor ";
		ss << "having ID " << servoNmb_ << ". Check servo motor ID parameter:";
		ss << e->getMsg();
		throw new ExceptionServoMotorBase(ss.str() );
	}catch(string errMsg){
		stringstream ss;
		ss << "ServoMotorPololuBase::constructor: no acces to servo motor ";
		ss << "having ID " << servoNmb_ << ". Check servo motor ID parameter:";
		ss << errMsg;
		throw new ExceptionServoMotorBase(ss.str());
	}catch(...){
		stringstream ss;
		ss << "ServoMotorPololuBase::constructor: no acces to servo motor ";
		ss << "having ID " << servoNmb_ << ". Check servo motor ID parameter:";
		ss << "(unknown error).";
		throw new ExceptionServoMotorBase(ss.str());
	}
	return;
}



ServoMotorPololuBase::~ServoMotorPololuBase(){
	pololuCtrl_ = NULL;
	return;
};

unsigned short ServoMotorPololuBase::getServoNumber(){return servoNmb_;};

unsigned short ServoMotorPololuBase::getMinPosInAbs(){return (neutralPosition_ - delta_);};

unsigned short ServoMotorPololuBase::getMidPosInAbs(){return  neutralPosition_;};

unsigned short ServoMotorPololuBase::getMaxPosInAbs(){return (neutralPosition_ + delta_);};

unsigned short ServoMotorPololuBase::setPositionInAbs(unsigned short newPosition){
	if((newPosition < this->getMinPosInAbs()) ||
			(newPosition > this->getMaxPosInAbs())){
		string msg("setPositionInDeg:: position value is out of range.");
		throw new ExceptionServoMotorBase(msg);
	}


	try{
		pololuCtrl_->setPosition(servoNmb_,newPosition);
	}catch(IException *e){
		string msg("setPositionInAbs:: error while trying to set a new position:");
		msg += e->getMsg();
		throw new ExceptionServoMotorBase(msg);
	}catch(...){
		string msg("setPositionInAbs:: error while trying to set a new position.");
		throw new ExceptionServoMotorBase(msg);
	}

	try{
		return (pololuCtrl_->getPosition(servoNmb_));
	}catch(IException *e){
		string msg("setPositionInAbs:: error while trying to set a new position:");
		msg += e->getMsg();
		throw new ExceptionServoMotorBase(msg);
	}catch(...){
		string msg("setPositionInAbs:: error while trying to set a new position.");
		throw new ExceptionServoMotorBase(msg);
	}

};

unsigned short ServoMotorPololuBase::getPositionInAbs(){
	try{
		return (pololuCtrl_->getPosition(servoNmb_));
	}catch(IException *e){
		string msg("setPositionInAbs:: error while trying to set a new position:");
		msg += e->getMsg();
		throw new ExceptionServoMotorBase(msg);
	}catch(...){
		string msg("setPositionInAbs:: error while trying to set a new position.");
		throw new ExceptionServoMotorBase(msg);
	}
};



/**
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

/**
 *
 * \brief Set the maximal speed of the servo motor movement.
 *
 * \param newSpeed unsigned short. Value 1 represents the lowest speed.
 *               Value 255 and 0 represent the maximal speed.
 *
 *
 *
 */
unsigned short ServoMotorPololuBaseAdv::setSpeed(unsigned short newSpeed){
	if(newSpeed > 255){
		newSpeed = 255;
	}
	try{
		pololuCtrl_->setSpeed(servoNmb_, newSpeed);
		return newSpeed;
	}catch(IException *e){
		string msg("setPositionInAbs:: error while trying to set a new speed value:");
		msg += e->getMsg();
		throw new ExceptionServoMotorBaseAdv(msg);
	}catch(...){
		string msg("setPositionInAbs:: unknown error while trying to set a new speed value.");
		throw new ExceptionServoMotorBaseAdv(msg);
	}
}

/**
 *
 * \brief Set the maximal acceleration of the servo motor movement.
 *
 * \param newSpeed unsigned short. Value 1 represents the lowest speed.
 *               Value 255 and 0 represent the maximal speed.
 *
 *
 *
 */
unsigned short ServoMotorPololuBaseAdv::setAcceleration(unsigned short newAcceleration){
	if(newAcceleration > 255){
		newAcceleration = 255;
	}
	try{
		pololuCtrl_->setSpeed(servoNmb_, newAcceleration);
		return newAcceleration;
	}catch(IException *e){
		string msg("setPositionInAbs:: error while trying to set a new acceleration value:");
		msg += e->getMsg();
		throw new ExceptionServoMotorBaseAdv(msg);
	}catch(...){
		string msg("setPositionInAbs:: unknown error while trying to set a new acceleration value.");
		throw new ExceptionServoMotorBaseAdv(msg);
	}
}


/**
 *
 *
 *
 *
 *
 *
 *
 *
 */

void  ServoMotor::setMinMaxDegree(short minDegree, short maxDegree){
	if(maxDegree <= minDegree){
		string msg("setMinMaxDegree:: min degree is larger or equal than max degree.");
		throw new ExceptionServoMotor(msg);
	}
	minDeg_ = minDegree;
	maxDeg_ = maxDegree;
	return;
}

void  ServoMotor::setMinMaxRadian(float minRadian, float maxRadian){
	if(maxRadian <= minRadian){
		string msg("setMinMaxRadian:: min value radian is larger or equal to max value radian.");
		throw new ExceptionServoMotor(msg);
	}

	try{
		this->setMinMaxDegree(this->rad2deg(minRadian),this->rad2deg(maxRadian));
	}catch(IException *e){
		string msg("setMinMaxRadian:: error while setting max and min values in radian");
		msg += e->getMsg();
		throw new ExceptionServoMotor(msg);
	}catch(...){
		string msg("setMinMaxRadian:: unknown error while setting max and min values in radian");
		throw new ExceptionServoMotor(msg);
	}

	return;
}

short ServoMotor::setPositionInDeg(short newPosition){
	short actPos = newPosition;
	if((actPos < minDeg_) || (actPos > maxDeg_)){
		string msg("setPositionInDeg:: degree /radiant value is out of range.");
		throw new ExceptionServoMotor(msg);
	}

	try{
		pololuCtrl_->setPosition(servoNmb_,this->mapDegValue2PosValue(actPos));
	}catch(IException *e){
		string msg("setPositionInDeg:: error while trying to set and move to new position.");
		msg += e->getMsg();
		throw new ExceptionServoMotor(msg);
	}catch(...){
		string msg("setPositionInDeg:: unknown error while trying to set and move to new position.");
		throw new ExceptionServoMotor(msg);
	}

	return actPos;
};

float ServoMotor::setPositionInRad(float newPosition){
	short deg;
	try{
		deg = this->setPositionInDeg(this->rad2deg(newPosition));
	}catch(IException *e){
		string msg("setPositionInRad:: error while trying to set and move to new position.");
		msg += e->getMsg();
		throw new ExceptionServoMotor(msg);
	}catch(...){
		string msg("setPositionInRad:: unknown error while trying to set and move to new position.");
		throw new ExceptionServoMotor(msg);
	};
	return (this->deg2rad(deg));
};

short ServoMotor::getPositionInDeg(){
	short deg;
	unsigned short pos;
	try{
		pos = this->ServoMotorPololuBase::getPositionInAbs();
		deg = this->mapPosValue2DegValue(pos);
	}catch(IException *e){
		string msg("getPositionInDeg:: error while trying to read servo motor position.");
		msg += e->getMsg();
		throw new ExceptionServoMotor(msg);
	}catch(...){
		string msg("getPositionInDeg:: unknown error while trying to read servo motor position.");
		throw new ExceptionServoMotor(msg);
	}

	return deg;
};

float ServoMotor::getPositionInRad(){
	float rad;
	short deg;
	unsigned short pos;
	try{
		pos = this->ServoMotorPololuBase::getPositionInAbs();
		deg = this->mapPosValue2DegValue(pos);
		rad = this->deg2rad(deg);
	}catch(IException *e){
		string msg("getPositionInRad:: error while trying to read servo motor position.");
		msg += e->getMsg();
		throw new ExceptionServoMotor(msg);
	}catch(...){
		string msg("getPositionInRad:: unknown error while trying to read servo motor position.");
		throw new ExceptionServoMotor(msg);
	}
	return rad;
};

void ServoMotor::showPololuValues (unsigned short& min, unsigned short& mid, unsigned short& max){
	mid = neutralPosition_;
	max = mid + delta_;
	min = mid - delta_;
	return;
};

float ServoMotor::deg2rad(unsigned short x){
	return (((float)x)*M_PI / 180.0);
};

short ServoMotor::rad2deg(float x){
	short d = (short) (((((float)x)*180.0) / M_PI) + 0.5);
	return d;
};

unsigned short ServoMotor::mapDegValue2PosValue(short d){
	float fact, pos;
	unsigned short servoPos;

	fact = ((float) this->maxDeg_) - ((float) this->minDeg_);
	fact = ( 2.0 * ((float) this->delta_) ) / fact;

	pos  =  fact * ( ((float) d) - ((float) this->minDeg_) );
	pos  = pos + ((float) (this->neutralPosition_ - this->delta_));

	servoPos = (unsigned short) pos;
	return (servoPos);
};

short ServoMotor::mapPosValue2DegValue(unsigned short p){
	float fact, pos;
	short deg;

	fact = ((float) this->maxDeg_) - ((float) this->minDeg_);
	fact = fact / (2.0*((float) this->delta_));

	pos = fact * ( ((float)  p) - ((float) (neutralPosition_ - delta_)) );
	pos = pos + ((float) minDeg_);

	deg = (short) pos;
	return (deg);
};


