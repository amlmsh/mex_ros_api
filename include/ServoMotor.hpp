//============================================================================
// Name        : ServoMotor.hpp
// Author      : Willi Penner / martin huelse (martin.huelse@fh-bielefeld.de)
//
// Description : Header file contains the interface IServoMotor and the class
//               ServoMotor with its functions to control a specific servo.
//============================================================================
#include "Pololu.hpp"
#include <cmath>

#ifndef SERVOMOTOR_HPP_
#define SERVOMOTOR_HPP_


/**
 *
 * \class IServoMotorBase
 *
 * \brief Interface class that specifies the basic
 * functionality of object that initializes and
 * controls a servo motor.
 *
 * The positions of the servo motor are defined
 * by positions values (given in units).
 *
 *
 */
class IServoMotorBase{
public:
	virtual ~IServoMotorBase(){};

	/**
	 *
	 * \brief Delivers the ID of the servo motor instance.
	 * This ID might related to certain connectors of the
	 * board / HW used to control the servo motor.
	 * This must be specified in the actual implementation
	 * of this inteface by the derived classes.
	 *
	 *  \return unsigned int number of the connector of the servo
	 *  motor on the board.
	 *
	 */
	virtual unsigned short getServoNumber() = 0;

	/**
	 *
	 * \brief Delivers the minimal position value the servo
	 * motor can / shall move to.
	 *
	 *  \return unsigned short
	 *
	 */
	virtual unsigned short getMinPosInAbs() = 0;

	/**
	 *
	 * \brief Delivers the position value in units for the so called
	 * neutral position of a servo motor given in units.
	 * This position might specifically defined by the developer
	 * for specific servo motors.
	 *
	 *  \return unsigned short
	 *
	 */
	virtual unsigned short getMidPosInAbs() = 0;

	/**
	 *
	 * \brief Delivers the maximal position value the servo motor
	 * can / shall move to.
	 *
	 *  \return unsigned short
	 *
	 */
	virtual unsigned short getMaxPosInAbs() = 0;

	/**
	 *
	 *
	 * \brief Moves the servo motor to the given position.
	 *
	 *  \param newPosition unsigned short position value given
	 *                     in units.
	 *
	 *  \return unsigned short actual position set given in units.
	 */
	virtual unsigned short setPositionInAbs(unsigned short newPosition) = 0;



	/**
	 *
	 *
	 * \brief Delivers the position value (in units) the servo trageted at.
	 *
	 *  \return unsigned short.
	 *
	 */
	virtual unsigned short getPositionInAbs() = 0;

};

/**
 *
 * \class IServoMotoBaseAdv
 *
 * \brief Interface class that specifies additional
 * functionality with respect to set maximal speed
 * and acceleration.
 *
 *
 */
class IServoMotoBaseAdv {
public:
	virtual ~IServoMotoBaseAdv(){};

	/**
	 *
	 *  \brief Sets the maximal speed the servo shall move.
	 *
	 *  \param unsigned short newSpeed (the meaning of the values
	 *      has to be specified in the derived class.)
	 *
	 *  \return unsigned short actual speed value set.
	 */
	virtual unsigned short setSpeed(unsigned short newSpeed) = 0;

	/**
	 *
	 *  \brief Sets the maximal acceleration the servo shall move.
	 *
	 *  \param unsigned short newAcceleration (the meaning of the values
	 *      has to be specified in the derived class.)
	 *
	 *  \return unsigned short actual accelaration value set.
	 */
	virtual unsigned short setAcceleration(unsigned short newAcceleration) = 0;

};



/**
 *
 *
 * \brief Interface to control a servo motor.
 * The interface provides the basic functions for the
 * control for a specific servo motor.
 *
 * Note, different types of servo motors have different specification
 * with respect to working range and the counting of position units
 * representing certain motor positions.
 * Furthermore, implementation of this interface can also differe
 * because of the boards used to drive the servo motors.
 * These parameters (working area and units) must be specified / documented
 * in the derived classes that implement these methods.
 *
 */
class IServoMotor {
public:
	virtual ~IServoMotor(){};

	/**
	 *
	 *
	 * \brief Moves the servo motor to the given position.
	 *
	 *  \param newPosition unsigned short position value given
	 *                     in degree.
	 *
	 *  \return unsigned short actual position set in degree.
	 */
	virtual short setPositionInDeg(short newPosition) = 0;

	/**
	 *
	 *
	 * \brief Moves the servo motor to the given position.
	 *
	 *  \param newPosition unsigned short position value given
	 *                     in radian.
	 *
	 *  \return unsigned short actual position set in radian
	 */
	virtual float setPositionInRad(float newPosition) = 0;

	/**
	 *
	 *
	 * \brief  Delivers the position value (in degree) the servo trageted at.
	 *
	 *  \return unsigned short.
	 *
	 */
	virtual short getPositionInDeg() = 0;

	/**
	 *
	 *
	 * \brief Delivers the position value (in radian) the servo trageted at.
	 *
	 *  \return unsigned short.
	 *
	 */
	virtual float getPositionInRad() = 0;


	/**
	 *
	 *
	 * \brief
	 *
	 *  Shows which settings have to be made in the Pololu Maestro Control Center for the servo,
	 *  based on the starting position and the delta for the specific servo.
	 *  See Pololu manual for further explainations.
	 */
	virtual void showPololuValues (unsigned short& min, unsigned short& mid, unsigned short& max) = 0;
};






/**
 *
 * \class ServoMotorPololuBase
 *
 *
 * \brief Class implements the interface IServoMotorBase
 * based on the pololu controller board
 * (https://www.pololu.com/docs/pdf/0J40/maestro.pdf).
 *
 */
class ServoMotorPololuBase : public IServoMotorBase{
public:

	/**
	 *
	 * \brief Parameterized constructor. In case of an error
	 * 			an exception is thrown.
	 *
	 *	\param servoID unsigned short. Index (>= 0) of the servo motor that
	 * 					identifies which slot / pin on the
	 * 					micro controller board the servo is
	 * 					connected with.
	 *
	 * 	\param neutralPos unsigned short. (neutralPos > 0) Neutral position of the servo,
	 * 					usually defined by an impulse of 1'500 micro seconds
	 * 					impulse length.
	 * 					The position value is usually given by impulse length per
	 * 					one micro seconds. Using the pololu micro-controller to
	 * 					control the servor motor the impulse length is given per
	 * 					units of 1/4 of a quarter of the micro-second.
	 * 					Therefore, position here is defined impulse length per
	 * 					one micro second per 1/4 micro second.
	 * 					Thus, position value = impulse lengths * 4 (units).
	 *					See Pololu manual for further explanations.
	 *
	 *	\param delta unsigned short. (delta < neutralPos) Defines the range of positions values,
	 * 					starting from the neutral positions +/- delta.
	 *
	 *
	 * 	\param *pololuController. Pointer to a pololu instance that controls the servo motors.
	 */
	ServoMotorPololuBase(unsigned short  servoID,
						 unsigned short  neutralPos,
						 unsigned short  delta,
						        IPololu *pololuController);
	~ServoMotorPololuBase();
	unsigned short getServoNumber();
	unsigned short getMinPosInAbs();
	unsigned short getMidPosInAbs();
	unsigned short getMaxPosInAbs();
	unsigned short setPositionInAbs(unsigned short newPosition);
	unsigned short getPositionInAbs();
protected:
	/**
	 *
	 * \var pololuCtrl_
	 *
	 * \brief Pointer to a API-object of the
	 * pololu controller (\ref class \IPololu).
	 */
	IPololu *pololuCtrl_ = NULL;


	/**
	 *
	 * \var servoNmb_
	 *
	 * \brief Index of the servo motor that
	 * identifies which slot / pin on the
	 * micro controller board the servo is
	 * connected with.
	 */
	unsigned int servoNmb_;

	/**
	 *
	 * \var neutralPosition_
	 *
	 * \brief Neutral position of the servo, usually defined by
	 * an impulse of 1'500 micro seconds impulse length.
	 * The position value is usually given by impulse length per
	 * one micro seconds. Using the pololu microcontroller to
	 * control the servor motor the impulse length is given per
	 * units of 1/4 of a quarter of the micro-second.
	 * Therefore, position here is defined impulse length per
	 * one micro second per 1/4 micro second.
	 * Thus, position value = impulse lengths * 4 (units).
	 *
	 * See Pololu manual for further explanations.
	 */
	unsigned short neutralPosition_;

	/**
	 *
	 * \var delta_
	 *
	 * \brief Defines the range of positions values.
	 * Starting from the neutral positions +/- delta.
	 *
	 */
	unsigned short delta_;

	ServoMotorPololuBase(){pololuCtrl_ = NULL;};
};

/**
 *
 * \class ServoMotorPololuBaseAdv
 *
 * \brief Implements the interface IServoMotoBaseAdv derived from
 * class ServoMotorPololuBase.
 *
 */
class ServoMotorPololuBaseAdv : public ServoMotorPololuBase, public IServoMotoBaseAdv{
public:

	/**
	 *
	 * \brief Parameterized constructor. In case of an error
	 * 			an exception is thrown.
	 *
	 *	\param servoID unsigned short. Index (>= 0) of the servo motor that
	 * 					identifies which slot / pin on the
	 * 					micro controller board the servo is
	 * 					connected with.
	 *
	 * 	\param neutralPos unsigned short. (neutralPos > 0) Neutral position of the servo,
	 * 					usually defined by an impulse of 1'500 micro seconds
	 * 					impulse length.
	 * 					The position value is usually given by impulse length per
	 * 					one micro seconds. Using the pololu micro-controller to
	 * 					control the servor motor the impulse length is given per
	 * 					units of 1/4 of a quarter of the micro-second.
	 * 					Therefore, position here is defined impulse length per
	 * 					one micro second per 1/4 micro second.
	 * 					Thus, position value = impulse lengths * 4 (units).
	 *					See Pololu manual for further explanations.
	 *
	 *	\param delta unsigned short. (delta < neutralPos) Defines the range of positions values,
	 * 					starting from the neutral positions +/- delta.
	 *
	 *
	 * 	\param *pololuController. Pointer to a pololu instance that controls the servo motors.
	 */
	ServoMotorPololuBaseAdv(unsigned short  servoID,
							unsigned short  neutralPos,
							unsigned short  delta,
							IPololu *pololuController) :
		ServoMotorPololuBase(servoID,neutralPos,delta,pololuController){;};
	~ServoMotorPololuBaseAdv(){pololuCtrl_ = NULL;};
	unsigned short setSpeed(unsigned short newSpeed);
	unsigned short setAcceleration(unsigned short newAcceleration);
protected:
	ServoMotorPololuBaseAdv(){throw new ExceptionPololu(string("NIY"));};
};


class ServoMotor : public ServoMotorPololuBaseAdv, public IServoMotor{
public:

	/**
	 *
	 * \brief Parameterized constructor. In case of an error
	 * 			an exception is thrown.
	 *
	 *	\param servoID unsigned short. Index (>= 0) of the servo motor that
	 * 					identifies which slot / pin on the
	 * 					micro controller board the servo is
	 * 					connected with.
	 *
	 * 	\param neutralPos unsigned short. (neutralPos > 0) Neutral position of the servo,
	 * 					usually defined by an impulse of 1'500 micro seconds
	 * 					impulse length.
	 * 					The position value is usually given by impulse length per
	 * 					one micro seconds. Using the pololu micro-controller to
	 * 					control the servor motor the impulse length is given per
	 * 					units of 1/4 of a quarter of the micro-second.
	 * 					Therefore, position here is defined impulse length per
	 * 					one micro second per 1/4 micro second.
	 * 					Thus, position value = impulse lengths * 4 (units).
	 *					See Pololu manual for further explanations.
	 *
	 *	\param delta unsigned short. (delta < neutralPos) Defines the range of positions values,
	 * 					starting from the neutral positions +/- delta.
	 *
	 *
	 * 	\param *pololuController. Pointer to a pololu instance that controls the servo motors.
	 */
	ServoMotor(unsigned short  servoID,
					 unsigned short  neutralPos,
					 unsigned short  delta,
						    IPololu *pololuController) :
				ServoMotorPololuBaseAdv(servoID,neutralPos,delta,pololuController){
		maxDeg_ = 180;
		minDeg_ = 0;
	};

	~ServoMotor(){pololuCtrl_ = NULL;};

	/**
	 *
	 * \brief Maps the given minimal degree value to the minimal position value and
	 * the given maximal degree value to the maximal position value.
	 * Per default the maximal position value is defined as 180 degree, while
	 * the minimal position is defined as 0 degree.
	 *
	 *	Note this method call overwrites values set by method setMinMaxRadian
	 *	and vice versa.
	 *
	 */
	void  setMinMaxDegree(short minDegree, short maxDegree);

	/**
	 *
	 * \brief Maps the given minimal degree value to the minimal position value and
	 * the given maximal degree value to the maximal position value.
	 * Per default the maximal position value is defined as 2*PI rad, while
	 * the minimal position is defined as 0 rad.
	 *
	 *	Note this method call overwrites values set by method setMinMaxRadian
	 *	and vice versa.
	 *
	 */
	void  setMinMaxRadian(float minRadian, float maxRadian);

	short setPositionInDeg(short newPosition);
	float setPositionInRad(float newPosition);
	short getPositionInDeg();
	float getPositionInRad();
	void  showPololuValues(unsigned short& min, unsigned short& mid, unsigned short& max);
protected:
	ServoMotor(){throw ExceptionPololu(string("NIY"));};
private:
	short          minDeg_ = 0;
	short          maxDeg_ = 180;
	float          deg2rad(unsigned short x);
	short          rad2deg(float x);
	unsigned short mapDegValue2PosValue(short d);
	short          mapPosValue2DegValue(unsigned short p);
};






/**
 *
 * \class ExceptionServerMotor
 *
 * \brief Implementation of the exception class for
 * class ServoMotor
 *
 */
class ExceptionServoMotor : public IException{
public:
	ExceptionServoMotor(string msg){
		msg_ = string("ExceptionServoMotor::") + msg;
	};
	string getMsg(){return msg_;}
protected:
	string msg_;
private:
	ExceptionServoMotor(){};
};

class ExceptionServoMotorBase : public IException{
public:
	ExceptionServoMotorBase(string msg){
		msg_ = string("ExceptionServoMotorBase::") + msg;
	};
	string getMsg(){return msg_;}
protected:
	string msg_;
private:
	ExceptionServoMotorBase(){};
};

class ExceptionServoMotorBaseAdv : public IException{
public:
	ExceptionServoMotorBaseAdv(string msg){
		msg_ = string("ExceptionServoMotorBaseAdv::") + msg;
	};
	string getMsg(){return msg_;}
protected:
	string msg_;
private:
	ExceptionServoMotorBaseAdv(){};
};




#endif /* SERVOMOTOR_HPP_ */
