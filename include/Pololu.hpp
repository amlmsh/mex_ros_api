//============================================================================
// Name        : Pololu.hpp
// Author      : Willi Penner, Martin Huelse (martin.huelse@fh-bielefeld.de)
//
// Description : Pololu header file. It contains the declaration of the
//               IPololu interface and the Pololu class
//============================================================================
#ifndef POLOLU_HPP_INCLUDED
#define POLOLU_HPP_INCLUDED

#include "SerialCom.hpp"


/**
 *
 * \brief Interface to control a Pololu controller. The interface
 *  provides the basic functions for the control of servo motors.
 *
 *
 */
class IPololu {
	friend class ServoMotor;
	friend class ServoMotorPololuBase;
	friend class ServoMotorPololuBaseAdv;
	friend class ServoMotorPololu;

public:

    /**
     *
     * \brief Moves motor to the specified position.
     * If an error occurs an exception is thrown.
     *
     *  \param unsigned short servoID. ID of the servo motor.
     *  \param unsigned short tragetPos. Target position to be reached.
     *
     *  \return unsigned short is undefined.
     *
     */
	virtual unsigned short setPosition(unsigned short servoID, unsigned short tragetPos) = 0;

    /**
     *
     *
     * \brief Sets the maximal speed of the specified servo motor.
     *	If an error occurs an exception is thrown.
     *
     *  \param unsigned short servoID. ID of the servo motor.
     *  \param unsigned maxSpeed. Maximal speed value.
     *
     *  \return unsigned short is undefined.
     *
     */
	virtual bool setSpeed(unsigned short servoID, unsigned short maxSpeed) = 0;


    /**
     *
     *
     * \brief Sets the maximal acceleration of the specified servo motor.
     *	If an error occurs an exception is thrown.
     *
     *  \param unsigned short servoID. ID of the servo motor.
     *  \param unsigned maxAccel. Maximal acceleration.
     *
     *  \return unsigned short is undefined.
     *
     */
	virtual bool setAcceleration(unsigned short servoID, unsigned short maxAccel) = 0;


    /**
     *
     * \brief Delivers the position value the specified motor is target at.
     * If an error occurs an exception is thrown.
     *
     *  \param unsigned short servoID. ID of the servo motor.
     *  \param unsigned short tragetPos. Target position to be reached.
     *
     *  \return unsigned short is undefined.
     *
     */
	virtual unsigned short getPosition(unsigned short servoID) = 0;


	virtual ~IPololu(){};

    /**
     *
     * \brief Returns the moving state of all motors the control board
     * is connected with.
     * If an error occurs an exception is thrown.
     *
     *
     *  \return bool. Return value is false if all motors don't move
     *  anymore, otherwise return value is true.
     *
     */
    virtual bool getMovingState() = 0;


    /**
     *
     * \brief Reads and returns all currently detected error codes.
     *
     * \return unsigned short. Errors codes represented as bit-codes.
     *
     *
     *
     */
    virtual unsigned short getErrors() = 0;
};

/**
 *
 * \brief Class that implements the interface IPololu for a Pololu
 * board that creates and manages serial communication object and
 * provides basic functions for programming the controller.
 *
 */
class Pololu : public IPololu {
friend class ServoMotor;
friend class ServoMotorPololuBase;
friend class ServoMotorPololuBaseAdv;
friend class ServoMotorPololu;

private:
	Pololu(); // throws just an exception if ever called

protected:
    SerialCom *serialCom_ = nullptr;
    bool isComPortOpen_ = false;

    unsigned short setPosition(unsigned short servo, unsigned short goToPosition);
    bool setSpeed(unsigned short servo, unsigned short goToSpeed);
    bool setAcceleration(unsigned short servo, unsigned short goToAcceleration);
    unsigned short getPosition(unsigned short servo);


public:
    /**
     *
     * \brief Constructor tries to open serial communication port having
     *  specified port name and baud rate. If the communication channel
     *  cannot be opened then a exception is thrown.
     *
     *  \param portName const char*. Name of the communication port.
     *  \param baudRate unsigned short. Baud rate parameter of serial communication
     *                                  port.
     *
     */
    Pololu(const char* portName, unsigned short baudRate);


    /**
     *
     * \brief Destructor. Closes the serial communicaton channel.
     *
     */
    ~Pololu();

    /**
     *
     * \brief Used to change the connection data. If the communication
     * channel is open then the channel is closed first.
     * The new connection parameters are set, but the channel will
     * not be opened. To open the channel the method openConnection
     * has to be called first.
     *
     * If an error occurs an exception is thrown.
     *
     *  \param portName const char*. Name of the communication port.
     *  \param baudRate unsigned short. Baud rate parameter of serial communication
     *                                  port.
     *
     */
    void initConnection(const char* portName, unsigned short baudRate);

    /**
     *
     * \brief Opens the serial connection with the currently specified
     * parameter (see method initConnection(...)).
     *
     * If an error occurs an exception is thrown.
     *
     *
     */
    void openConnection();


    /**
     *
     * \brief Closes the serial connection.
     *
     * If an error occurs an exception is thrown.
     *
     */
    void closeConnection();


    bool getMovingState();


    unsigned short getErrors();
};


class ExceptionPololu : public IException{
public:
	ExceptionPololu(string msg){
		msg_ = string("ExceptionPololu::") + msg;
	};
	string getMsg(){return msg_;}
protected:
	string msg_;
private:
	ExceptionPololu(){};
};

#endif // POLOLU_HPP_INCLUDED
