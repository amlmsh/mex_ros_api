//============================================================================
// Name        : Pololu.cpp
// Author      : Willi Penner, Martin Huelse (martin.huelse@fh-bielefeld.de
//
// Description : Pololu source file. It contains the definition of the
//               functions of the Pololu class.
//============================================================================
#include "Pololu.hpp"
#include "SerialCom.hpp"
#include <string>
#include <iostream>

Pololu::Pololu(){
	throw new ExceptionPololu(string("This unparameterized constructor shall not be called."));
}


Pololu::Pololu(const char* portName, unsigned short baudRate){
	try{
		isComPortOpen_ = false;
		serialCom_ = new SerialCom(portName, baudRate);
		if(serialCom_ == nullptr){
			string msg("Pololu(Contructor)::Could not create a SerialCom instance.");
			throw new ExceptionPololu(msg);
		}
	}catch(IException *e){
		throw e;
	}catch(...){
		string msg("Pololu(Contructor)::Unknown error while creating a SerialCom instance.");
		throw new ExceptionPololu(msg);
	}
}


Pololu::~Pololu(){
	if(serialCom_ != nullptr){
		serialCom_->closeSerialCom();
		delete serialCom_;
	}
	isComPortOpen_ = false;
	return;
}

void Pololu::openConnection(){
    try{
    	serialCom_->closeSerialCom();
        serialCom_->openSerialCom();
        isComPortOpen_ = true;
        return;
    }catch(ExceptionSerialCom  *e){
    	isComPortOpen_ = false;
    	throw new ExceptionPololu(string("openConnection::") + e->getMsg());
    }catch (std::string &errorMessage){
    	isComPortOpen_ = false;
    	throw new ExceptionPololu(string("openConnection::") + errorMessage);
    }catch(...){
    	isComPortOpen_ = false;
    	throw new ExceptionPololu(string("openConnection::Unknown error while openSerial"));
    }
}

void Pololu::closeConnection(){
    try{
        serialCom_->closeSerialCom();
        isComPortOpen_ = false;
        return;
    }catch(ExceptionSerialCom  *e){
    	isComPortOpen_ = false;
    	throw new ExceptionPololu(string("closeConnection::") + e->getMsg());
    }catch (std::string &errorMessage){
    	isComPortOpen_ = false;
    	throw new ExceptionPololu(string("closeConnection::") + errorMessage);
    }catch(...){
    	isComPortOpen_ = false;
    	throw new ExceptionPololu(string("closeConnection::Unknown error while executing closeSerial."));
    }
}

void Pololu::initConnection(const char* portName, unsigned short baudRate){
	try{
		this->closeConnection();
		isComPortOpen_ = false;
		serialCom_->initSerialCom(portName, baudRate);
	}catch (IException *e) {
		isComPortOpen_ = false;
		string msg("initConnection::Error while closing and initializing the serial com.");
		throw new ExceptionPololu(msg);
	}catch(...){
		isComPortOpen_ = false;
		string msg("initConnection:: Unknown error while closing and initializing the serial com.");
		throw new ExceptionPololu(msg);
	}
}


unsigned short Pololu::setPosition(unsigned short servo, unsigned short goToPosition){
	if(!isComPortOpen_){
		string msg("setPosition:: serial communication port is closed. ");
		msg += string("First call copenConnection methods.");
		throw new ExceptionPololu(msg);
	}


    /* Generates the command for the controller.
     * 0x84 = Pololu command for setting the position
     * servo = servo to address as a transfer parameter
     * goToPositiion = divided into 2 bytes, first the low bits, then the high bits
     */
    unsigned short sizeCommand = 4;
    unsigned char command[] = {0x84, (unsigned char)servo, (unsigned char)(goToPosition & 0x7F), (unsigned char)((goToPosition >> 7) & 0x7F)};
    try
    {
        serialCom_->writeSerialCom(command, sizeCommand, NULL, 0);
    }catch (IException *e){
        string msg("setPosition::error while sending the position data.");
        msg += e->getMsg();
        throw new ExceptionPololu(msg);
    }catch (std::string &errorMessage){
        string msg("setPosition::error while sending the position data.");
        msg += errorMessage;
        throw new ExceptionPololu(msg);
    }catch(...){
        string msg("setPosition::unknown error while sending the position data.");
        throw new ExceptionPololu(msg);
    }
    return goToPosition;
}


bool Pololu::setSpeed(unsigned short servo, unsigned short goToSpeed){
	if(!isComPortOpen_){
		string msg("setSpeed:: serial communication port is closed");
		msg += string("First call copenConnection.");
		throw new ExceptionPololu(msg);
	}


    /* Generates the command for the controller.
     * 0x87 = Pololu command for setting the speed
     * servo = servo to address as a transfer parameter
     * goToSpeed = divided into 2 bytes, first the low bits, then the high bits
     */
    unsigned short sizeCommand = 4;
    unsigned char command[] = {0x87, (unsigned char)servo, (unsigned char)(goToSpeed & 0x7F), (unsigned char)((goToSpeed >> 7) & 0x7F)};
    try
    {
        serialCom_->writeSerialCom(command, sizeCommand, NULL, 0);
    }catch (IException *e){
        string msg("setSpeed::error while sending the max speed data.");
        msg += e->getMsg();
        throw new ExceptionPololu(msg);
    }catch (std::string &errorMessage){
        string msg("setSpeed::error while sending the max speed data.");
        msg += errorMessage;
        throw new ExceptionPololu(msg);
    }catch(...){
        string msg("setSpeed::unknown error while sending the max speed data.");
        throw new ExceptionPololu(msg);
    }
    return true;
}


bool Pololu::setAcceleration(unsigned short servo, unsigned short goToAcceleration){
	if(!isComPortOpen_){
		string msg("setAcceleration:: serial communication port is closed");
		msg += string("First call copenConnection.");
		throw new ExceptionPololu(msg);
	}


    /* Generates the command for the controller.
     * 0x89 = Pololu command for setting the acceleration
     * servo = servo to address as a transfer parameter
     * goToAcceleration = divided into 2 bytes, first the low bits, then the high bits
     */
    unsigned short sizeCommand = 4;

    unsigned char command[] = {0x89, (unsigned char)servo, (unsigned char)(goToAcceleration & 0x7F), (unsigned char)((goToAcceleration >> 7) & 0x7F)};
    try
    {
        serialCom_->writeSerialCom(command, sizeCommand, NULL, 0);
    }catch (IException *e){
        string msg("setAcceleration::error while sending the max acceleration data.");
        msg += e->getMsg();
        throw new ExceptionPololu(msg);
    }catch (std::string &errorMessage){
        string msg("setAcceleration::error while sending the max acceleration data.");
        msg += errorMessage;
        throw new ExceptionPololu(msg);
    }catch(...){
        string msg("setAcceleration::error while sending the max acceleration data.");
        throw new ExceptionPololu(msg);
    }
    return 1;
}

unsigned short Pololu::getPosition(unsigned short servo){
	if(!isComPortOpen_){
		string msg("getPosition:: serial communication port is closed");
		msg += string("First call copenConnection.");
		throw new ExceptionPololu(msg);
	}


    /* Generates the command for the controller.
     * 0x90 = Pololu command for reading out the position
     * servo = servo to address as a transfer parameter
     */
    unsigned short sizeCommand = 2;
    unsigned short sizeResponse = 2;
    unsigned char response[sizeResponse];

    unsigned char command[] = {0x90, (unsigned char)servo};
    try
    {
        serialCom_->writeSerialCom(command, sizeCommand, response, sizeResponse);
    }catch (IException *e){
        string msg("getPosition::error while sending the 'get position' data:");
        msg += e->getMsg();
        throw new ExceptionPololu(msg);
    }catch (std::string &errorMessage){
        string msg("getPosition::error while sending the 'get position' data:");
        msg += errorMessage;
        throw new ExceptionPololu(msg);
    }catch(...){
        string msg("getPosition:: unknown error while sending the 'get position' data.");
        throw new ExceptionPololu(msg);
    }

    return response[0] + 256 * response[1];
}


bool Pololu::getMovingState(){
	if(!isComPortOpen_){
		string msg("getMovingState:: serial communication port is closed");
		msg += string("First call copenConnection.");
		throw new ExceptionPololu(msg);
	}

    /* Generates the command for the controller.
     * 0x93 = Pololu command for reading out the movement of all servos
     */
    unsigned short sizeCommand = 1;
    unsigned short sizeResponse = 1;
    unsigned char response[sizeResponse];

    unsigned char command[] = {0x93};
    try
    {
        serialCom_->writeSerialCom(command, sizeCommand, response, sizeResponse);
    }catch (IException *e){
        string msg("getMovingState:: error while sending the 'moving state' data:");
        msg += e->getMsg();
        throw new ExceptionPololu(msg);
    }catch (std::string &errorMessage){
        string msg("getMovingState:: error while sending the 'moving state' data:");
        msg += errorMessage;
        throw new ExceptionPololu(msg);
    }catch(...){
        string msg("getMovingState:: unknown error while sending the 'moving state' data.");
        throw new ExceptionPololu(msg);
    }
    return response[0];
}



unsigned short Pololu::getErrors(){
	if(!isComPortOpen_){
		string msg("getMovingState:: serial communication port is closed");
		msg += string("First call copenConnection.");
		throw new ExceptionPololu(msg);
	}

    /* Generates the command for the controller.
     * 0xA1 = Pololu command for reading out error flags
     */
    unsigned short sizeCommand = 1;
    unsigned short sizeResponse = 2;
    unsigned char response[sizeResponse];

    unsigned char command[] = {0xA1};

    int counter = 0;
    int limit = 5;
    while(1){
    	try
    	{
    		counter++;
     		serialCom_->writeSerialCom(command, sizeCommand, response, sizeResponse);
     		break;
    	}catch (IException *e){
    		if(counter < limit){
    			continue;
    		};
    		string msg("getErrors:: error while trying to read error data:");
    		msg += e->getMsg();
    		throw new ExceptionPololu(msg);
    	}catch (std::string &errorMessage){
       		if(counter < limit){
        			continue;
        	};
    		string msg("getErrors:: error while trying to read error data:");
    		msg += errorMessage;
    		throw new ExceptionPololu(msg);
    	}catch(...){
       		if(counter < limit){
        			continue;
        	};
    		string msg("getErrors:: unknown error while trying to read error data.");
    		throw new ExceptionPololu(msg);
    	}
    }
    return response[0] + (256 * response[1]);
}
