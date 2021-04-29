//============================================================================
// Name        : SerialCom.cpp
// Author      : Willi Penner
//
// Description : SerialCom source file. It contains the definition of the
//               functions of the SerialCom class.
//============================================================================
#include "SerialCom.hpp"
#include <stdio.h>
#include <string>
#include <iostream>
#include <sstream>

#ifdef _WIN32
    #include <windows.h>
#else
	#include <fcntl.h>
	#include <unistd.h>
	#include <stdint.h>
	#include <termios.h>
	#include <stdbool.h>
#endif



#ifdef _WIN32
    	/**
    	 *
    	 * Implementation of the serial com class for WIN32
    	 *
    	 */

	 	 SerialComWIN32(const char* portName="COM0",
	 			 	 	 unsigned short baudRate=9600){
	 		portName_ = portName;
	 		baudRate_ = baudRate;
	 		port_ = NULL;
	 	 }

	 	 void initSerialCom (const char* portName="COM0",
	 			 	 	 	 unsigned short baudRate=9600){
	 		try{
	 			/**< Before a serial connection is reinitialized, a possible open connection is closed. */
	 			if(isSerialComOpen_){
	 				CloseHandle(port_);
	 				isSerialComOpen_ = false;
	 			};
	 			portName_ = portName;
	 			baudRate_ = baudRate;
	 		}catch(IException *e){
	 			string msg("(caught IException error :: initSerialCom::");
	 			msg += string("throw ExceptionSerialCom: error while closing an open serial connection:: ");
	 			msg += e->getMsg() + string(")");
	 			throw new ExceptionSerialCom(msg);
	 		}catch(string m){
	 			string msg("(caught string error :: initSerialCom::");
	 			msg += string("throw ExceptionSerialCom: error while closing an open serial connection:: ");
	 			msg += m + string(")");
	 			throw new ExceptionSerialCom(msg);
	 		}catch(...){
	 			string msg("(caught unknown error :: initSerialCom::");
	 			msg += string("throw ExceptionSerialCom: error while closing an open serial connection:: ");
	 			msg += string(")");
	 			throw new ExceptionSerialCom(msg);
	 		}

	 	    return;
	 	 }

	 	 bool openSerialCom (){
	 		bool success = false;

	 		/**< Opens a serial connection using the CreateFileA function from <windows.h>. Port_ is
	          	 opened with read and write access. */
	 		try{
	 			port_ = open(portName_, O_RDWR | O_NOCTTY); //you have to set the permission for the /dev/ttyACM0
	 			if (port_ == -1){
	 				string msg = string("openSerialCom: LINUX Failed to open port '") + string(portName_) + string("'.");
	 				throw new ExceptionSerialCom(msg);
	 			}
	 		}catch(...){
	 			string msg("openSerialCom: LINUX unknown Error while open serial port '");
	 			msg += string(portName_) + string("'.");
	 			throw new ExceptionSerialCom(msg);
	 		}
	 		/**< Flushes the file buffer of the opened connection. */
	 		success = tcflush(port_, TCIOFLUSH);
	 		if (success){
	 			throw std::string("SerialCom::openSerialCom: Failed to flush file buffer.\n");
	 			close(port_);
	 			return 0;
	 		}
	 		// Get the current configuration of the serial port.
	 		struct termios options;
	 		success = tcgetattr(port_, &options);
	 		if (success){
	 			throw std::string("SerialCom::openSerialCom: Failed to get serial settings.\n");
	 			close(port_);
	 			return 0;
	 		}
	 		// Turn off any options that might interfere with our ability to send and
	 		// receive raw binary bytes.
	 		options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
	 		options.c_oflag &= ~(ONLCR | OCRNL);
	 		options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

	 		// Set up timeouts: Calls to read() will return as soon as there is
	 		// at least one byte available or when 100 ms has passed.
	 		options.c_cc[VTIME] = 1;
	 		options.c_cc[VMIN] = 0;

	 		// This code only supports certain standard baud rates. Supporting
	 		// non-standard baud rates should be possible but takes more work.
	 		cfsetospeed(&options, B9600);
	 		cfsetispeed(&options, cfgetospeed(&options));
	 		success = tcsetattr(port_, TCSANOW, &options);
	 		if (success){
	 			throw std::string("SerialCom::openSerialCom: Failed to set serial settings.\n");
	 			close(port_);
	 			return false;
	 		}
	 		return true;
	 	 }

	 	 bool closeSerialCom(){
	 		 if (CloseHandle(port_) == 0){
	 			 throw std::string("SerialCom::closeSerialCom: Failed to close port.\n");
	 			 return 0;
	 		 }
	 		 return 1;
	 	 }

	 	 bool writeSerialCom(unsigned char command[],
	 			 	 	 	 unsigned short sizeCommand,
	 			 	 	 	 unsigned char *response,
	 			 	 	 	 unsigned short sizeResponse){

	 		if ((sizeCommand != 1) && (sizeCommand != 2) && (sizeCommand != 4)){
	 			throw std::string("SerialCom::writeSerialCom: wrong parameter sizeCommand, allowed parameter 1,2 or 4.");
	 		}

	    	DWORD bytesTrasfered; //Is given to the write or read command as a pointer. After executing the WriteFile or ReadFile, bytesTranfered contains the number of bytes transmitted or received.
	    	bool success = 0;

	    	//** Sending the command to the controller via port_. */
	    	success = WriteFile(port_, cmd, sizeCmd, &bytesTrasfered, NULL);
	    	if (!success){
	    		throw std::string("SerialCom::writeSerialCom: Failed to write to port.");
	    		return 0;
	    	}

	    	//** Check whether data needs to be read. */
	    	if (sizeRes > 0){
	    		success = ReadFile(port_, (void *)res, sizeRes, &bytesTrasfered, NULL);
	    	}else{
	    		return 1; //No need to read data. Confirmation of the successful writing of the data.
	    	}
	    	if (!success){
	    		throw std::string("SerialCom::writeSerialCom: Failed to read from port.");
	    		return 0;
	    	}
	    	return true; //Confirmation that data was written and read data were saved in the response array.
	 	 }


	 	 HANDLE getPort(){
	 		 return port_;
	 	 }

#else
    	/**
    	 *
    	 * Implementation of the serial com class for LINUX
    	 *
    	 */
    	SerialComLINUX::SerialComLINUX(const char* portName,
    									unsigned short baudRate){
    		isSerialComOpen_ = false;
    		portName_ = portName;
    		baudRate_ = baudRate;
    		port_ = 0;
    		return;
    	}

    	void SerialComLINUX::initSerialCom(const char* portName,
    										unsigned short baudRate){
    		try{
    			/**< Before a serial connection is reinitialized, a possible open connection is closed. */
    			if(isSerialComOpen_){
        			string msg("initSerialCom:: port is already open, close port first before initialization.");
        			throw new ExceptionSerialCom(msg);
    			};
    			portName_ = portName;
    			baudRate_ = baudRate;
    		}catch(IException *e){
    			string msg("(caught IException error :: initSerialCom::");
    			msg += string("throw ExceptionSerialCom: error while closing an open serial connection:: ");
    			msg += e->getMsg() + string(")");
    			throw new ExceptionSerialCom(msg);
    		}catch(string m){
    			string msg("(caught string error :: initSerialCom::");
    			msg += string("throw ExceptionSerialCom: error while closing an open serial connection:: ");
    			msg += m + string(")");
    			throw new ExceptionSerialCom(msg);
    		}catch(...){
    			string msg("(caught unknown error :: initSerialCom::");
    			msg += string("throw ExceptionSerialCom: error while closing an open serial connection:: ");
    			msg += string(")");
    			throw new ExceptionSerialCom(msg);
    		}

    	    return;
    	};

    	bool SerialComLINUX::openSerialCom(){
    		int success = -1;

    		if(isSerialComOpen_){
    			string msg("openSerialCom:: port is already open, close port first before open it.");
    			throw new ExceptionSerialCom(msg);
    		}

    		port_ = open(portName_, O_RDWR | O_NOCTTY); //success requires  permission
    		if (port_ == -1){
    			string msg("openSerialCom: LINUX cannot open port, check permission of '");
    			msg += string(portName_) + string("'.");
    			throw new ExceptionSerialCom(msg);
    		}

    		/**< Flushes the file buffer of the opened connection. */
    		success = tcflush(port_, TCIOFLUSH); // function delivers value 0 on success, -1 otherwise
    		if (success != 0){
    			close(port_);
    			string msg("SerialCom::openSerialCom: Failed to flush file buffer. Close port again.");
    			throw new ExceptionSerialCom(msg);
    		}

    		// Get the current configuration of the serial port.
    		struct termios options;
    		success = tcgetattr(port_, &options);
    		if (success != 0){
    			close(port_);
    			string msg("SerialCom::openSerialCom: Failed to read serial settings.");
    			msg += string(portName_) + string(" setting cannot be read. Port closed again.");
    			throw new ExceptionSerialCom(msg);
    		}

    		// Turn off any options that might interfere with our ability to send and
    		// receive raw binary bytes.
    		options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
    		options.c_oflag &= ~(ONLCR | OCRNL);
    		options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    		// Set up timeouts: Calls to read() will return as soon as there is
    		// at least one byte available or when 100 ms has passed.
    		options.c_cc[VTIME] = 1;
    		options.c_cc[VMIN] = 0;

    		// This code only supports certain standard baud rates. Supporting
    		// non-standard baud rates should be possible but takes more work.
    		cfsetospeed(&options, B9600);
    		cfsetispeed(&options, cfgetospeed(&options));
    		success = tcsetattr(port_, TCSANOW, &options);
    		if (success != 0){
       			close(port_);
    			string msg("SerialCom::openSerialCom: Failed to set serial settings.");
    			msg += string(portName_) + string(" parameters cannot be written. Port closed again.");
    			throw new ExceptionSerialCom(msg);
    		}

    		isSerialComOpen_ = true;
    		return true;
    	};

    	bool SerialComLINUX::closeSerialCom(){
    		if(isSerialComOpen_){
    			try{
    				close(port_);
    			}catch(...){
    				string msg("closeSerialCom:: error while trying to close port.");
    				msg += string("state of port is now undefined.");
    				throw ExceptionSerialCom(msg);
    			}
    		}
    		isSerialComOpen_ = false;
     		return true;
    	};

    	bool SerialComLINUX::writeSerialCom(unsigned char cmd[],
    										unsigned short sizeCmd,
    										unsigned char *res,
    										unsigned short sizeRes){
    		if(!isSerialComOpen_){
    			string msg("writeSerialCom:: port is not open yet, open port first before write/reading.");
    			throw new ExceptionSerialCom(msg);
    		}

    		// check parameter values of this function call
    		if ((sizeCmd != 1) && (sizeCmd != 2) && (sizeCmd != 4)){
    			string msg("SerialCom::writeSerialCom: wrong parameter sizeCommand,");
    			msg += string("allowed parameter values are either 1,2 or 4.");
    			throw new ExceptionSerialCom(msg);
    		}

    		if ((sizeRes != 0) && (sizeRes != 1) && (sizeRes != 2)){
    			string msg("SerialCom::writeSerialCom: wrong parameter sizeRes,");
    			msg += string("allowed parameter values are either 0,1 or 2.");
    			throw new ExceptionSerialCom(msg);
    		}


    		//** Sending the command to the controller via port_. */
    		if(write(port_, cmd, sizeCmd) == -1){
    			string msg("SerialCom::writeSerialCom: Failed to write to port '");
    			msg += string(portName_) + string("'.");
    			throw new ExceptionSerialCom(msg);
    		}

    		//** Check whether data needs to be read. */
    		if (sizeRes > 0){
    			unsigned counter = 0;
    			unsigned maxCounterValue = 20; // a kind of timeout
    			unsigned short dataRecv = 0;
    			unsigned short dataRecvTotal = 0;
    			do{ // try to read data from port
    				counter++;
    				dataRecv = read(port_, (void *)res, sizeRes);
    				if(dataRecv > 0){
    					dataRecvTotal += dataRecv;
    				}
    				if(maxCounterValue == counter) break;
    			}while(dataRecv != sizeRes);

    			if(dataRecv != sizeRes){
    				stringstream ss;
    				ss << "SerialCom::writeSerialCom: Failed while reading from port '";
    				ss << portName_ << "'.";
    				ss << "size of data (byte) received = " << dataRecvTotal << " unequal to expected";
    				ss << " data size to be received = " << sizeRes << ". ";
        			throw new ExceptionSerialCom(ss.str());
    			}
    		};
    		return true;
    	};

    	int  SerialComLINUX::getPort(){
    		if(isSerialComOpen_){
    			return port_;
    		};

    		string msg("SerialCom::getPort: Cannot provide port number '");
    		msg += string(portName_) + string("' is not open yet.");
    		msg += string ("Open port first before calling this methode.");
    		throw new ExceptionSerialCom(msg);
    	};

#endif // LINUX: end definition of methods


