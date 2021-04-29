//============================================================================
// Name        : SerialCom.hpp
// Author      : Willi Penner
//
// Description : SerialCom header file. It contains the declaration of the
//               ISerialCom interface and the SerialCom class
//============================================================================
#ifndef SERIALCOM_HPP_INCLUDED
#define SERIALCOM_HPP_INCLUDED

#include <string>


#ifdef _WIN32
    #include <windows.h>
#else
	#include <iostream>
#endif


using namespace std;

/**
 *
 * \class SerialCom
 *
 *  \brief Interface for a serial connection via serial port.
 *  The implementation of this communication requires the
 *  configuration setting of the Pololu as "USB Dual Port"
 *  (see also https://www.pololu.com/docs/0J40/5.a).
 *
 */
class ISerialCom {
public:
	virtual ~ISerialCom(){};


    /** \brief Used to initiate the SerialCom object with port name and baud rate.
     *  The function puts the object in the same state as the constructor with transfer parameters.
     *  It can be used to change the port name and the baud rate of a SerialCom object.
     *
     *  \param portName : The port name is used to open a serial connection via the port name for the controller specified by the operating system.
     *  \param baudRate : The baud rate determines the transmission speed at which communication between the PC and controller takes place.
     */
    virtual void initSerialCom(const char* portName, unsigned short baudRate) = 0;

    /** \brief Opens the serial com defined by the initSerialCom(...) method  or the constructor
     * parameter. If the port can be successfully be opened the method retuns true.
     * If an error occurs an exception (IException) is thrown.
     * If the serial com is already  open an exception (IException) is thrown.
     *
     *  \return Returns TRUE on successful opening of a serial connection.
     *  \throws IException
     */
    virtual bool openSerialCom() = 0;

    /** \brief Closes a serial connection.
     *	If an error occurs an exception (IException) is thrown.
     *  \return Returns true on successful closing of a serial connection.
     */
    virtual bool closeSerialCom() = 0;

    /**
     *
     *
     * \brief This method is used to write commands to and read response from the controller via
     * the open serial connection. If the serial port is not open an
     * exception (IException) is thrown. If writing or reading fails, an exception is thrown.
     *
     * \param command[] : Contains the command to be sent
     *                    (size of 1, 2 or 4 bytes, depending on the command).
     *
     * \param sizeCommand : Contains the size (in bytes) of the command (1, 2 or 4).
     *
     * \param response : Array of the given size (sizeResponse) where the response of the micro-controller
     *                   can be / is stored.
     *                   If no return value is expected, the pointer can be NULL.
     *
     * \param sizeResponse : Contains the size of the command (1 or 2, in case of no
     *                       response is expected the value has to be 0).  Thus, a sizeResponse value
     *                       of 0 indicates that no response from the micro-controller
     *                       will be read.
     *
     * \return Returns true if writing / and reading was done successfully.
     *
     */
    virtual bool writeSerialCom(unsigned char command[], unsigned short sizeCommand, unsigned char *response, unsigned short sizeResponse) = 0;
};




class SerialComBase : public ISerialCom{
	protected:
		bool  isSerialComOpen_ = false;
		const char* portName_ = nullptr;
		unsigned short baudRate_ = 0;
};


#ifdef _WIN32
	class SerialComWIN32 : public SerialComBase{
		public:
				 SerialComWIN32(const char* portName="COM0", unsigned short baudRate=9600);
			void initSerialCom (const char* portName="COM0", unsigned short baudRate=9600);
			bool openSerialCom ();
			bool closeSerialCom();
			bool writeSerialCom(unsigned char command[], unsigned short sizeCommand, unsigned char *response, unsigned short sizeResponse);
	        HANDLE getPort();
		protected:
	        HANDLE port_;
	};
#else
	class SerialComLINUX : public SerialComBase{
		public:
				 SerialComLINUX(const char* portName="/dev/ttyACM0", unsigned short baudRate=9600);
		    void initSerialCom(const char* portName="/dev/ttyACM0", unsigned short baudRate=9600);
		    bool openSerialCom();
		    bool closeSerialCom();
		    bool writeSerialCom(unsigned char command[], unsigned short sizeCommand, unsigned char *response, unsigned short sizeResponse);
		    int  getPort();
		protected:
		    int port_;
	};
#endif


#ifdef _WIN32
	class SerialCom : public SerialComWIN32{
	public:
		SerialCom(const char* portName="COM0",
				unsigned short baudRate=9600) : SerialComWIN32(portName,baudRate){};
	};
#else
	class SerialCom : public SerialComLINUX{
	public:
		SerialCom(const char* portName="/dev/ttyACM0",
				unsigned short baudRate=9600) : SerialComLINUX(portName,baudRate){};
		~SerialCom(){
				this->closeSerialCom();
		};
	};
#endif


/**
 *
 * \class IException
 *
 * \brief Interface for exceptions to be overwritten
 * by the derived classes.
 *
 */
class IException{
	public:

		/**
		 *
		 * \return string error messages
		 *
		 */
		virtual string getMsg() = 0;
};


/**
 *
 * \class ExceptionSerialCom
 *
 * \brief Exception class for class \ref SerialCom
 *
 *
 */
class ExceptionSerialCom : public IException{
	public:

		/**
		 *
		 * \brief Constructor of the class
		 *
		 * \param string error message
		 *
		 */
		ExceptionSerialCom(string msg){
			msg_ = string("ExceptionSerialCom::") + msg;
		};

		/**
		 * Returns the error message.
		 *
		 * \return string containing the error message
		 *
		 */
		string getMsg(){return msg_;}
	protected:
		string msg_;
	private:
		ExceptionSerialCom(){};
};


#endif // SERIALCOM_HPP_INCLUDED
