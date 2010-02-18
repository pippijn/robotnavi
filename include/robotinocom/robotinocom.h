//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _ROBOTINOCOM_H_
#define _ROBOTINOCOM_H_

#ifdef WIN32
#ifdef robotinocom_EXPORTS
#define ROBOTINOCOM_EXPORT __declspec(dllexport)
#else
#define ROBOTINOCOM_EXPORT //__declspec(dllimport)
#endif
#else
#define ROBOTINOCOM_EXPORT
#endif

#include <string>
#include "robotinocom/camerastuff.h"
#include "rec/robotino/com/northstar_interface.h"
#include "robotinocom/robotinoinfo.h"

using rec::robotino::com::NorthStarCommand;
using rec::robotino::com::NorthStarReadings;

class RobotinoComImpl;

/**
NoError : no error
ErrorConnectionRefused : if the connection was refused
ErrorSocketSend : if a send to the socket failed
ErrorSocketRead :  if a read from the socket failed
ErrorImageServerCreate : if the creation of the image server failed
ErrorImageServerBind : if the image server could not bind to the imagePort()
ErrorTimeout : if Robotino did not send something for more than timeout() milliseconds
ErrorConnectionBusy : if Robotino is already connected to someone else
ErrorClientCreate : if the creation of the client socket fails
ErrorHostNotFound : if a DNS lookup fails
ErrorUndefined : undefined error
*/
enum RobotinoComError {NoError,ErrorConnectionRefused,ErrorSocketSend,ErrorSocketRead,ErrorImageServerCreate,ErrorImageServerBind,
ErrorTimeout,ErrorConnectionBusy,ErrorClientCreate,ErrorHostNotFound,ErrorUndefined,ErrorWinsock2Initialization};



/**
IdleState : not connected
ConnectingState : after calling connectToHost(), if connectToHost() is successfull proceeds to ConnectedState otherwise to IdleState
ConnectedState : connected
*/
enum RobotinoComState {IdleState,ConnectingState,ConnectedState};

typedef void(*RobotinoErrorCb_t)( void* data, int error );
typedef void(*RobotinoConnectionClosedCb_t)( void* data );
typedef void(*RobotinoConnectedCb_t)( void* data );
typedef void(*RobotinoInfoReceivedCb_t)( void* data );
typedef void(*RobotinoImageReceivedCb_t)( void* userData, const unsigned char* imageData, unsigned int imageDataSize, RobotinoImageType imageType );

/**
* @brief	Provides complete commication with Robotino
*
* This class establishes a connection to Robotino and allows you to control its outputs
* and retrieve its inputs. Motors and other output devices can be controlled, camera images,
* distance sensors, bumpers and other input devices can be read.
*/
class ROBOTINOCOM_EXPORT RobotinoCom
{
public:
	RobotinoCom();
	~RobotinoCom();

	/**
	* Initialize the communication.
	*
	* @param initWinsock2	Defines wether the Winsock2 interface should be initialized.
	*						Has no effect on platforms other than Windows.
	* @return	TRUE if connection was successful, FALSE otherwise.
	* @throws	nothing.
	*/
	bool init( bool initWinsock2 = true );

	/**
	* Attempts to make a connection to the host and return immediately.
	* Any connection or pending connection is closed immediately.
	* When the connection succeeds, it emits connected(). isConnected() returns true from now on.
	* If there is an error at any point, it emits error().
	* @param hostname	An IP address in string form.
	* @param port		The used port to connect to the host.
	* @throws			nothing.
	* @see				isConnected, close
	*/
	void connectToHost( std::string hostname, int port );

	/**
	* Attempts to make a connection to the host and return immediately.
	* Any connection or pending connection is closed immediately.
	* When the connection succeeds, it emits connected(). isConnected() returns true from now on.
	* If there is an error at any point, it emits error().
	* This overload accepts host and port combinations of the form "HOSTNAME:PORT".
	* @param hostname	An IP address in string form. The port number may be added if separated by a colon.
	* @throws			nothing.
	* @see				isConnected, close
	*/
	void connectToHost( std::string hostname );

	/**
	* Close any connection immediately.
	* @throws	nothing.
	*/
	void close();

	/**
	* Sets the image port.
	* @param	port	The port number used for images.
	* @throws	nothing.
	*/
	void setImagePort( unsigned int port );
	/**
	* Retrieves the port number used for images.
	* @return	the image port number.
	* @throws	nothing.
	*/
	unsigned int imagePort() const;

	/**
	* Sets the server timeout. If update has not been called for more than timeout milliseconds the communication is closed by Robotino.
	* @param msecs	The server timeout in milliseconds.
	* @throws	nothing.
	* @see update
	*/
	void setTimeout( unsigned int msecs );
	/**
	* Retrieves the server timeout.
	* @return	Server timeout in milliseconds.
	* @throws	nothing.
	*/
	unsigned int timeout() const;

	/**
	* Returns the english description of the error.
	* @param error	The error number to look for.
	* @return	English description of given error number.
	* @throws	nothing.
	*/
	static std::string errorString( int error );

	/**
	* After calling stopAsyncCommunication() this allows to restart the asynchronous communication.
	* When connectToHost() is successful, i.e. the connected() signal is emitted, the asynchronous communication
	* thread is started. This thread receives status messages from Robotino and sends KeepAlive messages to keep
	* the connection open.
	* @throws	nothing.
	* @see		stopAsyncCommunication, connectToHost
	*/
	void startAsyncCommunication();

	/**
	* By stopping the asynchronous communication thread its up to you to call receive() periodically.
	* You are also responsible for sending control or KeepAlive messages. Otherwise the connection will time out.
	* @throws	nothing.
	* @see		startAsyncCommunication
	*/
	void stopAsyncCommunication();

	/**
	* Retrieves the current connection status.
	* @return	TRUE if there is a usable connection, FALSE otherwise.
	* @throws	nothing.
	* @see		connectToHost
	*/
	bool isConnected() const;

	/**
	* Returns the error code of the last error.
	* The error is cleared automatically by every call to connectToHost().
	* @return	The error code of the last encountered error.
	* @throws	nothing.
	* @see		errorString, clearError, RobotinoComError, connectToHost
	*/
	RobotinoComError error() const;

	/**
	* Sets the last error to RobotinoComError::NoError. Afterwards error() returns no error.
	* @throws	nothing.
	* @see		error, errorString
	*/
	void clearError();

	/**
	* Retrieves the state of the client object.
	* @return	TRUE if the client was created, FALSE otherwise.
	* @throws	nothing.
	*/
	bool isValid() const;

	/**
	* Sends the currently set values to Robotino and waits for the answer.
	* @return	TRUE on success, FALSE otherwise.
	* @throws	nothing.
	*/
	bool update();

	/**
	* Blocks until an image is received or the connection is closed.
	* @return	TRUE on success, FALSE otherwise.
	* @throws	nothing.
	*/
	bool receiveImage();

	/**
	* Sends the current set values to Robotino.
	* @return	TRUE if sending was successful.
	* @throws	nothing.
	*/
	/*bool sendControlMessage();*/

	/**
	* This callback function is called after connectToHost() has been called and
	* a connection has been successfully established.
	* @param f		The callback function.
	* @param data	User data that will be forwarded to the callback function.
	* @throws		nothing.
	* @see			connectToHost, isConnected
	*/
	void setConnectedCallback( RobotinoConnectedCb_t f, void* data );

	/**
	* This callback function is called after an error occurred.
	* If a connection is established a the connectionClosed() callback
	* is called afterwards.
	* @param f		The callback function.
	* @param data	User data that will be forwarded to the callback function.
	* @throws		nothing.
	* @see			errorString
	*/
	void setErrorCallback( RobotinoErrorCb_t f, void* data );

	/**
	* This callback function is called whenever a connection is closed,
	* i.e. afer calling close() or after an error occured.
	* @param f		The callback function.
	* @param data	User data that will be forwarded to the callback function.
	* @throws		nothing.
	* @see			close, error
	*/
	void setConnectionClosedCallback( RobotinoConnectionClosedCb_t f, void* data);

	/**
	* This callback function is called when a RobotinoInfo object is received.
	* @param f		The callback function.
	* @param data	User data that will be forwarded to the callback function.
	* @throws		nothing.
	*/
	void setRobotinoInfoReceivedCallback( RobotinoInfoReceivedCb_t f, void* data );

	/**
	* This callback function is called when an image is received.
	* @param f		The callback function.
	* @param data	User data that will be forwarded to the callback function.
	* @throws		nothing.
	* @see			lockImage, setImageRequest
	*/
	void setRobotinoImageReceivedCallback( RobotinoImageReceivedCb_t f, void* data );

	/**
	* Retrieve the latest image received.
	* After successfully locking an image make sure to call unlockImage() when you finished using the data.
	* @return	A valid pointer to a RobotinoImage on success, NULL otherwise.
	* @throws	nothing.
	* @see		unlockImage, setImageRequest, RobotinoImage
	*/
	const RobotinoImage* const lockImage();

	/**
	* Release the image obtained by lockImage().
	* @throws	nothing.
	* @see		lockImage
	*/
	void unlockImage();

	/**
	* Signal Robotino that you are ready for receiving images or tell him to stop transmitting images.
	* @param request	TRUE to start receiving images, FALSE to stop it.
	* @throws	nothing.
	* @see		lockImage, setRobotinoImageReceivedCallback
	*/
	void setImageRequest( bool request );

	/**
	* Retrieves the number of Robotinos motors.
	* @return	The number of motors.
	* @throws	nothing.
	* @see		getDriveLayout, actualVelocity
	*/
	unsigned int numMotors() const;

	/**
	* Set the velocity of the given motor.
	* This overrides any previous set/addVelocity calls for the given motor.
	* @param motor	The active motor, starting from 0.
	* @param rpm	The target speed in Rounds Per Minute
	* @throws	nothing.
	* @see		numMotors, actualVelocity
	*/
	void setVelocity( unsigned int motor, float rpm );

	/**
	* Sets the layout of the robots drive system in order to drive with mm/s. Default values are for Robotino.
	* @param rb		Distance from robot center to wheel center.
	* @param rw		Radius of the wheels.
	* @param fctrl	Frequency of control loop measuring the speed.
	* @param gear	Gear
	* @param mer	Motor encoder resolution.
	* @throws	nothing.
	* @see		getDriveLayout
	*/
	void setDriveLayout( double rb = 125.0f , double rw = 40.0, double fctrl = 900.0, double gear = 16, double mer = 2000 );

	/**
	* Retrieves the layout parameters of the robots drive system.
	* @param rb		Distance from robot center to wheel center.
	* @param rw		Radius of the wheels.
	* @param fctrl	Frequency of control loop measuring the speed.
	* @param gear	Gear
	* @param mer	Motor encoder resolution.
	* @throws	nothing.
	* @see		setDriveLayout
	*/
	void getDriveLayout( double* rb, double* rw, double* fctrl, double* gear, double* mer );

	/**
	* Set the robots velocity.
	* @param vx		Velocity in x direction in mm/s.
	* @param vy		Velocity in y direction in mm/s.
	* @param omega	Angular velocity in deg/s.
	* @throws	nothing.
	* @see		addVelocity, actualVelocity
	*/
	void setVelocity( double vx, double vy, double omega );

	/**
	* The set velocity of the motor is calculated as the mean value of the added velocities scaled by priority.
	* @param motor	The active motors, starting from 0.
	* @param rpm	The additive speed in Rounds Per Minute.
	* @param priority	The scaling factor.
	* @throws	nothing.
	* @see		setVelocity, numMotors, actualVelocity
	*/
	void addVelocity( unsigned int motor, float rpm, float priority );

	/**
	* Returns the actual velocity of the motor received by the last status message.
	* @param motor	The motor in question.
	* @return		The velocity of the motor in rounds per minute. If no status
	*				message has been received, or if there is no operational connection
	*				to Robotino, 0 is returned.
	* @throws	nothing.
	* @see		setVelocity, numMotors
	*/
	float actualVelocity( unsigned int motor );

	/**
	* Set the PID controllers proportional constant of the motor.
	* @param motor	The active motor, starting from 0.
	* @param value	The new proportional constant.
	* @throws	nothing.
	* @see		setKd, setKi, numMotors
	*/
	void setKp( unsigned int motor, float value );

	/**
	* Set the PID controllers differential constant of the motor.
	* @param motor	The active motor, starting from 0.
	* @param value	The new differential constant.
	* @throws	nothing.
	* @see		setKp, setKi, numMotors
	*/
	void setKd( unsigned int motor, float value );

	/**
	* Set the PID controllers integral constant of the motor.
	* @param motor	The active motor, starting from 0.
	* @param value	The new integral constant.
	* @throws	nothing.
	* @see		setKp, setKd, numMotors
	*/
	void setKi( unsigned int motor, float value );

	/**
	* Set point for power output
	* @param value set point value. Range -100 to 100.
	*/
	void setPowerOutput( float value );

	/**
	* The current delivered by the power output.
	* @param rawValue Stores the raw value as measured by an 10bit adc, so range is from 0 to 1024
	* @return Delivered current in A.
	*/
	float powerOutputCurrent( float* rawValue = NULL ) const;

	/**
	Read the external encoder input
	@param velocity Stores the actual velocity in ticks/s
	@param position Stores the actual position in ticks since power on or resetEncoderInputPosition( true )
	*/
	void encoderInput( int* velocity, int* position ) const;

	/**
	* @param reset Set the current position to zero if true. Does nothing otherwise.
	* @see resetPosition, position
	*/
	void resetEncoderInputPosition( bool reset );

	/**
	* The current delivered by power amplifier for the given motor.
	* @param motor The motor. Range [0;numMotors]
	* @param rawValue Stores the raw value as measured by an 10bit adc, so range is from 0 to 1024
	* @return Delivered current in A.
	*/
	float motorCurrent( unsigned int motor, float* rawValue = NULL ) const;


	/**
	* Returns the state of Robotinos bumper.
	* @return	TRUE if the bumper is active (i.e. has contact to something), FALSE otherwise.
	* @throws	nothing.
	*/
	bool bumper() const;

	/**
	* Returns the number of Robotinos distance sensors.
	* @return	The number of distance sensors.
	* @throws	nothing.
	* @see		distance
	*/
	unsigned int numDistanceSensors() const;

	/**
	* Returns the reading of distance sensors.
	* @param n	The number of the sensor. Range [1;numDistanceSensors].
	* @return	The sensor reading in Volts.
	* @see		numDistanceSensors
	*/
	float distance( unsigned int n ) const;

	/**
	* Returns the raw reading of analog inputs.
	* @param name	The name of the sensor.
	* @return		The direct output of the given sensor.
	* @throws	nothing.
	* @see		distance, analogInput
	*/
	unsigned int adc( std::string name ) const;

	/**
	* Returns the reading from an external analog input.
	* @param n	The number of the analog input. Range [0;numADC-1]
	* @return	The reading of the input in volts.
	* @throws	nothing.
	* @see		numADC
	*/
	float analogInput( unsigned int n ) const;

	/**
	* Sets the value of a specified digital output.
	* @param name	The name of the desired output.
	* @param value	The output value of the digital output.
	*				Low if value is FALSE, high otherwise.
	* @throws	nothing.
	* @see		numDigitalOutput
	*/
	void setDigitalOutput( std::string name, bool value );

	/**
	* Set the value of a specified digital output.
	* @param n		The number of the output. Range [0;numDigitalOutput-1]
	* @param value	The output value. Low if value is FALSE, high otherwise.
	* @throws	nothing.
	* @see		numDigitalOutput
	*/
	void setDigitalOutput( unsigned int n, bool value );

	/**
	* Retrieves the value of the referenced digital input.
	* @param name	The name of the desired Input.
	* @return		TRUE if the input value is high, FALSE otherwise.
	* @throws	nothing.
	* @see		numDigitalInput
	*/
	bool digitalInput( std::string name ) const;

	/**
	* Retrieves the value of the referenced digital input.
	* @param n	The number of the input. Range [0;numDigitalInput-1]
	* @return	TRUE if the value is high, FALSE otherwise.
	* @throws	nothing.
	* @see		numDigitalInput
	*/
	bool digitalInput( unsigned int n ) const;

	/**
	* Returns the number of analog digital converters.
	* @return	The number of ADCs.
	* @throws	nothing.
	* @see		adc, analogInput
	*/
	unsigned int numADC() const;

	/**
	* Returns the number of digital inputs.
	* @return	The number of digital inputs.
	* @throws	nothing.
	* @see		digitalInput
	*/
	unsigned int numDigitalInput() const;

	/**
	* Returns the number of digital outputs.
	* @return	The number of digital Outputs.
	* @throws	nothing.
	* @see		setDigitalOutput
	*/
	unsigned int numDigitalOutput() const;

	/**
	@return The number of relays.
	@see setRelay
	*/
	unsigned int numRelays() const;

	/**
	@param relay The relay to switch. Range [0;numRelays-1]
	@param close If true, the relay is closed. Otherwise the relay is opened.
	@see numRelays
	*/
	void setRelay( unsigned int relay, bool close );

	/**
	* Returns the serial line update frequency.
	* @return	The serial line frequency.
	* @throws	nothing.
	*/
	unsigned int serialLineFrequency() const;

	/**
	* Shutdown Robotino. This will only work while a connection is established.
	* @throws	nothing.
	* @see		isConnected
	*/
	void setShutdown();

	/**
	* Set the camera parameters like resolution and compression.
	* @param param	The camera parameter set including resolution and compression.
	* @throws	nothing.
	* @see		RobotinoCameraParameters, isConnected
	*/
	void setCameraParameters( const RobotinoCameraParameters& param );

	/**
	* Returns the camera parameters.
	* @return	The current camera parameters, including resolution and compression level.
	* @throws	nothing.
	* @see		setCameraParameters, RobotinoCameraParameters
	*/
	RobotinoCameraParameters cameraParameters() const;

	/**
	* Returns the current communication state.
	* @return	The current communication state.
	* @throws	nothing.
	* @see		RobotinoComState, connectToHost, close, isConnected
	*/
	RobotinoComState state() const;

	/**
	* Save an image to disk. If no file extension is given, the extension is appended automatically
	* according to the value of image->parameters.type.
	*
	* @param image		The image, that should be saved.
	* @param fileName	The target filename of the image file.
	* @return			TRUE on success, FALSE otherwise.
	* @throws			nothing.
	* @see				RobotinoImage, cameraParameters, lockImage
	*/
	bool saveImageToFile( const RobotinoImage* image, const std::string& fileName );

	/**
	* Retrieves the current motor time of the specified motor.
	* @param motor	The number of the desired motor.
	* @return		The motor time of the given motor or 0 if the motor number is invalid.
	* @throws		nothing.
	* @see			resetMotorTime
	*/
	float motorTime( unsigned int motor ) const;

	/**
	* @param motor The number of the desired motor.
	* @param reset Set the current position of the motor to zero if true. Does nothing otherwise.
	* @see position
	*/
	void resetPosition( unsigned int motor, bool reset );

	/**
	* @param motor The number of the desired motor.
	* @return The position of the motor estimated by counting the ticks generated by the motor's encoder.
	* @see resetPosition
	*/
	int position( unsigned int motor ) const;

	/**
	* Resets the motor time of the given motor.
	* @param motor	The number of the motor.
	* @param reset	Set to TRUE to reset the specified motor timer.
	* @throws	nothing.
	* @see		motorTime
	*/
	void resetMotorTime( unsigned int motor, bool reset );

	/**
	* Controls the brake of the specified motor.
	* @param motor	The number of the desired motor.
	* @param on		TRUE to activate the brake, FALSE to unlock it.
	* @throws	nothing.
	*/
	void setBrake( unsigned int motor, bool on );

	/**
	* Milliseconds since the RobotinoCom object is created. The images' timestamp is given by this function.
	* @return	The elapsed time since object creation in milliseconds.
	* @throws	nothing.
	* @see		saveImageToFile
	*/
	float msecsElapsed() const;

	/**
	* Retrieves the current readings of the NorthStar tracking device.
	* @return	An object containing all current values concerning the NorthStar tracking.
	* @throws	nothing.
	* @see		NorthStarReadings, setNorthStarCommand
	*/
	NorthStarReadings northStarReadings() const;

	/**
	* Gives a command to the NorthStar tracking device.
	* @param northStarCommand	The command set for the NorthStar device.
	* @throws	nothing.
	* @see		NorthStarCommand
	*/
	void setNorthStarCommand( const NorthStarCommand& northStarCommand );

	/**
	* Retrieves the current power drain.
	* @return	The current drain in mA.
	* @throws	nothing.
	* @see		voltageBatt1, voltageBatt1plus2
	*/
	float current() const;

	/**
	* Retrieves the voltage of the first battery.
	* @return	The remaining voltage in V.
	* @throws	nothing.
	* @see		voltageBatt1plus2, current
	*/
	float voltageBatt1() const;

	/**
	* Retrieves the voltage of both batteries.
	* @return	The remaining voltage of both batteries.
	* @throws	nothing.
	* @see		voltageBatt1, current
	*/
	float voltageBatt1plus2() const;

	/**
	* Retrieves informational messages from Robotino.
	* @return	An object containing messages and additional information.
	* @throws	nothing.
	* @see		RobotinoInfo
	*/
	RobotinoInfo robotinoInfo() const;

	/**
	Enable/Disable the gripper attached to Robotino's power output
	@param enable If true the gripper is enabled, otherwise the gripper is disabled
	*/
	void setGripperEnabled( bool enable );

	/**
	@return Returns true, if the gripper was enabled by a previous call to setGripperEnabled( true ). Otherwise returns false.
	@see setGripperEnabled
	*/
	bool isGripperEnabled() const;

	/**
	Open/Close the gripper attached to Robotino's power output
	@param close If true the gripper is closed, otherwise the gripper is opened.
	*/
	void setGripperClosed( bool close );

	/**
	@return Returns true if the gripper is opened.
	*/
	bool isGripperOpened() const;

	/**
	@return Returns true if the gripper is closed.
	*/
	bool isGripperClosed() const;

	/**
	Set the odometry to the given position.
	@param x x position in mm
	@param y y position in mm
	@param phi orientation in degree
	*/
	void setOdometry( float x, float y, float phi );

	/**
	Get the current position estimated by the odometry.
	@param x x position in mm
	@param y y position in mm
	@param phi orientation in degree
	*/
	void getOdometry( float* x, float* y, float* phi );


	/**
	* Returns the current RobotinoCom version.
	* @return	The current version.
	* @throws	nothing.
	*/
	static unsigned int version();

private:
	RobotinoComImpl* _impl;
};

#endif

/**  \mainpage robotinocom API documentation

<B>This API is depreceted. Please use the <A HREF="../rec_robotino_com/index.html">rec::robotino::com</A> API instead.</B>

The application programming interface (API) for Robotino(r) from Festo Didactic permits full
access to Robotino's sensors and actors. Communication between the control program and Robotino
is handled via TCP and UDP and is therefor fully network transparent. It does not matter whether the
control program runs direcly on Robotino or on a remote system.

The API is available in binary form for Windows and Linux and <A HREF="http://svn.servicerobotics.eu/trunk">source code</A>
via svn.

\section install Installation
Install the API either from binary or from source. On a german Windows the default install path is
C:\\Programme\\REC GmbH\\OpenRobotinoAPI\\1. On Linux the API is installed in /usr/local/OpenRobotinoAPI/1. In the following this path is refered to as "install".
Please replace "Programme" with your systems program folder name.

To build your own program you need to
-# include install/include in your compilers include search path
-# use \#include "robotinocom/robotinocom.h" in your program to use RobotinoCom
-# include install/lib/win32 or install/lib/linux to your linkers library search path
-# link against robotinocom.lib on win32 and librobotinocom.so on linux systems

If you are familiry with <A HREF="http://www.cmake.org">cmake</A> you might prefer using install/tools/FindOpenRobotino1.cmake.

\section usage_sec Usage
In the following you will find a simple example on how to drive Robotino in x direction with 100mm/s for 10 seconds.

\subsection includes Including headers
You need to include at least "robotinocom/robotinocom.h". "xutils.h" defines mlseep and <iostream> is needed for printing status information.
<PRE>
#include "robotinocom/robotinocom.h"

#include "xutils.h"

#include <iostream>
</PRE>

\subsection handlers Callback functions
You might install a couple of callback functions. This is optional but helps you to see what is going on.
<PRE>
void errorCb( void* data, int error )
{
  std::cout << std::endl <<  "Error " << RobotinoCom::errorString( error ) << std::endl;
}

void connectedCb( void* data )
{
  std::cout << std::endl << "Connected" << std::endl;
}

void connectionClosedCb( void* data )
{
  std::cout << std::endl << "Connection closed." << std::endl;
}
</PRE>

\subsection main The main program
Within the main program the following steps are neccessary to drive Robotino:
-# Initialise RobotinoCom
-# Install callback handlers (optional)
-# Setup the network connection
-# Continuously send set point values to Robotino
-# Close the connection

\subsubsection init Initialise RobotinoCom and install callbacks handlers
Robotino's ip address is defined. If not given as the first command line argument we use
172.26.1.1 which is Robotino's default address. If you are running this program on Robotino
you might use localhost as address to be independent from any changes to Robotino's network settings.
<PRE>
int main(int argc, char **argv)
{
  std::string hostname = "172.26.1.1";
  if( argc > 1 )
  {
    hostname = argv[1];
  }

  RobotinoCom com;
  unsigned int startTime;

  if( false == com.init() )
  {
    return 1;
  }

  com.setErrorCallback( &errorCb, NULL );
  com.setConnectedCallback( &connectedCb, NULL );
  com.setConnectionClosedCallback( &connectionClosedCb, NULL );
</PRE>

\subsubsection connect Setup the network connection
The connection to Robotino is established by connectToHost. This function is non blocking
so we check the status of the connection as long as we are in the ConnectingState. This state is left by either
the connection is successfully established or by an error.
<PRE>
  std::cout << "Connecting to host " << hostname;
  com.connectToHost( hostname );

  while( com.state() == ConnectingState )
  {
    std::cout << ".";
    msleep( 200 );
  }
  std::cout << std::endl;

  if( com.error() != NoError )
  {
    goto CLEANUP;
  }
</PRE>
  
\subsubsection setpoint Continuously send set point values to Robotino
Now we are ready to send set point values for Robotino's motors. This is done by
setVelocity. The data is send by a call to update. This function not only sends
all set point values to Robotino but also receives the status of Robotino's sensors.
<PRE>
  startTime = com.msecsElapsed();

  com.stopAsyncCommunication();
  while( false == com.bumper() )
  {
    unsigned int msecsElapsed = com.msecsElapsed() - startTime;

    if( msecsElapsed < 10000 )
    {
      //drive forward with 100mm/s
      com.setVelocity( 100, //vx=100 mm/s
                         0, //vy=0 mm/s
                         0  //omega=0 deg/s
                         );
    }
    else
    {
      break;
    }
    //sending the set values to Robotino and receiving the sensor readings
    if( com.update() == false )
    {
      break;
    }
  }
</PRE>

\subsubsection close Close the connection
After 10s or if update failed the connection is closed and the program exits.
<PRE>
CLEANUP:
  com.close();
  
  return 0;
}
</PRE>
*/
