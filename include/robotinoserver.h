//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _ROBOTINOSERVER_H_
#define _ROBOTINOSERVER_H_

#include <string>
#include <vector>

#include "robotinocom/robotinocom.h"
#include "rec/robotino/com/northstar_interface.h"
#include "robotinocom/robotinoinfo.h"

using rec::robotino::com::NorthStarCommand;
using rec::robotino::com::NorthStarReadings;

#ifdef WIN32
#ifdef robotinoserver_EXPORTS
  #define ROBOTINOSERVER_EXPORT __declspec(dllexport)
#else
  #define ROBOTINOSERVER_EXPORT //__declspec(dllimport)
#endif
#else
#define ROBOTINOSERVER_EXPORT
#endif

#define ROBOTINOSERVER_PORT 80

class RobotinoServerImpl;

class ROBOTINOSERVER_EXPORT RobotinoServer
{
public:
  typedef enum {ErrorServerCreate, ErrorServerBind, ErrorDgramServiceCreate, ErrorServerListen,
    ErrorHeaderReceive, ErrorInvalidCommandID, ErrorHeaderLengthToBig, ErrorDataReceive, ErrorReceiveTimeout,
    ErrorMessageSend,ErrorWinsock2Initialization} Error;

  typedef void(*ioControlUpdatedCb_t)( RobotinoServer* const server, void* data );
  typedef void(*cameraControlUpdatedCb_t)( RobotinoServer* const server, void* data );
  typedef void(*imageRequestCb_t)( RobotinoServer* const server, bool request, void* data );
  typedef void(*errorCb_t)( RobotinoServer::Error error, void* data );
  typedef void(*connectCb_t)( void* data );
  typedef void(*disconnectCb_t)( void* data );
  
  RobotinoServer();
  ~RobotinoServer();

  /**Returns the english description of the error.*/
  static std::string errorString( Error error );

  /**Initialize the server. The initWinsock2 parameter has no effect on platforms other than Windows.*/
  bool init( bool initWinsock2 = true );

  /**Start the server in an individual thread.*/
  void start();

  /**Run the server within the calling thread. This call is blocking.*/
  void run();

  /**Closes the server. Waits for run() to finish.*/
  void close();

  /**Close all active connections.*/
  void closeConnections();

  void setIoControlUpdatedCb( ioControlUpdatedCb_t, void* data );

  void setCameraControlUpdatedCb( cameraControlUpdatedCb_t, void* data );

  void setImageRequestCb( imageRequestCb_t, void* data );
  
  void setErrorCallback( errorCb_t, void* data );

  void setConnectCallback( connectCb_t, void* data );

  void setDisconnectCallback( disconnectCb_t, void* data );


  void setImage( const RobotinoImage* image );

  /**Copy the p2q buffer to buffer. Buffer must be an array of size p2qSize().*/
  void get_p2q( unsigned char* buffer );

  /**Copy data to the q2p buffer. Data must be an array of size q2pSize().*/
  void set_q2p( const unsigned char* data );

  /**Returns the set velocity of motor in rpm. The motors are counted starting with 1.*/
  float velocity( unsigned int motor );

  /**Set the actual velocity of motor.*/
  void setVelocity( unsigned int motor, float rpm );

  void setPosition( unsigned int motor, int ticks );
  void setMotorTime( unsigned int motor, unsigned int time );

  bool isResetPosition( unsigned int motor );
  bool isResetMotorTime( unsigned int motor );

  /**Set the distance in mm measured by sensor. The sensors are counted starting with 1.*/
  void setDistance( unsigned int sensor, unsigned int distance );

  /**Returns true if Robotino is expected to shutdown.*/
  bool isShutdown();

  /**Returns the current camera parameters.*/
  RobotinoCameraParameters cameraParameters();

  /**Returns the set state of digital output number n.*/
  bool digitalOutput( unsigned int n );
  
  /**Set the update frequency of the serial line.*/
  void setSerdFrequency( unsigned int freq );

  void setBumper( const bool on );

  unsigned char getKp( const unsigned int motor );
  unsigned char getKi( const unsigned int motor );
  
  void setNorthStarReadings( const NorthStarReadings& readings );

  NorthStarCommand northStarCommand() const;

	/**
	Read the position set by RobotinoCom::setOdoemtry at the remote site
	@param isSet True if RobotinoCom::setOdoemtry was called at the remote site, false otherwise
	*/
	void getOdometry( bool* isSet, float* x, float* y, float* phi );

	/**
	Set the current position which is read at the remote site by RobotinoCom::getOdometry
	*/
	void setOdometry( float x, float y, float phi );

  void encodeIoStatus();

  void setRobotinoInfo( const RobotinoInfo& info );

  unsigned int getFirmwareVersion() const;
  
  static unsigned int version();

	void setImagePartSize( unsigned int numBytesPerPart );

	//The default port is 80. Use this function before calling init
	void setServerPort( unsigned int port );
	unsigned int serverPort() const;

	void reset();

	static void serverPortRange( unsigned int* minPort, unsigned int* maxPort );
	static void setServerPortRange( unsigned int minPort, unsigned int maxPort );

	static std::vector<unsigned int> serverPortsInUse();

private:
  RobotinoServerImpl* _impl;
};

#endif
