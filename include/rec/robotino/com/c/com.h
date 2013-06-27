//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_COM_C_COM_H_
#define _REC_ROBOTINO_COM_C_COM_H_

#include "rec/robotino/com/c/globals.h"

/** \file com.h
    \brief In "rec/robotino/com/c/com.h" you can find functions for manipulating the communication interface to Robotino.
  
		Use com_construct() to create a new com object. Set the IP-address of Robotino with com_setAddress(). com_connect() establishes the connection.

<PRE>
#include "rec/robotino/com/c/com.h"

ComId com;

int main( int argc, char **argv )
{
  com = com_construct();

  if( argc > 1 )
  {
    com_setAddress( com, argv[1] );
  }
  else
  {
    com_setAddress( com, "172.26.1.1" );
  }

  if( FALSE == com_connect( com ) )
  {
    exit( 1 );
  }
  else
  {
    char addressBuffer[256];
    com_address( com, addressBuffer, 256 );
    printf( "Connected to %s\n", addressBuffer );
  }

  com_destroy( com );

  return 0;
}
</PRE>  
*/

typedef int ComId;

#define INVALID_COMID -1

/**
Construct an interface for communicating to one Robotino
@return Returns the ID of the newly constructed communication interface.
*/
DLLEXPORT ComId com_construct();

/**
Destroy the communication interface assígned to id
@param id The id of the communication interface to be destroyed
@return Returns TRUE (1) on success. Returns FALSE (0) if the given ComId is invalid.
*/
DLLEXPORT BOOL com_destroy( ComId id );


/**
@param id The ComId returned by com_construct().
@param address A null terminated string containing the (IP)-address (plus port) of Robotino.
The default to connect to Robotino is 172.26.1.1 (port can be omitted.
To connect to RobotinoSim running at localhost use 127.0.0.1:8080 (or higher ports).
@return Returns TRUE (1) on success. Returns FALSE (0) if the given ComId is invalid.
*/
DLLEXPORT BOOL com_setAddress( ComId id, const char* address );

/**
@param id The ComId returned by com_construct().
@param addressBuffer Will contain the currently active server address set with com_setAddress().
@param addressBuffersSize The size of addressBuffer.
@return Returns TRUE (1) on success. Returns FALSE (0) if the given ComId is invalid.
@see com_setAddress
*/
DLLEXPORT BOOL com_address( ComId id, char* addressBuffer, unsigned int addressBuffersSize );

/**
@param id The ComId returned by com_construct().
@param port The image server port. To be able to receive images from Robotino there is a UDP server
running on your machine. The default port is 8080, but you might change this.
@return Returns TRUE (1) on success. Returns FALSE (0) if the given ComId is invalid.
*/
DLLEXPORT BOOL com_setImageServerPort( ComId id, int port );

/**
Establish the communication. Call com_setAddress() first.
@param id The ComId returned by com_construct().
@return Returns TRUE (1) on success. Returns FALSE (0) if the given ComId is invalid.
*/
DLLEXPORT BOOL com_connect( ComId id );

/**
Stop communication.
@param id The ComId returned by com_construct().
@return Returns TRUE (1) on success. Returns FALSE (0) if the given ComId is invalid.
*/
DLLEXPORT BOOL com_disconnect( ComId id );

/**
Check if the communication is established.
@param id The ComId returned by com_construct().
@return Returns TRUE (1) if the communication is active. Returns FALSE (0) if the communication is inactive or if the given ComId is invalid.
*/
DLLEXPORT BOOL com_isConnected( ComId id );

//to include all headers at this point is not nice but necessary for using this lib in Matlab
//on the other hand you only need to include com.h to get it all
#include "rec/robotino/com/c/bumper.h"
#include "rec/robotino/com/c/camera.h"
#include "rec/robotino/com/c/omnidrive.h"

#endif
