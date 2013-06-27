//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_LABVIEW_COM_H_
#define _REC_ROBOTINO_LABVIEW_COM_H_

#include "rec/robotino/labview_c/globals.h"

DLLEXPORT BOOL com_labview(
														// return values
														int* id,
														BOOL* isConnected,
														int* state,
														int* errorCode,

														// input values
														int comId, //wird nicht durch den Benutzer, sondern intern durch das VI gesetzt (Zahl zwischen 0 und einschließlich 15)
																		//wenn mehr als 16 Com Vis erzeugt werden, soll ab dem 17 VI -1 übergeben werden

														BOOL connect, //on true: construct com object and establish connection to robotinoAddress::robotinoPort
																					//on false: disconnect and destroy com object

														const char* robotinoAddress,
														int robotinoPort,
														int imageServerPort //accept image at UDP port imageServerPort
														);

DLLEXPORT int com_construct();

DLLEXPORT BOOL com_reset( int id );

DLLEXPORT BOOL com_setAddress( int id, const char* address, int port );

DLLEXPORT BOOL com_setImageServerPort( int id, unsigned int port );

DLLEXPORT BOOL com_connect( int id );

DLLEXPORT BOOL com_isConnected( int id );

DLLEXPORT BOOL com_destroy( int id );

DLLEXPORT BOOL com_update( int id );

DLLEXPORT BOOL com_setAutoupdate( int id, BOOL on, int period );

DLLEXPORT BOOL com_error( int* errorCode, int id );

DLLEXPORT BOOL com_errorString( char* errorString, int bufferLength, int id );

#endif
