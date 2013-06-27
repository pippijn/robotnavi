//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_LABVIEW_NORTHSTAR_H_
#define _REC_ROBOTINO_LABVIEW_NORTHSTAR_H_

#include "rec/robotino/labview_c/globals.h"

DLLEXPORT BOOL northstar_labview(
																 float* x,
																 float* y,
																 float* phi,
																 int* spots,
																 int* spot0,
																 int* spot1,
																 int* currentRoom,
																 int* timestamp,
																 
																 int comID,
																 int roomNumber,
																 BOOL setOrigin );

DLLEXPORT BOOL northstar_setRoomNumber( int comID, int roomNumber );
DLLEXPORT BOOL northstar_getRoomNumber( int* roomNumber, int comID );
DLLEXPORT BOOL northstar_x( int* x, int comID );
DLLEXPORT BOOL northstar_y( int* y, int comID );
DLLEXPORT BOOL northstar_phi( float* phi, int comID );
DLLEXPORT BOOL northstar_numSpots( int* numSpots, int comID );
DLLEXPORT BOOL northstar_spot0( int* value, int comID );
DLLEXPORT BOOL northstar_spot1( int* value, int comID );

#endif
