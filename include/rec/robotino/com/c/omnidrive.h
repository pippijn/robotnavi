//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_COM_C_OMNIDRIVE_H_
#define _REC_ROBOTINO_COM_C_OMNIDRIVE_H_

#include "rec/robotino/com/c/globals.h"
#include "rec/robotino/com/c/com.h"

/** \file omnidrive.h
    \brief In "rec/robotino/com/c/omnidrive.h" you can find functions for manipulating Robotino's omnidrive.

		Use omnidrive_construct() to create a new omnidrive object. Associate the omnidrive object with a com object using omnidrive_setComId().
		Use omnidrive_setVelocity() to drive Robotino.
*/

typedef int OmniDriveId;

#define INVALID_OMNIDRIVEID -1

/**
Construct an OmniDrive object
@return Returns the ID of the newly constructed OmniDrive object.
*/
DLLEXPORT OmniDriveId omnidrive_construct();

/**
Destroy the OmniDrive object assígned to id
@param id The id of the OmniDrive object to be destroyed
@return Returns TRUE (1) on success. Returns FALSE (0) if the given OmniDriveId is invalid.
*/
DLLEXPORT BOOL omnidrive_destroy( OmniDriveId id );

/**
Associated an OmniDrive object with a communication interface, i.e. binding the OmniDrive to a specific Robotino
@return Returns TRUE (1) on success. Returns FALSE (0) if the given OmniDriveId or ComId is invalid.
*/
DLLEXPORT BOOL omnidrive_setComId( OmniDriveId id, ComId comId );

/**
Drive Robotino associated with id
@param vx		Velocity in x-direction in mm/s
@param vy		Velocity in y-direction in mm/s
@param omega	Angular velocity in deg/s
@return Returns the ID of the newly constructed communication interface.
*/
DLLEXPORT BOOL omnidrive_setVelocity( OmniDriveId id, float vx, float vy, float omega );

#endif //_REC_ROBOTINO_COM_C_OMNIDRIVE_H_
