//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_COM_C_BUMPER_H_
#define _REC_ROBOTINO_COM_C_BUMPER_H_

#include "rec/robotino/com/c/globals.h"
#include "rec/robotino/com/c/com.h"

/** \file bumper.h
    \brief In "rec/robotino/com/c/bumper.h" you can find functions for reading Robotino's bumper.

		Use bumper_construct() to create a new bumper object. Associate the bumper object with a com object using bumper_setComId().
		Use bumper_value() to read the bumper's state.
*/

typedef int BumperId;

#define INVALID_BUMPERID -1

/**
Construct an Bumper object
@return Returns the ID of the newly constructed Bumper object.
*/
DLLEXPORT BumperId bumper_construct();

/**
Destroy the Bumper object assígned to id
@param id The id of the Bumper object to be destroyed
@return Returns TRUE (1) on success. Returns FALSE (0) if the given BumperId is invalid.
*/
DLLEXPORT BOOL bumper_destroy( BumperId id );

/**
Associated a Bumper object with a communication interface, i.e. binding the Bumper to a specific Robotino
@return Returns TRUE (1) on success. Returns FALSE (0) if the given BumperId or ComId is invalid.
*/
DLLEXPORT BOOL bumper_setComId( BumperId id, ComId comId );

/**
Get the current state of the bumper.
@param id	The bumper id.
@return Returns TRUE (1) if the bumper is pressed. Returns FALSE (0) otherwise.
*/
DLLEXPORT BOOL bumper_value( BumperId id );

#endif //_REC_ROBOTINO_COM_C_BUMPER_H_
