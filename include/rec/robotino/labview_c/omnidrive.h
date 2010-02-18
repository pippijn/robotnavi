//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_LABVIEW_OMNIRIVE_H_
#define _REC_ROBOTINO_LABVIEW_OMNIRIVE_H_

#include "rec/robotino/labview_c/globals.h"

DLLEXPORT BOOL omnidrive_labview( float* m1, float* m2, float* m3, int comID, float vx_set, float vy_set, float omega_set );

DLLEXPORT BOOL omnidrive_set( int comID, float vx, float vy, float omega );
DLLEXPORT BOOL omnidrive_m1( float* value, int comID );
DLLEXPORT BOOL omnidrive_m2( float* value, int comID );
DLLEXPORT BOOL omnidrive_m3( float* value, int comID );

#endif
