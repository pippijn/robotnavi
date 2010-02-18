//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_LABVIEW_DISTANCESENSOR_H_
#define _REC_ROBOTINO_LABVIEW_DISTANCESENSOR_H_

#include "rec/robotino/labview_c/globals.h"

DLLEXPORT BOOL distanceSensor_labview( float* voltage, int* heading, int comID, int number );

DLLEXPORT BOOL distanceSensor_voltage( float* voltage, int comID, int number );

DLLEXPORT BOOL distanceSensor_heading( int* heading, int number );

#endif
