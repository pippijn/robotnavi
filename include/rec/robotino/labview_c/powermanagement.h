//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH
#ifndef _REC_ROBOTINO_LABVIEW_POWERMANAGEMENT_H_
#define _REC_ROBOTINO_LABVIEW_POWERMANAGEMENT_H_

#include "rec/robotino/labview_c/globals.h"

DLLEXPORT BOOL power_labview( float* current, float* voltage, int comID );

DLLEXPORT BOOL power_current( float* current, int comID );

DLLEXPORT BOOL power_voltage( float* voltage, int comID );

#endif

