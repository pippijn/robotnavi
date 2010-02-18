//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_LABVIEW_MOTOR_H_
#define _REC_ROBOTINO_LABVIEW_MOTOR_H_

#include "rec/robotino/labview_c/globals.h"

DLLEXPORT BOOL motor_labview(
														 float* actualSpeed,
														 float* actualPos,
														 float* current,
														 
														 int comID,
														 float targetSpeed,
														 BOOL brake,
														 BOOL resetPosition,
														 float kp,
														 float ki,
														 float kd,
														 int motorNumber );

DLLEXPORT BOOL motor_setSetPointSpeed( int comID, float speed, int number );
DLLEXPORT BOOL motor_resetPosition( int comID, int number );
DLLEXPORT BOOL motor_setBrake( int comID, BOOL brake, int number );
DLLEXPORT BOOL motor_setKp( int comID, float kp, int number );
DLLEXPORT BOOL motor_setKi( int comID, float ki, int number );
DLLEXPORT BOOL motor_setKd( int comID, float kd, int number );

DLLEXPORT BOOL motor_actualSpeed( float* speed, int comID, int number );
DLLEXPORT BOOL motor_actualPosition( float* pos, int comID, int number );
DLLEXPORT BOOL motor_current( float* current, int comID, int number );

#endif
