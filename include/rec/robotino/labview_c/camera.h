//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_LABVIEW_CAMERA_H_
#define _REC_ROBOTINO_LABVIEW_CAMERA_H_

#include "rec/robotino/labview_c/globals.h"

DLLEXPORT BOOL camera_labview( char* destImage, unsigned int* destImageSize, int comId, unsigned int destImageMaxSize, int resolution );

DLLEXPORT BOOL camera_setResolution( int comID, int resolution /* 0 = QVGA, 1 = VGA */ );

DLLEXPORT BOOL camera_imageAvailable( BOOL* isAvailable, int comID );

DLLEXPORT BOOL camera_imageSize( unsigned int* imageSize, int comID );

DLLEXPORT BOOL camera_copyImage( char* destImage, int comID, unsigned int maxBufferSize );

#endif
