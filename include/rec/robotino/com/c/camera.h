//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_COM_C_CAMERA_H_
#define _REC_ROBOTINO_COM_C_CAMERA_H_

#include "rec/robotino/com/c/globals.h"
#include "rec/robotino/com/c/com.h"

/** \file camera.h
    \brief In "rec/robotino/com/c/camera.h" you can find functions for reading Robotino's bumper.

		Use camera_construct() to create a new camera object. Associate the camera object with a com object using camera_setComId().
		Use camera_setStreaming() to enable/disable streaming of images. Use camera_getImage() to get images from Robotino's camera.
*/

typedef int CameraId;

#define INVALID_CAMERAID -1

/**
Construct an Camera object
@return Returns the ID of the newly constructed Camera object.
*/
DLLEXPORT CameraId camera_construct();

/**
Destroy the Camera object ass�gned to id
@param id The id of the Camera object to be destroyed
@return Returns TRUE (1) on success. Returns FALSE (0) if the given CameraId is invalid.
*/
DLLEXPORT BOOL camera_destroy( CameraId id );

/**
Associated an Camera object with a communication interface, i.e. binding the Camera to a specific Robotino
@return Returns TRUE (1) on success. Returns FALSE (0) if the given CameraId or ComId is invalid.
*/
DLLEXPORT BOOL camera_setComId( CameraId id, ComId comId );

/**
Grab image.
@param id The camera id.
@return Returns TRUE (1) if a new image is available since the last call of camera_grab. Returns FALSE (0) otherwise.
*/
DLLEXPORT BOOL camera_grab( CameraId id );

/**
Size of image aquired by grab.
@param width Image width.
@param height Image height.
@return Returns TRUE (1) on success. Returns FALSE (0) if the given CameraId is invalid or if no image has been grabed up to this point in time.
*/
DLLEXPORT BOOL camera_imageSize( CameraId id, unsigned int* width, unsigned int* height );


/**
Get Robotino's camera image. Do not forget to call camera_setStreaming( id, TRUE )  and camera_grab first.
Get the size of the image by camera_imageSize first. imageBufferSize must be at least 3*width*height. The image
copied to image buffer is an interleaved RGB image width 3 channels and 1 byte per channel.
@param id The camera id.
@param imageBuffer The image is copied imageBuffer.
@param imageBufferSize The size of imageBuffer.
@param width Image width.
@param height Image height.
@return Returns TRUE (1) on success. Returns FALSE (0) if the given CameraId.
@see camera_setStreaming camera_imageSize
*/
DLLEXPORT BOOL camera_getImage(
															 CameraId id,
															 unsigned char* imageBuffer,
															 unsigned int imageBufferSize,
															 unsigned int* width,
															 unsigned int* height );


/**
Start/Stop streaming of camera images
@param id The camera id.
@param streaming If TRUE (1) streaming is started. Otherwise streaming is stopped.
@return Returns TRUE (1) on success. Returns FALSE (0) if the given CameraId.
*/
DLLEXPORT BOOL camera_setStreaming( CameraId id, BOOL streaming );

#endif //_REC_ROBOTINO_COM_C_CAMERA_H_
