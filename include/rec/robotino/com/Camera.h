//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_COM_CAMERA_H_
#define _REC_ROBOTINO_COM_CAMERA_H_

#include "rec/robotino/com/Actor.h"

namespace rec
{
	namespace robotino
	{     
		namespace com
		{
			/**
			* @brief	Represents a camera.
			*/
			class
#ifdef WIN32
#  ifdef rec_robotino_com_EXPORTS
		__declspec(dllexport)
#endif
#  ifdef rec_robotino_com2_EXPORTS
		__declspec(dllexport)
#endif
#endif
			Camera : public Actor
			{
			public:
				Camera();
				~Camera();

				typedef enum { UNKNOWN_RESOLUTION, QVGA, VGA } ImageResoluton_t;

				/**
				* Type of data received
				*/
				typedef enum { UNKNOWN_TYPE, JPEG, RAW, BMP } ImageType_t;

				/** 
				* Sets the associated communication object.
				*
				* @param id	The id of the associated communication object.
				* @throws	RobotinoException If the given communication object doesn't provide a camera.
				*/
				void setComId( const ComId& id );

				/**
				* Turns the streaming on and off.
				*
				* @param streaming	TRUE to turn streaming on.
				* @throws	RobotinoException If an error occured.
				*/
				void setStreaming( bool streaming );

				/** 
				* Checks if streaming is enabled.
				*
				* @return	TRUE if streaming is enabled.
				* @throws	nothing.
				*/ 
				bool isStreaming() const;

				/**
				* Sets this camera to the given resolution (if possible)
				*
				* @param resolution	The target resolution
				* @throws	RobotinoException if given com object is invalid.
				*/
				void setResolution( ImageResoluton_t resolution );

				/**
				* Returns the current image resolution, this camera delivers.
				*
				* @return	The current camera resolution
				* @throws	RobotinoException if given com object is invalid.
				*/
				ImageResoluton_t resolution() const;

				/**
				* Called when an image is received.
				* Note: This function is called from outside the applications main thread.
				* This is extremely important particularly with regard to GUI applications.

				@param imageData Contains the image data.
				@param type The type of the image data.
				* @throws		nothing.
				*/
				virtual void imageReceivedEvent( const unsigned char* data, unsigned int dataSize, ImageType_t type );
			};
		}
	}
}

#endif
