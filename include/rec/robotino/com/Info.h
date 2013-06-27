//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_COM_INFO_H_
#define _REC_ROBOTINO_COM_INFO_H_

#include "rec/robotino/com/Actor.h"

#include <string>

namespace rec
{
	namespace robotino
	{
		namespace com
		{
			class InfoImpl;

			/**
			* @brief	Retrieves informational messages from Robotino.
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
			Info : public Actor
			{
			public:
				Info();

				virtual ~Info();

				/** 
				* Sets the associated communication object.
				*
				* @param id	The id of the associated communication object.
				* @throws	RobotinoException If the given communication object doesn't exist.
				*/
				void setComId( const ComId& id );

				/**
				* Gets the current text message of this device.
				*
				* @return	The current text message.
				* @throws	RobotinoException if the current communication object is invalid.
				*/
				std::string text() const;

				/**
				* Returns wether this device is in passive mode.
				*
				* @return	TRUE if in passive mode, FALSE otherwise.
				* @throws	RobotinoException if the current communication object is invalid.
				*/
				bool isPassiveMode() const;

				/**
				* Called on info receive.
				* Note: This function is called from outside the applications main thread.
				* This is extremely important particularly with regard to GUI applications.

				* @param text The info messages text
				* @throws		nothing.
				*/
				virtual void infoReceivedEvent( const std::string& text );
			};
		}
	}
}
#endif
