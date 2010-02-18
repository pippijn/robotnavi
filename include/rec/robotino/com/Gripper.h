//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_COM_GRIPPER_H_
#define _REC_ROBOTINO_COM_GRIPPER_H_

#include "rec/robotino/com/Actor.h"

namespace rec
{
	namespace robotino
	{
		namespace com
		{
			/**
			* @brief	Represents a digital output device.
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
			Gripper : public Actor
			{
			public:
				Gripper();

				~Gripper();

				/**
				By assigning a Com object to the gripper, any PowerOutput assigend to this Com object is disabled.
				The PowerOutput is enabled if all Gripper objects are removed from this Com object.
				@see Actor::setComId
				*/
				void setComId( const ComId& id );

				/**
				Open gripper.
				*/
				void open();

				/**
				Close gripper.
				*/
				void close();

				/**
				@return Returns true if gripper is opened. False otherwise.
				@see isClosed
				*/
				bool isOpened() const;

				/**
				@return Returns true if gripper is closed. False otherwise.
				@see isOpened
				*/
				bool isClosed() const;
			};
		}
	}
}
#endif //_REC_ROBOTINO_COM_GRIPPER_H_


