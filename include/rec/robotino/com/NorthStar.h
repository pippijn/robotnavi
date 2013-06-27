//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_COM_NORTHSTAR_H_
#define _REC_ROBOTINO_COM_NORTHSTAR_H_

#include "rec/robotino/com/Actor.h"
#include "rec/robotino/com/northstar_interface.h"

namespace rec
{
	namespace robotino
	{
		namespace com
		{
			/**
			* @brief	Represents the NorthStar tracking device.
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
			NorthStar : public Actor
			{
			public:
				NorthStar();

				/**
				* Retrieves the current readings of the NorthStar tracking device.
				* @return	An object containing all current values concerning the NorthStar tracking.
				* @throws	RobotinoException if the underlying communication object is invalid
				* @see		NorthStarReadings
				*/
				NorthStarReadings northStarReadings() const;

				/**
				* Gives a command to the NorthStar tracking device.
				* @param northStarCommand	The command set for the NorthStar device.
				* @throws	RobotinoException if the underlying communication object is invalid
				* @see		NorthStarCommand
				*/
				void setNorthStarCommand( const NorthStarCommand& northStarCommand );
			};
		}
	}
}
#endif
