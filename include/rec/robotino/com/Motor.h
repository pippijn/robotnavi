//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_COM_MOTOR_H_
#define _REC_ROBOTINO_COM_MOTOR_H_

#include "rec/robotino/com/Actor.h"

namespace rec
{
	namespace robotino
	{
		namespace com
		{

			/**
			* @brief	Represents a single motor
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
			Motor : public Actor
			{
			public:
				Motor();

				/**
				* @return Returns the number of drive motors on Robotino
				* @throws nothing
				*/
				static unsigned int numMotors();

				/**
				* Sets the number of this motor.
				*
				* @param number	number of this motor
				* @throws	RobotinoException if the current communication object is invalid.
				*/
				void setMotorNumber( unsigned int number );

				/**
				* Sets the setpoint speed of this motor.
				*
				* @param speed	Set point speed in rpm.
				* @throws	RobotinoException if the current communication object is invalid.
				*/
				void setSetPointSpeed( float speed );

				/**
				* Resets the position of this motor.
				* @throws	RobotinoException if the current communication object is invalid.
				*/
				void resetPosition();

				/**
				* Controls the brakes of this motor.
				*
				* @param brake	If set to TRUE, this will activate the brake. If set to FALSE, the brake is released.
				* @throws	RobotinoException if the current communication object is invalid.
				*/
				void setBrake( bool brake );

				/**
				* Sets the proportional, integral and  differential constant of the PID controller.
				*
				* @throws	RobotinoException if the current communication object is invalid.
				*/
				void setPID( float kp, float ki, float kd );

				/**
				* Retrieves the actual speed of this motor.
				*
				* @return	Speed in rpm.
				* @throws	RobotinoException if the current communication object is invalid.
				*/
				float actualSpeed() const;

				/**
				* Retrieves the actual position of this motor.
				*
				* @throws	RobotinoException if the current communication object is invalid.
				*/
				int actualPosition() const;

				/**
				* Retrieves the current of this motor.
				* @throws	RobotinoException if the current communication object is invalid.
				*/
				float motorCurrent() const;

				/**
				* The current is measured by a 10 bit adc and is not converted into A.
				* @return The current delivered by to this motor. Range from 0 to 1023.
				* @throws	RobotinoException if the current communication object is invalid.
				*/
				float rawCurrentMeasurment() const;

			private:
				unsigned int _motorNumber;
			};
		}
	}
}
#endif
