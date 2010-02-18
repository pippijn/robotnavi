#ifndef _REC_IOCONTROL_REMOTESTATE_SENSORSTATE_H_
#define _REC_IOCONTROL_REMOTESTATE_SENSORSTATE_H_

#include "rec/iocontrol/robotstate/State.h"
#include "rec/robotino/com/northstar_interface.h"

namespace rec
{
	namespace iocontrol
	{
		namespace remotestate
		{
			/**
			@brief The SensorState is the collection of all sensor readings received from Robotino.
			*/
			class SensorState
			{
			public:
				SensorState();

				/**
				Reset this state to default values.
				*/
				void reset();

				float powerOutputCurrent;
				float powerOutputRawCurrent;
				
				int encoderInputPosition;
				int encoderInputVelocity;

				//Velocity/Position
				float actualSpeed[rec::iocontrol::robotstate::State::numMotors];
				int actualPosition[rec::iocontrol::robotstate::State::numMotors];
				float motorCurrent[rec::iocontrol::robotstate::State::numMotors];
				float rawMotorCurrent[rec::iocontrol::robotstate::State::numMotors];

				//digital inputs
				bool dIn[rec::iocontrol::robotstate::State::numDigitalInputs];
				//analog inputs
				float aIn[rec::iocontrol::robotstate::State::numAnalogInputs];
				//distance sensors
				float distanceSensor[rec::iocontrol::robotstate::State::numDistanceSensors];
				//bumper
				bool bumper;

				float current;
				float voltage;

				std::string info;
				bool isPassiveMode;

				bool isGripperOpened;
				bool isGripperClosed;

				rec::robotino::com::NorthStarReadings northStarReadings;

				float odometryX;
				float odometryY;
				float odometryPhi;
			};
		}
	}
}

#endif
