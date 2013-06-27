#ifndef _REC_IOCONTROL_REMOTESTATE_SETSTATE_H_
#define _REC_IOCONTROL_REMOTESTATE_SETSTATE_H_

#include "rec/iocontrol/robotstate/State.h"
#include "rec/robotino/com/northstar_interface.h"

namespace rec
{
	namespace iocontrol
	{
		namespace remotestate
		{
			/**
			@brief The SetState is the collection of all set point values being send to Robotino.
			*/
			class SetState
			{
			public:
				SetState();

				/**
				Reset this state to default values.
				*/
				void reset();

				bool brake[rec::iocontrol::robotstate::State::numMotors];
				float setPointSpeed[rec::iocontrol::robotstate::State::numMotors];
				bool resetPosition[rec::iocontrol::robotstate::State::numMotors];

				/** proportional constant */
				float kp[rec::iocontrol::robotstate::State::numMotors];
				/** integral constant */
				float ki[rec::iocontrol::robotstate::State::numMotors];
				/** differential constant */
				float kd[rec::iocontrol::robotstate::State::numMotors];
				/** digital outputs */
				bool dOut[rec::iocontrol::robotstate::State::numDigitalOutputs];
				/** relays */
				bool relays[rec::iocontrol::robotstate::State::numRelays];

				float powerOutputSetPoint;

				bool encoderInputResetPosition;
				
				/**
				0: undefined
				1: QVGA
				2: VGA
				*/
				unsigned char imageResolution;

				bool closedGripper;
				bool enableGripper;

				rec::robotino::com::NorthStarCommand northStarCommand;

				bool setOdometry;
				float odometryX;
				float odometryY;
				float odometryPhi;

				bool shutdown;

				bool isDriveSystemControl;
				float vx;
				float vy;
				float vomega;
			};
		}
	}
}

#endif
