/*  Interface to NorthStar
Copyright (C) 2006 REC GmbH
*/

#ifndef _REC_ROBOTINO_COM_NORTHSTAR_INTERFACE_H_
#define _REC_ROBOTINO_COM_NORTHSTAR_INTERFACE_H_

namespace rec
{
	namespace robotino
	{
		namespace com
		{
			/**
			* @brief	Data read from Evolution Robotics NorthStar
			*
			* This class holds data delivered by the NorthStar sensor
			*/
			class NorthStarReadings
			{
			public:
				NorthStarReadings()
				{
					reset();
				}

				/**
				* The sequence number is increased whenever th Northstar delivers new data.
				*/
				inline unsigned int sequenceNo() const;

				/**
				* The current room id.
				*/
				inline char roomId() const;

				/**
				* The number of visible spots.
				*/
				inline unsigned char numSpotsVisible() const;

				/**
				* The current position in x direction. The scale depends on the calibration.
				*/
				inline signed short posX() const;

				/**
				* The current position in y direction. The scale depends on the calibration.
				*/
				inline signed short posY() const;

				/**
				* The current orientation in degrees.
				*/
				inline float posTheta() const;

				/**
				* The signal strength of spot 1.
				*/
				inline unsigned short magSpot0() const;

				/**
				* The signal strength of spot 2.
				*/
				inline unsigned short magSpot1() const;

				/**
				* @see sequenceNo
				*/
				inline void setSequenceNo( unsigned int number );
				inline void incSequenceNo();
				inline void setRoomId( char roomId );
				inline void setNumSpotsVisible( unsigned char numSpotsVisible );
				inline void setPosX( signed short posX );
				inline void setPosY( signed short posY );
				inline void setPosTheta( float posTheta );
				inline void setMagSpot0( unsigned short magSpot0 );
				inline void setMagSpot1( unsigned short magSpot1 );

				/**
				* Reset all values to default.
				*/
				void reset()
				{
					_sequenceNo = 0;
					_roomId = 0 ;
					_numSpotsVisible = 0 ;
					_posX = 0 ;
					_posY = 0 ;
					_posTheta = 0.0 ;
					_magSpot0 = 0 ;
					_magSpot1 = 0 ;
				}

			private:
				unsigned int _sequenceNo;
				char _roomId;
				unsigned char _numSpotsVisible;
				signed short _posX;
				signed short _posY;
				float _posTheta;
				unsigned short _magSpot0;
				unsigned short _magSpot1;
			};

			unsigned int NorthStarReadings::sequenceNo() const{ return _sequenceNo; }
			char NorthStarReadings::roomId() const{ return _roomId; }
			unsigned char NorthStarReadings::numSpotsVisible() const{ return _numSpotsVisible; }
			signed short NorthStarReadings::posX() const{ return _posX; }
			signed short NorthStarReadings::posY() const{ return _posY; }
			float NorthStarReadings::posTheta() const{ return _posTheta; }
			unsigned short NorthStarReadings::magSpot0() const{ return _magSpot0; }
			unsigned short NorthStarReadings::magSpot1() const{ return _magSpot1; }

			void NorthStarReadings::setSequenceNo( unsigned int number ){ _sequenceNo = number; }
			void NorthStarReadings::incSequenceNo(){ ++_sequenceNo; if( _sequenceNo <= 1 ) _sequenceNo = 2; }
			void NorthStarReadings::setRoomId( char roomId ){ _roomId = roomId; }
			void NorthStarReadings::setNumSpotsVisible( unsigned char numSpotsVisible ){ _numSpotsVisible = numSpotsVisible; }
			void NorthStarReadings::setPosX( signed short posX ){ _posX = posX; }
			void NorthStarReadings::setPosY( signed short posY ){ _posY = posY; }
			void NorthStarReadings::setPosTheta( float posTheta ){ _posTheta = posTheta; }
			void NorthStarReadings::setMagSpot0( unsigned short magSpot0 ){ _magSpot0 = magSpot0; }
			void NorthStarReadings::setMagSpot1( unsigned short magSpot1 ){ _magSpot1 = magSpot1; }

			/**
			* @brief The current state of the calibration.
			*/
			typedef enum{ UndefinedCalState=-1, RotationStart=0, RotationStop, LinearStart, LinearStop, RotationStopResetWeights=9, LinearStopResetWeights=11 } NorthStarCalState;

			/**
			* @brief The current mode of the calibration.
			*/
			typedef enum{ ReportMode=0, Calibration, StartReport, StopReport, ResetSequenceNumber } NorthStarCalFlag;

			/**
			* @brief	Command send to Evolution Robotics NorthStar
			*
			* This class holds a command for NorthStar.
			*/
			class NorthStarCommand
			{
			public:
				NorthStarCommand()
				{
					reset();
				}

				inline char roomId() const;
				inline NorthStarCalState calState() const;
				inline unsigned char calFlag() const;
				inline unsigned short calDistance() const;

				inline void setRoomId( char roomId );

				inline void setCalState( unsigned char calState );

				inline void setCalFlag( unsigned char calFlag );

				inline void setCalDistance( unsigned short calDistance );

				void reset()
				{
					_roomId = -1;
					_calState = 0;
					_calFlag = 0;
					_calDistance = 10;
				}

			private:
				char _roomId;
				unsigned char _calState;
				
				/**
				0: continue current operation
				1: set current position as new calibration position
				*/
				unsigned char _calFlag;
				unsigned short _calDistance;
			};

			char NorthStarCommand::roomId() const{ return _roomId; }
			NorthStarCalState NorthStarCommand::calState() const{ return (NorthStarCalState)_calState; }
			unsigned char NorthStarCommand::calFlag() const{ return _calFlag; }
			unsigned short NorthStarCommand::calDistance() const{ return _calDistance; }

			void NorthStarCommand::setRoomId( char roomId ){ _roomId = roomId; }
			void NorthStarCommand::setCalState( unsigned char calState ){ _calState = calState; }
			void NorthStarCommand::setCalFlag( unsigned char calFlag ){ _calFlag = calFlag; }
			void NorthStarCommand::setCalDistance( unsigned short calDistance ){ _calDistance = calDistance; }
		}
	}
}

#endif //_REC_ROBOTINO_COM_NORTHSTAR_INTERFACE_H_

