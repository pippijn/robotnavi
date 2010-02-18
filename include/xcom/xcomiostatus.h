//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _XCOMIOSTATUS_H_
#define _XCOMIOSTATUS_H_

#include "xcom/xcomcommand.h"
#include <string>
#include "rec/robotino/com/northstar_interface.h"

using rec::robotino::com::NorthStarReadings;

class QDSA_Encoder;

class XComIOStatus : public XComCommand
{
public:
  XComIOStatus();
  
  XComIOStatus( const XComIOStatus& other );
  XComIOStatus& operator=( const XComIOStatus& other );

  ~XComIOStatus();

  QDSA_Encoder* encoder();
  void setSerdFreq( uint16_t f );

  uint16_t serdFreq() const;

  NorthStarReadings northStarReadings() const;
  void setNorthStarReadings( const NorthStarReadings& northStarReadings );

  void setVelocity( uint8_t motor, float value );
  void setAD( std::string name, unsigned int value );

  void setPassiveMode( bool passive );
  inline bool isPassiveMode() const;

  static unsigned int encodedDataSize();

  void reset();

	void setOdometry( float x, float y, float phi );
	void getOdometry( float* x, float* y, float* phi );

private:
  bool customDecode( XComDataStreamDecoder& in );
  void customEncode( XComDataStreamEncoder& out );
  
  uint16_t _serdFreq;
  QDSA_Encoder* _encoder;
  NorthStarReadings _northStarReadings;
  bool _isPassiveMode;
	
	unsigned char* _tmp_q2p_buffer;

	float _odometryX;
	float _odometryY;
	float _odometryPhi;
};

bool XComIOStatus::isPassiveMode() const{ return _isPassiveMode; }

#endif
