//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _XCOMIOCONTROL_H_
#define _XCOMIOCONTROL_H_

#include "xcom/xcomcommand.h"
#include "rec/iocontrol/sercom/qdsa_encoder.h"
#include "rec/robotino/com/northstar_interface.h"

using rec::robotino::com::NorthStarCommand;
using rec::robotino::com::NorthStarCalState;

class XComIOControl : public XComCommand
{
public:
  XComIOControl();
  ~XComIOControl();
  QDSA_Encoder* encoder();

  XComIOControl( const XComIOControl& other );
  XComIOControl& operator=( const XComIOControl& other );

  void setShutdown();
  bool getShutdown();

  void setImageRequest( uint8_t request );
  uint8_t getImageRequest() const;

  void setImagePort( uint16_t port );
  uint16_t getImagePort() const;

  void addVelocity( unsigned int motor, float value, float priority );
  void setVelocity( unsigned int motor, float value );
  float velocity( unsigned int motor );

  void setKp( unsigned int motor, float value );
  void setKi( unsigned int motor, float value );
  void setKd( unsigned int motor, float value );

  void setNorthStarCommand( const NorthStarCommand& northStarCommand );
  NorthStarCommand northStarCommand() const;

  bool operator==( const XComIOControl& c ) const;

  void reset();

	void setOdometry( float x, float y, float phi );
	void getOdometry( bool* isSet, float* x, float* y, float* phi );

private:
  QDSA_Encoder* _encoder;
  
  bool customDecode( XComDataStreamDecoder& in );
  void customEncode( XComDataStreamEncoder& out );

  float* _denominator;
  float* _velocity;

  uint8_t _imageRequest;
  uint8_t _shutdown;
  uint16_t _imagePort;
  NorthStarCommand _northStarCommand;

	unsigned char* _tmp_p2q_buffer;

	uint8_t _isSetOdometry;
	float _odometryX;
	float _odometryY;
	float _odometryPhi;
};

#endif