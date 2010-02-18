//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _XCOMIMAGEHEADER_H_
#define _XCOMIMAGEHEADER_H_

#include "robotinocom/robotinocom.h"
#include "xcom/xcomdatastream.h"

class XComImageHeader
{
public:
  static unsigned int encodedDataSize();
  static unsigned int stopSequenceSize();
  static bool writeStopSequence( unsigned char* data, unsigned int dataSize );
  static bool findStopSequence( const unsigned char* data, unsigned int dataSize );

  XComImageHeader();
  void setImageType( RobotinoImageType type );
  void setResolution( RobotinoResolution resolution );
  void setWidth( unsigned int width );
  void setHeight( unsigned int height );
  void setNumColorChannels( unsigned char n );
  void setBitsPerChannels( unsigned int n );
  void setImageDataSize( uint32_t size );
  void setParameters( const RobotinoImageParameters& parameters );

  RobotinoImageParameters parameters() const;
  uint32_t imageDataSize() const;

  void encode();
  const unsigned char* encodedData();

  bool decode( const unsigned char* data, unsigned int dataSize );

private:
  XComDataStreamEncoder _out;
  RobotinoImageParameters _parameters;

  uint32_t _imageDataSize;

  static unsigned int _encodedDataSize;
  static const unsigned int _startSequenceSize = 10;
  static const uint8_t _startSequence[_startSequenceSize];
  static const unsigned int _stopSequenceSize = 10;
  static const uint8_t _stopSequence[_stopSequenceSize];
};

#endif