//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _XCOMDATASTREAM_H_
#define _XCOMDATASTREAM_H_

#include "xstdint.h"

class XComHeader;

class XComDataStreamEncoder
{
public:
  XComDataStreamEncoder( uint32_t initialSize = 100, uint32_t incrementSize = 100 );
  ~XComDataStreamEncoder();

  XComDataStreamEncoder( const XComDataStreamEncoder& other );

  XComDataStreamEncoder& operator=( const XComDataStreamEncoder& other );

  void clear();

  uint8_t* data();
  inline uint32_t dataSize() const;

  inline bool isEmpty() const;

  XComDataStreamEncoder& operator<<( const XComHeader& i );
  XComDataStreamEncoder& operator<<( uint8_t i );
  XComDataStreamEncoder& operator<<( int8_t i );
  XComDataStreamEncoder& operator<<( uint16_t i );
  XComDataStreamEncoder& operator<<( int16_t i );
  XComDataStreamEncoder& operator<<( uint32_t i );
  XComDataStreamEncoder& operator<<( int32_t i );
  XComDataStreamEncoder& operator<<( float i );

private:
  uint32_t _initialSize;
  uint32_t _incrementSize;
  uint8_t* _data;
  uint32_t _dataSize;
  uint32_t _currentSize;
  bool _isLittleEndianMachine;
  void reallocate();
};

uint32_t XComDataStreamEncoder::dataSize() const
{
  return _currentSize;
}

bool XComDataStreamEncoder::isEmpty() const
{
  return ( 0 == _currentSize );
}

class XComDataStreamDecoder
{
public:
  XComDataStreamDecoder( const uint8_t* data, uint32_t dataSize );

  XComDataStreamDecoder& operator>>( uint8_t& i );
  XComDataStreamDecoder& operator>>( int8_t& i );
  XComDataStreamDecoder& operator>>( uint16_t& i );
  XComDataStreamDecoder& operator>>( int16_t& i );
  XComDataStreamDecoder& operator>>( uint32_t& i );
  XComDataStreamDecoder& operator>>( int32_t& i );
  XComDataStreamDecoder& operator>>( float& i );

private:
  const uint8_t* _data;
  const uint32_t _dataSize;
  uint32_t _offset;
  bool _isLittleEndianMachine;
};

#endif