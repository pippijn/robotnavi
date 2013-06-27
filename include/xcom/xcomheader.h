//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _XCOMHEADER_H_
#define _XCOMHEADER_H_

#include <xcom/xcomcommand.h>

class XComHeader
{
private:
  commandid_t _id;
  commandlength_t _length;
public:
  static const unsigned int length = sizeof( commandid_t ) + sizeof( commandlength_t );
  XComHeader();
  XComHeader( commandid_t id, commandlength_t length );
  void decode( const uint8_t* data, uint32_t dataSize );
  bool isValid();
  static uint8_t* encode( commandid_t id, commandlength_t length, uint32_t* dataSize );
  
  void setLength( commandlength_t length );
  
  commandid_t getId() const;
  commandlength_t getLength() const;
};

#endif