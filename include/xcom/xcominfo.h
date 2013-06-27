//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _XCOMINFO_H_
#define _XCOMINFO_H_

#include "xcom/xcomcommand.h"
#include "robotinocom/robotinoinfo.h"

class XComInfo : public XComCommand
{
public:
  XComInfo();
  XComInfo( const XComInfo& other );
  XComInfo& operator=( const XComInfo& other );

  static unsigned int encodedDataSize();

  void setInfo( const RobotinoInfo& info );
  inline RobotinoInfo info() const;

private:
  bool customDecode( XComDataStreamDecoder& in );
  void customEncode( XComDataStreamEncoder& out );

  RobotinoInfo _info;
  uint8_t _messageBuffer[RobotinoInfo::MaxMessageLength+1];
};

RobotinoInfo XComInfo::info() const{ return _info; }


#endif