//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _XCOMCOMMAND_H_
#define _XCOMCOMMAND_H_

#include <xcom/xcomdatastream.h>

#include <boost/thread/mutex.hpp>
#define XCOMCOMMAND_MUTEXLOCKER {boost::mutex::scoped_lock lk( _mutex );}

#define RECEIVE_TIMEOUT 1000 //ms

typedef uint8_t commandid_t;
typedef uint16_t commandlength_t;

class XComCommand
{
public:
  enum{ IOControl = 0, IOStatus, CameraControl, KeepAlive, RawData, ConnectionBusy, Info, DummyCom1, DummyCom2, DummyCom3, IDMAX=50 };
  enum{ MaxCommandLength = 32000 };

  const commandid_t id;

  XComCommand( const commandid_t i );
  bool decode( const uint8_t* data, uint32_t length );
  void encode( XComDataStreamEncoder* out, bool resetAfterEncode = true );

protected:
  virtual bool customDecode( XComDataStreamDecoder& in );
  virtual void customEncode( XComDataStreamEncoder& out );

  bool _resetAfterEncode;
  boost::mutex _mutex;
};

#endif
