//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _XCOMKEEPALIVE_H_
#define _XCOMKEEPALIVE_H_

#include <xcom/xcomcommand.h>

class XComKeepAlive : public XComCommand
{
public:
  XComKeepAlive()
  : XComCommand( KeepAlive )
  {
  }

  XComKeepAlive( const XComKeepAlive& other )
  : XComCommand( KeepAlive )
  {
  }

  XComKeepAlive& operator=( const XComKeepAlive& other )
  {
    return *this;
  }
};

#endif