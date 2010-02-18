//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _XCOMDGRAMSERVICE_H_
#define _XCOMDGRAMSERVICE_H_

#include <xcom/xcomnetworking.h>
#include "xstdint.h"
#include <string>

class XComDgramService
{
public:
  XComDgramService();

  bool create();
  void close();
  void shutdown();
  bool bind( uint16_t port );
  bool isBound();
  bool ready();

  bool send( const uint8_t* data, unsigned int length, sockaddr_in* addr, socklen_t addrlen );
  bool send( const uint8_t* data, unsigned int length, std::string address , uint16_t port );

  uint32_t recv( uint8_t* data, uint32_t maxNumBytesToReceive );

  uint16_t port();

private:
  SOCKET _socket;
  uint16_t _port;
  sockaddr_in _recvAddr;
  sockaddr_in _senderAddr;
  int _senderAddrSize;
  bool _isBound;
};

#endif
