//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _XCOMSTREAMSERVICE_H_
#define _XCOMSTREAMSERVICE_H_

#include "xcom/xcomnetworking.h"
#include "xstdint.h"

#include <string>

#ifdef USE_QT3
#include <qcstring.h>
#define USE_QT
#endif

class XComStreamService
{
public:
  XComStreamService();
  XComStreamService( const XComStreamService& service );
  ~XComStreamService();
  bool create();
  bool connect( std::string hostname, uint16_t port );

  /**Sendet data.*/
#ifdef USE_QT
  bool send( const QByteArray& data );
#endif
  bool send( const uint8_t* data, uint32_t length );
  
#ifdef USE_QT
  QByteArray recv( uint32_t numBytesToReceive, bool* ok ) const;
#endif
  bool recv( uint8_t* data, uint32_t numBytesToReceive ) const;

  /**Prüft zunächst, ob Daten zum Empfang bereit liegen. Liegen keine Daten vor, wird false zurückgegeben. Ansonsten
  wird recv aufgerufen und der Rückgabewert von recv zurückgegeben.*/
  bool tryRecv( uint8_t* data, uint32_t numBytesToReceive );

  /**Disable all sends and receives on the socket. You still have to call close().*/
  int shutdown();

  /**Closes the socket.*/
  void close();
  bool bind( uint16_t port );
  bool listen();
  XComStreamService accept();
  bool isValid();

  bool isConnected() const;

  std::string clientAddress() const;
  void setClientAddress( const std::string& address );

  void setSocket( SOCKET socket );

  const XComStreamService& operator=( const XComStreamService& );

  /**On success returns the IP address in dotted format. Returns an empty string if name can not be found.
  If name is already a valid IP address simply returns name.*/
  static std::string getHostByName( const std::string& name );

private:
  bool _isConnected;
  SOCKET _socket;
  sockaddr_in _clientService;
  std::string _hostname;
  uint16_t _port;
  std::string _clientAddress;

  void copy( const XComStreamService& source );

  void setBlocking( bool block );
};

#endif
