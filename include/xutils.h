//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _XUTILS_H_
#define _XUTILS_H_

#ifdef WIN32
#include "winsock2.h"
#include <conio.h>
static void msleep( int x )
{
  Sleep( x );
};
static int mygetch()
{
  return _getch();
}
#else
#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include <string>
#include <sys/wait.h>
#include <sys/types.h>
static void msleep( int x )
{
  usleep( 1000*x );
};
static int mygetch()
{
  struct termios oldt,newt;
  int ch;
  tcgetattr( STDIN_FILENO, &oldt );
  newt = oldt;
  newt.c_lflag &= ~( ICANON | ECHO );
  tcsetattr( STDIN_FILENO, TCSANOW, &newt );
  ch = getchar();
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
  return ch;
};
static int tsystem( const std::string& path )
{
  // reset sigchld
  struct sigaction act;
  memset( &act, 0, sizeof( act ) );
  act.sa_flags = SA_RESETHAND;
  act.sa_handler = SIG_DFL;
  sigaction( SIGCHLD, &act, NULL );

  int status = system( path.c_str() );
  /*
  pid_t id = fork();
  if( id == 0 )
  {
    if( execlp( path.c_str(), path.c_str(), NULL ) == -1 )
    {
      // error
      exit(1);
    }
  }
  else
  {
    int status = 0;
    std::cerr << "id: " << id << std::endl;
    std::cerr << waitpid( id, &status, __WALL );
    std::cerr << "status: " << status << std::endl;
    return status;
  }*/
  memset( &act, 0, sizeof( act ) );
  act.sa_flags = SA_NOCLDWAIT;
  act.sa_handler = SIG_IGN;
  sigaction( SIGCHLD, &act, NULL );
  return status;
};
#endif

#define PI 3.14159265358979f

#endif

