//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_COM_C_GLOBALS_H_
#define _REC_ROBOTINO_COM_C_GLOBALS_H_

typedef int BOOL;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef NULL
#define NULL 0
#endif

#ifdef WIN32
  #ifdef __cplusplus
    #define DLLEXPORT extern "C" __declspec(dllexport)
  #else
    #define DLLEXPORT __declspec(dllexport)
  #endif
#else
  #ifdef __cplusplus
    #define DLLEXPORT extern "C"
  #else
    #define DLLEXPORT
  #endif
#endif

#endif //_REC_ROBOTINO_COM_C_GLOBALS_H_

/**  \mainpage rec_robotino_com_c API documentation

This is a C wrapper to the <A HREF="../rec_robotino_com/index.html">rec::robotino::com</A> API for Robotino. Below is an example how to use this interface.

<PRE>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>

#include "rec/robotino/com/c/com.h"

#ifdef WIN32
#include <windows.h>
// _getch
#include <conio.h>
#else
// getchar
#include <stdio.h>
// usleep
#include <unistd.h>
#endif

ComId com;
OmniDriveId omniDrive;
BumperId bumper;

void msleep( unsigned int ms )
{
#ifdef WIN32
  SleepEx( ms, FALSE );
#else
  ::usleep( ms * 1000 );
#endif
}

void waitForKey()
{
#ifdef WIN32
  _getch();
#else
  ::getchar();
#endif
}

//rotate vector in by deg degrees and store the output in out
void rotate( const float* in, float* out, float deg )
{
  const float pi = 3.14159265358979f;

  float rad = 2 * pi / 360.0f * deg;

  out[0] = (float)( cos( rad ) * in[0] - sin( rad ) * in[1] );
  out[1] = (float)( sin( rad ) * in[0] + cos( rad ) * in[1] );
}

void drive()
{
  const float startVector[2] = {200.0f, 0.0f};
  float dir[2];
  float a = 0.0f;
  unsigned int msecsElapsed = 0;

  while( com_isConnected( com ) && FALSE == bumper_value( bumper ) )
  {
    //rotate 360degrees in 10s
    rotate( startVector, dir, a );
    a = 360.0f * msecsElapsed / 10000;

    omnidrive_setVelocity( omniDrive, dir[0], dir[1], 0 );

    msleep( 50 );
    msecsElapsed += 50;
  }
}

void error( const char* message )
{
  printf( "%s\n", message );
  printf( "Press any key to exit..." );
  waitForKey();
  exit( 1 );
}

int main( int argc, char **argv )
{
  com = com_construct();

  if( argc > 1 )
  {
    com_setAddress( com, argv[1] );
  }
  else
  {
    //com_setAddress( com, "172.26.1.1" );
    com_setAddress( com, "192.168.101.101" );
  }

  if( FALSE == com_connect( com ) )
  {
    error( "Error on connect" );
  }
  else
  {
    char addressBuffer[256];
    com_address( com, addressBuffer, 256 );
    printf( "Connected to %s\n", addressBuffer );
  }

  omniDrive = omnidrive_construct();
  omnidrive_setComId( omniDrive, com );

  bumper = bumper_construct();
  bumper_setComId( bumper, com );

  drive();

  omnidrive_destroy( omniDrive );
  bumper_destroy( bumper );
  com_destroy( com );

  printf( "Press any key to exit...\n" );

  waitForKey();
}

</PRE>
*/
