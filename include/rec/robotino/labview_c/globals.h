//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_LABVIEW_GLOBALS_H_
#define _REC_ROBOTINO_LABVIEW_GLOBALS_H_

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

#endif