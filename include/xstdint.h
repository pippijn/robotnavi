//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _XSTDINT_H_
#define _XSTDINT_H_

#ifdef WIN32
typedef unsigned char uint8_t;
typedef char int8_t;
typedef unsigned short uint16_t;
typedef short int16_t;
typedef unsigned int uint32_t;
typedef int int32_t;
#if 1 // 32 bit system
typedef long long int int64_t;
typedef long long unsigned int uint64_t;
#else
typedef long int int64_t;
typedef long unsigend int uint64_t;
#endif
#else
#include <stdint.h>
#endif

#endif

