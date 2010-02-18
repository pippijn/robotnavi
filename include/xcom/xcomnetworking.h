//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifdef WIN32

#include "winsock2.h"

typedef int socklen_t;

#define SHUT_RDWR SD_BOTH

#else

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/tcp.h>

#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define SOCKADDR sockaddr
#define closesocket ::close

typedef int SOCKET;

#endif

