#ifndef _NODE_SERVER_H_
#define _NODE_SERVER_H_


#include "mbed.h"
#include "EthernetInterface.h"
#include "xmlparser.h"
#include "xmlcall.h"
#include "tcp_ros.h"
#include <malloc.h>



void nodeServerStart(TCPSocketServer ssock,TCPSocketConnection csock,int port=40040);
//void SubServer(TCPSocketServer ssock,TCPSocketConnection csock,int port=40040);
#endif
