#ifndef _NODE_SERVER_H_
#define _NODE_SERVER_H_


#include "mbed.h"
#include "EthernetInterface.h"
#include "xmlparser.h"
#include "xmlcall.h"
#include "tcp_ros.h"
#include <malloc.h>
#include "SoftPWM.h"
/*
void Triger ();
void RiseEcho();
void FallEcho();
void init();
*/
void nodeServerStart(TCPSocketServer ssock,TCPSocketConnection csock,int port=40040);

#endif
