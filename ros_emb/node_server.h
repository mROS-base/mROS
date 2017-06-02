#ifndef _NODE_SERVER_H_
#define _NODE_SERVER_H_


#include "mbed.h"
#include "EthernetInterface.h"



void nodeServerStart(TCPSocketServer ssock,TCPSocketConnection csock,int port=8000);

#endif
