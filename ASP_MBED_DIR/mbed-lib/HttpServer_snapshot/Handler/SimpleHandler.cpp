/*
Copyright (c) 2010 Donatien Garnier (donatiengar [at] gmail [dot] com)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
#define _DEBUG_SIMPLE_HANDLER

#include "SimpleHandler.h"

SimpleHandler::SimpleHandler(const char* rootPath, const char* path, TCPSocketConnection* pTCPSocketConnection) : HTTPRequestHandler(rootPath, path, pTCPSocketConnection)
{
#ifdef _DEBUG_SIMPLE_HANDLER
    printf("++++(SimpleHeader)Initialize\r\n");
#endif
}

SimpleHandler::~SimpleHandler()
{
#ifdef _DEBUG_SIMPLE_HANDLER
    printf("++++(SimpleHeader)Handler destroyed\r\n");
#endif
}

void SimpleHandler::doGet()
{
#ifdef _DEBUG_SIMPLE_HANDLER
    printf("++++(SimpleHeader) doGet()\r\n");
#endif
    const char* resp = "Hello world !";
    setContentLen( strlen(resp) );
    respHeaders()["Connection"] = "close";
    writeData(resp, strlen(resp));
#ifdef _DEBUG_SIMPLE_HANDLER
    printf("++++(SimpleHeader) doGet Exit\r\n");
#endif
}

void SimpleHandler::doPost()
{

}

void SimpleHandler::doHead()
{

}


void SimpleHandler::onReadable() //Data has been read
{

}

void SimpleHandler::onWriteable() //Data has been written & buf is free
{
#ifdef _DEBUG_SIMPLE_HANDLER
    printf("\r\n++++SimpleHandler::onWriteable() event\r\n");
#endif
//    close(); //Data written, we can close the connection
}

void SimpleHandler::onClose() //Connection is closing
{
    //Nothing to do
}
