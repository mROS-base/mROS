/*
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

//#define _DEBUG_RPC_HANDLER

#include "RPCHandler.h"
#include "mbed_rpc.h"

#define RPC_DATA_LEN 128

RPCHandler::RPCHandler(const char* rootPath, const char* path, TCPSocketConnection* pTCPSocketConnection) : HTTPRequestHandler(rootPath, path, pTCPSocketConnection)
{}

RPCHandler::~RPCHandler()
{
#ifdef _DEBUG_RPC_HANDLER
    printf("++++(RPC Handler)Handler destroyed\r\n");
#endif
}

void RPCHandler::doGet()
{
#ifdef _DEBUG_RPC_HANDLER
    printf("++++(RPC Handler)doGet\r\n");
#endif
    char resp[RPC_DATA_LEN] = {0};
    char req[RPC_DATA_LEN] = {0};

#ifdef _DEBUG_RPC_HANDLER
    printf("++++(RPC Handler)Path : %s\r\n", path().c_str());
    printf("++++(RPC Handler)Root Path : %s\r\n", rootPath().c_str());
#endif
    //Remove path
    strncpy(req, path().c_str(), RPC_DATA_LEN-1);
#ifdef _DEBUG_RPC_HANDLER
    printf("++++(RPC Handler)RPC req(before) : %s\r\n", req);
#endif
    //Remove "%20", "+", "," from req
    cleanReq(req);
#ifdef _DEBUG_RPC_HANDLER
    printf("++++(RPC Handler)RPC req(after) : %s\r\n", req);
#endif
    //Do RPC Call
    RPC::call(req, resp); //FIXME: Use bool result
#ifdef _DEBUG_RPC_HANDLER
    printf("++++(RPC Handler)Response %s \r\n",resp);
#endif
    //Response
    setContentLen( strlen(resp) );

    //Make sure that the browser won't cache this request
    respHeaders()["Cache-control"]="no-cache;no-store";
    respHeaders()["Pragma"]="no-cache";
    respHeaders()["Expires"]="0";

    //Write data
    respHeaders()["Connection"] = "close";
    writeData(resp, strlen(resp));
#ifdef _DEBUG_RPC_HANDLER
    printf("++++(RPC Handler)Exit RPCHandler::doGet()\r\n");
#endif
}

void RPCHandler::doPost()
{

}

void RPCHandler::doHead()
{

}


void RPCHandler::onReadable() //Data has been read
{

}

void RPCHandler::onWriteable() //Data has been written & buf is free
{
#ifdef _DEBUG_RPC_HANDLER
    printf("++++(RPC Handler)onWriteable event\r\n");
#endif
   // close(); //Data written, we can close the connection
}

void RPCHandler::onClose() //Connection is closing
{
    //Nothing to do
}

void RPCHandler::cleanReq(char* data)
{
    char* p;
    if((p = strstr(data, "+"))!=NULL)memset((void*) p, ' ', 1);
    else if((p = strstr(data, ","))!=NULL)memset((void*) p, ' ', 1);
    else if((p = strstr(data, "%20"))!=NULL) {
        memset((void*) p, ' ', 1);
        while(*(p+2)!=NULL) {
            p++;
            memset((void*) p,*(p+2),1);
        }
    }

    if((p = strstr(data, "+"))!=NULL)memset((void*) p, ' ', 1);
    else if((p = strstr(data, ","))!=NULL)memset((void*) p, ' ', 1);
    else if((p = strstr(data, "%20"))!=NULL) {
        memset((void*) p, ' ', 1);
        while(*(p+2)!=NULL) {
            p++;
            memset((void*) p,*(p+2),1);
        }
    }
}


