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
#ifdef _DEBUG_ALL
#define _DEBUG_REQUEST_HANDLER
#endif

#include "HTTPRequestHandler.h"

#include <string.h>

#define HTTP_REQUEST_TIMEOUT 5000

//HTTPRequestHandler::HTTPRequestHandler(const char* rootPath, const char* path, TCPSocket* pTCPSocket) : NetService(),
//   m_pTCPSocketConnection(pTCPSocketConnection), m_reqHeaders(), m_respHeaders(),
//   m_rootPath(rootPath), m_path(path), m_errc(200),
//   m_watchdog(), m_timeout(0),**/ m_closed(false), m_headersSent(false) //OK
HTTPRequestHandler::HTTPRequestHandler(const char* rootPath, const char* path, TCPSocketConnection* pTCPSocketConnection) :
    m_pTCPSocketConnection(pTCPSocketConnection), m_reqHeaders(), m_respHeaders(),
    m_rootPath(rootPath), m_path(path), m_errc(200), m_closed(false), m_headersSent(false)
{
#ifdef _DEBUG_REQUEST_HANDLER
    printf("+++(HTTPRequestHandler) init \r\n");
#endif
    //Read & parse headers
    readHeaders();
//*  m_pTCPSocket->setOnEvent(this, &HTTPRequestHandler::onTCPSocketEvent);
//*  setTimeout(HTTP_REQUEST_TIMEOUT);
#ifdef _DEBUG_REQUEST_HANDLER
    printf("+++(HTTPRequestHandler) init end \r\n");
#endif
}

HTTPRequestHandler::~HTTPRequestHandler()
{
    close();
#ifdef _DEBUG_REQUEST_HANDLER
    printf("+++(HTTPRequestHandler) Destroy end\r\n");
#endif
}

void HTTPRequestHandler::onTimeout() //Connection has timed out
{
    close();
}

void HTTPRequestHandler::close() //Close socket and destroy data
{
    if(m_closed)
        return;
    m_closed = true; //Prevent recursive calling or calling on an object being destructed by someone else
    /**  m_watchdog.detach(); **/
//*  onClose();
//*  m_pTCPSocket->resetOnEvent();
//*  m_pTCPSocket->close();
//*  delete m_pTCPSocket; //Can safely destroy socket
//*  NetService::close();
}

map<string, string>& HTTPRequestHandler::reqHeaders() //const
{
    return m_reqHeaders;
}

string& HTTPRequestHandler::path() //const
{
    return m_path;
}

int HTTPRequestHandler::dataLen() const
{
    map<string,string>::const_iterator it;
    it = m_reqHeaders.find("Content-Length");
    if( it == m_reqHeaders.end() ) {
        return 0;
    }
    return atoi((*it).second.c_str()); //return 0 if parse fails, so that's fine
}

int HTTPRequestHandler::readData(char* buf, int len)
{
    return m_pTCPSocketConnection->receive(buf, len);
}

string& HTTPRequestHandler::rootPath() //const
{
    return m_rootPath;
}

void HTTPRequestHandler::setErrCode(int errc)
{
    m_errc = errc;
}

void HTTPRequestHandler::setContentLen(int len)
{
    char len_str[6] = {0};
    sprintf(len_str, "%d", len);
    respHeaders()["Content-Length"] = len_str;
}

map<string, string>& HTTPRequestHandler::respHeaders()
{
    return m_respHeaders;
}

int HTTPRequestHandler::writeData(const char* buf, int len)
{
    if(!m_headersSent) {
        m_headersSent = true;
        writeHeaders();
    }
    return m_pTCPSocketConnection->send((char *)buf, len);
}
/**
void HTTPRequestHandler::setTimeout(int ms)
{
  m_timeout = 1000*ms;
  resetTimeout();
}
**/
/**
void HTTPRequestHandler::resetTimeout()
{
  m_watchdog.detach();
  m_watchdog.attach_us<HTTPRequestHandler>(this, &HTTPRequestHandler::onTimeout, m_timeout);
}
**/

void HTTPRequestHandler::readHeaders()
{
    while( readLine(line, 128) > 0) { //if == 0, it is an empty line = end of headers
        int n = sscanf(line, "%[^:]: %[^\n]", key, value);
        if ( n == 2 ) {
#ifdef _DEBUG_REQUEST_HANDLER
            printf("\r\n+++(HTTPRequestHandler)Read header : %s : %s\r\n", key, value);
#endif
            m_reqHeaders[key] = value;
        }
        //TODO: Impl n==1 case (part 2 of previous header)
    }
}

void HTTPRequestHandler::writeHeaders() //Called at the first writeData call
{
    //Response line
    sprintf(line, "HTTP/1.1 %d MbedInfo\r\n", m_errc); //Not a violation of the standard not to include the descriptive text
    m_pTCPSocketConnection->send(line, strlen(line));

    map<string,string>::iterator it;
    while( !m_respHeaders.empty() ) {
        it = m_respHeaders.begin();
        sprintf(line, "%s: %s\r\n", (*it).first.c_str(), (*it).second.c_str() );
#ifdef _DEBUG_REQUEST_HANDLER
        printf("\r\n+++(HTTPRequestHandler)%s", line);
#endif
        m_pTCPSocketConnection->send(line, strlen(line));
        m_respHeaders.erase(it);
    }
    m_pTCPSocketConnection->send("\r\n",2); //End of head
}

int HTTPRequestHandler::readLine(char* str, int maxLen)
{
    int ret;
    int len = 0;
    for(int i = 0; i < maxLen - 1; i++) {
        ret = m_pTCPSocketConnection->receive(str, 1);
        if(ret <= 0) {
            break;
        }
        if( (len > 1) && *(str-1)=='\r' && *str=='\n' ) {
            str--;
            len-=2;
            break;
        } else if( *str=='\n' ) {
            len--;
            break;
        }
        str++;
        len++;
    }
    *str = 0;
    return len;
}
/**
void HTTPRequestHandler::onTCPSocketEvent(TCPSocketEvent e)
{
  //printf("\r\nEvent %d in HTTPRequestHandler\r\n", e);
  printf("\r\n+++(HTTPRequestHandler)Event in HTTPRequestHandler\r\n");

  if(m_closed)
  {
    printf("\r\n+++(HTTPRequestHandler)WARN: Discarded\r\n");
    return;
  }

  switch(e)
  {
        case TCPSOCKET_READABLE:
    resetTimeout();
   onReadable();
    break;
  case TCPSOCKET_WRITEABLE:
    resetTimeout();
    onWriteable();
    break;
  case TCPSOCKET_CONTIMEOUT:
  case TCPSOCKET_CONRST:
  case TCPSOCKET_CONABRT:
  case TCPSOCKET_ERROR:
  case TCPSOCKET_DISCONNECTED:
    DBG("\r\nConnection error in handler\r\n");
    close();
    break;
  }
}
**/