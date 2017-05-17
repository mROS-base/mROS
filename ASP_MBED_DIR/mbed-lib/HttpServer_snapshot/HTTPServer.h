#define _DEBUG_ALL

#ifndef HTTP_SERVER_H
#define HTTP_SERVER_H

#ifdef _DEBUG_ALL
#define _DEBUG_HTTP_SERVER_H
#endif

#include <string>
using std::string;

#include <map>
using std::map;

#include "HTTPRequestHandler.h"
//#include "rtos.h"
#include "cmsis_os.h"
#include "mbed.h"
#include "EthernetInterface.h"
//#include "HTTPRequestDispatcher.h"

//#include "dbg/dbg.h"

#define THREAD_MAX 5
//Thread *threads[THREAD_MAX];
//Thread *xthread;
//#include "HTTPServer2.h"

uint8_t threads_id[THREAD_MAX];
uint8_t xthread_id = 0;

struct handlersComp { //Used to order handlers in the right way
    bool operator() (const string& handler1, const string& handler2) const {
        //The first handler is longer than the second one
        if (handler1.length() > handler2.length())
            return true; //Returns true if handler1 is to appear before handler2
        else if (handler1.length() < handler2.length())
            return false;
        else //To avoid the == case, sort now by address
            return ((&handler1)>(&handler2));
    }
};

map< string, HTTPRequestHandler*(*)(const char*, const char* , TCPSocketConnection* ), handlersComp > m_lpHandlers;
template<typename T>
void HTTPServerAddHandler(const char* path)  //Template decl in header
{
    m_lpHandlers[path] = &T::inst;
}

void ListenThread(void const *args);
enum HTTP_METH {
    HTTP_GET,
    HTTP_POST,
    HTTP_HEAD
};

bool getRequest(TCPSocketConnection* client,string* path, string* meth)
{
    char req[128];
    char c_path[128];
    char c_meth[128];
    const int maxLen = 128;
    char* p = req;
    //Read Line
    int ret;
    int len = 0;
    for(int i = 0; i < maxLen - 1; i++) {
        ret = client->receive(p, 1);
        if(!ret) {
            break;
        }
        if( (len > 1) && *(p-1)=='\r' && *p=='\n' ) {
            p--;
            len-=2;
            break;
        } else if( *p=='\n' ) {
            len--;
            break;
        }
        p++;
        len++;
    }
    *p = 0;
#ifdef _DEBUG_HTTP_SERVER_H
    printf("Parsing request : %s\r\n", req);
#endif
    ret = sscanf(req, "%s %s HTTP/%*d.%*d", c_meth, c_path);
    if(ret !=2)        return false;
    *meth = string(c_meth);
    *path = string(c_path);
    return true;
}

#define _DEBUG_HTTP_SERVER_H 1

void dispatchRequest(TCPSocketConnection* client)
{
    string path;
    string meth;
    HTTP_METH methCode;
#ifdef _DEBUG_HTTP_SERVER_H
    printf("Dispatching req\r\n");
#endif
    if( !getRequest(client,&path, &meth ) ) {
#ifdef _DEBUG_HTTP_SERVER_H
        printf("dispatchRequest Invalid request\r\n");
#endif
        //close();
        return; //Invalid request
    }
    if( !meth.compare("GET") ) {
#ifdef _DEBUG_HTTP_SERVER_H
        printf("dispatchRequest HTTP_GET\r\n");
#endif
        methCode = HTTP_GET;
    } else if( !meth.compare("POST") ) {
#ifdef _DEBUG_HTTP_SERVER_H
        printf("dispatchRequest HTTP_POST\r\n");
#endif
        methCode = HTTP_POST;
    } else if( !meth.compare("HEAD") ) {
#ifdef _DEBUG_HTTP_SERVER_H
        printf("dispatchRequest HTTP_HEAD\r\n");
#endif
        methCode = HTTP_HEAD;
    } else {
#ifdef _DEBUG_HTTP_SERVER_H
        printf("dispatchRequest() Parse error\r\n");
#endif
        //close(); //Parse error
        return;
    }
#ifdef _DEBUG_HTTP_SERVER_H
    printf("Looking for a handler\r\n");
#endif
    map< string, HTTPRequestHandler*(*)(const char*, const char*, TCPSocketConnection*), handlersComp >::iterator it;
    int root_len = 0;
    for (it = m_lpHandlers.begin(); it != m_lpHandlers.end(); it++) {
#ifdef _DEBUG_HTTP_SERVER_H
        printf("Checking %s...\r\n", (*it).first.c_str());
#endif
        root_len = (*it).first.length();
        if ( root_len &&
                !path.compare( 0, root_len, (*it).first ) &&
                (path[root_len] == '/' || path[root_len] == '\0')) {
#ifdef _DEBUG_HTTP_SERVER_H
            printf("Found (%s)\r\n", (*it).first.c_str());
#endif
            // Found!
            break;  // for
        }
    }
    if((it == m_lpHandlers.end()) && !(m_lpHandlers.empty())) {
#ifdef _DEBUG_HTTP_SERVER_H
        printf("Using default handler\r\n");
#endif
        it = m_lpHandlers.end();
        it--; //Get the last element
        if( ! (((*it).first.length() == 0) || !(*it).first.compare("/")) ) //This is not the default handler
            it = m_lpHandlers.end();
        root_len = 0;
    }
    if(it == m_lpHandlers.end()) {
#ifdef _DEBUG_HTTP_SERVER_H
        printf("No handler found\r\n");
#endif
        return;
    }
#ifdef _DEBUG_HTTP_SERVER_H
    printf("Handler found.\r\n");
#endif
    HTTPRequestHandler* pHdlr = (*it).second((*it).first.c_str(), path.c_str() + root_len, client);
    //****  client = NULL; //We don't own it anymore
    switch(methCode) {
        case HTTP_GET:
            pHdlr->doGet();
            break;
        case HTTP_POST:
            pHdlr->doPost();
            break;
        case HTTP_HEAD:
            pHdlr->doHead();
            break;
    }
    delete pHdlr;
    // delete client;
    // delete m_pTCPSocketConnection;
#ifdef _DEBUG_HTTP_SERVER_H
    printf("(dispatcherRequest)return\r\n");
#endif
    return ;
}

void HTTPServerChild (void const *arg)
{
#ifdef _DEBUG_HTTP_SERVER_H
    printf("HTTPServerChiled Start......\r\n");
#endif
    TCPSocketConnection* client = (TCPSocketConnection*)arg;

    for (;;) {
#ifdef _DEBUG_HTTP_SERVER_H
        printf("(HTTPServer.h<HTTPServerChild>)Connection from %s\r\n", client->get_address());
#endif
        dispatchRequest(client);
#ifdef _DEBUG_HTTP_SERVER_H
        printf("(HTTPServer.h<HTTPServerChild>)Close %s\r\n", client->get_address());
#endif
        client->close();
        client->reset_address();
		//delete client;
        //Thread::signal_wait(1);
		slp_tsk();
    }
}

void HTTPServerCloser (void const *arg)
{
    TCPSocketConnection *client = (TCPSocketConnection*)arg;

    for (;;) {
        client->close();
#ifdef _DEBUG_HTTP_SERVER_H
        printf("Close %s\r\n", client->get_address());
#endif
		// Thread::signal_wait(1);
		slp_tsk();
    }
}

void HTTPServerStart(int port = 80)
{
    int i, t = 0;
    TCPSocketConnection clients[THREAD_MAX];
    TCPSocketConnection xclient;
	T_CTSK ctsk;
	T_RTSK pk_rtsk;
	
    for (i = 0; i < THREAD_MAX; i++) {
		//        threads[i] = NULL;
		threads_id[i] = 0;
    }
    xthread_id = 0;
	
    TCPSocketServer server;
    server.bind(port);
    server.listen();
    // server.set_blocking(false);
#ifdef _DEBUG_HTTP_SERVER_H
    printf("Wait for new connection...\r\n");
#endif
    for (;;) {
#ifdef _DEBUG_HTTP_SERVER_H
	printf("**Start Loop** \r\n");
#endif
     	if(t >= 0) {
            if(server.accept(clients[t]) == 0) {
	            syslog(LOG_NOTICE, "server.accept(clients[%d]) passed.", t);
                // fork child process
				//                if (threads[t]) {
				if (threads_id[t] != 0) {
					//                    threads[t]->signal_set(1);
					syslog(LOG_NOTICE, "wake up task with ID=%d", threads_id[t]);
					wup_tsk(threads_id[t]);
                } else {
		//  threads[t] = new Thread(HTTPServerChild, (void*)&clients[t], osPriorityNormal, 1024 * 3, NULL);
					ctsk.tskatr = TA_ACT;
					ctsk.exinf = (intptr_t)&clients[t];
					ctsk.task = (TASK)HTTPServerChild;
					ctsk.itskpri = (PRI)osPriorityNormal;
					ctsk.stksz = 1024 * 3;
					ctsk.stk = NULL;					
					threads_id[t] = acre_tsk(&ctsk);
                }
#ifdef _DEBUG_HTTP_SERVER_H
                printf("Forked %d\r\n", t);
#endif
            }
        } else {
            if(server.accept(xclient) == 0) {
	            syslog(LOG_NOTICE, "server.accept(xclient) passed.");	
                // closer process
				//                if (xthread) {
				if (xthread_id != 0) {
                    // xthread->signal_set(1);
					syslog(LOG_NOTICE, "wake up task with ID=%d", xthread_id);
					wup_tsk(xthread_id);
                } else {
                    //xthread = new Thread(HTTPServerCloser, (void*)&xclient);
					ctsk.tskatr = TA_ACT;
					ctsk.exinf = (intptr_t)(&xclient);
					ctsk.task = (TASK)HTTPServerCloser;
					ctsk.itskpri = (PRI)osPriorityNormal;
					ctsk.stksz = 1024;
					ctsk.stk = NULL;					
					xthread_id = acre_tsk(&ctsk);
                }
#ifdef _DEBUG_HTTP_SERVER_H
                printf("Connection full\r\n");
#endif
            }
        }

        t = -1;
		
        for (i = 0; i < THREAD_MAX; i++) {
			if (threads_id[i] != 0) {
				ref_tsk(threads_id[i], &pk_rtsk);
			}

			//            if ((threads[i] == NULL)
			//             || ((threads[i]->get_state() == Thread::WaitingAnd) && (*clients[i].get_address() == 0))) {
			if ((threads_id[i] == 0) ||
			((pk_rtsk.tskstat == TTS_WAI && pk_rtsk.tskwait == TTW_SLP) &&
			(*clients[i].get_address() == 0))) {
                if (t < 0) t = i; // next empty thread
            }
        }
        // Thread::wait(100);
		dly_tsk(100);
        rot_rdq(osPriorityNormal);
    }
}
#include "Handler/RPCHandler.h"
#include "Handler/FSHandler.h"
#include "Handler/SimpleHandler.h"
#include "SnapshotHandler.h"

#endif
