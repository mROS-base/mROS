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
//#define _DEBUG_FS_HANDLER

#include "FSHandler.h"

#include "kernel_cfg.h"

#define CHUNK_SIZE 128

#define DEFAULT_PAGE "/index.htm"

//Semaphore FSHandler::req_sem(1);


//*FSHandler::FSHandler(const char* rootPath, const char* path, TCPSocket* pTCPSocket) : HTTPRequestHandler(rootPath, path, pTCPSocket), m_err404(false)
FSHandler::FSHandler(const char* rootPath, const char* path, TCPSocketConnection* pTCPSocketConnection) : HTTPRequestHandler(rootPath, path, pTCPSocketConnection), m_err404(false)
{}

FSHandler::~FSHandler()
{
#ifdef _DEBUG_FS_HANDLER
    printf("\r\n+++FSHandler destroy\r\n");
#endif
    if(m_fp){
#ifdef _DEBUG_FS_HANDLER
    printf("\r\n+++FSHandler fclose start\r\n");
#endif
        fclose(m_fp);
#ifdef _DEBUG_FS_HANDLER
    printf("\r\n+++FSHandler fclose end\r\n");
#endif
       
        }
#ifdef _DEBUG_FS_HANDLER
    printf("\r\nHandler destroyed\r\n");
#endif
}

//static init
map<string,string> FSHandler::m_lFsPath = map<string,string>();

void FSHandler::mount(const string& fsPath, const string& rootPath)
{
    m_lFsPath[rootPath]=fsPath;
}

void FSHandler::doGet()
{
	//    req_sem.wait();
	wai_sem(REQ_SEM);
#ifdef _DEBUG_FS_HANDLER
    printf("\r\nIn FSHandler::doGet() - rootPath=%s, path=%s\r\n", rootPath().c_str(), path().c_str());
#endif
    //FIXME: Translate path to local/path
    string checkedRootPath = rootPath();
    if(checkedRootPath.empty())
        checkedRootPath="/";
    string filePath = m_lFsPath[checkedRootPath];
    if (path().size() > 1) {
        filePath += path();
    } else {
        filePath += DEFAULT_PAGE;
    }
#ifdef _DEBUG_FS_HANDLER
    printf("Trying to open %s\n", filePath.c_str());
#endif
    m_fp = fopen(filePath.c_str(), "r"); //FIXME: if null, error 404

    if(!m_fp) {
        m_err404 = true;
        setErrCode(404);
        const char* msg = "File not found.";
        setContentLen(strlen(msg));
        respHeaders()["Content-Type"] = "text/html";
        respHeaders()["Connection"] = "close";
        writeData(msg,strlen(msg)); //Only send header
        printf("\r\nExit FSHandler::doGet() w Error 404\r\n");
		//        req_sem.release();
		sig_sem(REQ_SEM);
        return;
    }

    //Seek EOF to get length
    fseek(m_fp, 0, SEEK_END);
    setContentLen( ftell(m_fp) );
    fseek(m_fp, 0, SEEK_SET); //Goto SOF

    respHeaders()["Connection"] = "close";
    onWriteable();
#ifdef _DEBUG_FS_HANDLER
    printf("\r\nExit SimpleHandler::doGet()\r\n");
#endif
    // req_sem.release();
	sig_sem(REQ_SEM);
}

void FSHandler::doPost()
{

}

void FSHandler::doHead()
{

}

void FSHandler::onReadable() //Data has been read
{

}

void FSHandler::onWriteable() //Data has been written & buf is free
{
#ifdef _DEBUG_FS_HANDLER
    printf("\r\nFSHandler::onWriteable() event\r\n");
#endif
    if(m_err404) {
        //Error has been served, now exit
        close();
        return;
    }

    static char rBuf[CHUNK_SIZE];
    while(true) {
        int len = fread(rBuf, 1, CHUNK_SIZE, m_fp);
        if(len>0) {
            int writtenLen = writeData(rBuf, len);
            if(writtenLen < 0) { //Socket error
#ifdef _DEBUG_FS_HANDLER
                printf("FSHandler: Socket error %d\n", writtenLen);
#endif
                /**  Not Work
                                if(writtenLen == TCPSOCKET_MEM) {
                                    fseek(m_fp, -len, SEEK_CUR);
                                    return; //Wait for the queued TCP segments to be transmitted
                                } else {
                                    //This is a critical error
                                    **/
                                    close();
                                    return;
                                    /**
                                }
                **/
            } else if(writtenLen < len) { //Short write, socket's buffer is full
                fseek(m_fp, writtenLen - len, SEEK_CUR);
                return;
            }
        } else {
            close(); //Data written, we can close the connection
            return;
        }
    }
}

void FSHandler::onClose() //Connection is closing
{
    /**
#ifdef _DEBUG_FS_HANDLER
        printf("FSHandler: onClose start \r\n");
#endif
    if(m_fp){
#ifdef _DEBUG_FS_HANDLER
        printf("FSHandler: fclose start \r\n");
#endif
    fclose(m_fp);
#ifdef _DEBUG_FS_HANDLER
    printf("FSHandler: fclose end \r\n");
#endif
}
#ifdef _DEBUG_FS_HANDLER
        printf("FSHandler: onClose end \r\n");
#endif
**/
}
