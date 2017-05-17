/*******************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only
* intended for use with Renesas products. No other uses are authorized. This
* software is owned by Renesas Electronics Corporation and is protected under
* all applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT
* LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
* AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR
* ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE
* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software
* and to discontinue the availability of this software. By using this software,
* you agree to the additional terms and conditions found by accessing the
* following link:
* http://www.renesas.com/disclaimer*
* Copyright (C) 2015 Renesas Electronics Corporation. All rights reserved.
*******************************************************************************/

#ifndef SNAPSHOT_HANDLER_H
#define SNAPSHOT_HANDLER_H

#include "HTTPRequestHandler.h"

class SnapshotHandler : public HTTPRequestHandler
{
public:
    SnapshotHandler(const char* rootPath, const char* path, TCPSocketConnection* pTCPSocketConnection);
    virtual ~SnapshotHandler();

    static void attach_req(int(*fptr)(const char ** pp_data)) {
        callback_func_req = fptr;
    }

//protected:
    static inline HTTPRequestHandler* inst(const char* rootPath, const char* path, TCPSocketConnection* pTCPSocketConnection) { return new SnapshotHandler(rootPath, path, pTCPSocketConnection); } //if we ever could do static virtual functions, this would be one

    virtual void doGet();
    virtual void doPost();
    virtual void doHead();

    virtual void onReadable(); //Data has been read
    virtual void onWriteable(); //Data has been written & buf is free
    virtual void onClose(); //Connection is closing

private:
    static int (*callback_func_req)(const char ** pp_data);
    // static Semaphore req_sem;
    const char * send_data_buf;
    int send_size;
    int send_index;
    bool m_err404;
};

#endif
