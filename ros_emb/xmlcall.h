#ifndef _XMLCALL_H_
#define _XMLCALL_H_


#include <string>
#include <sstream>

#include <stdint.h>
#include <stdio.h>
#include <vector>
#include <stdlib.h>

using namespace std;

//master API
/*
@param
    id => caller_id(caller name)
    srv => service name
    s_uri => service URI
    c_uri => caller URI
    topic => topic name
    type => topic type
*/
string  unregisterService(string id,string srv,string s_uri,string c_uri);
string  registerService(string id,string srv,string s_uri);
string  unregisterSubscriber(string id,string topic,string c_uri);
string  registerSubscriber(string id,string topic,string type,string c_uri);
string  unregisterPublisher(string id,string topic,string c_uri);
string  registerPublisher(string id,string topic,string type,string c_uri);



//slave API response make
/*
@param
    prt => communication protocol
            TCP?[[str,!XMLRPCLeagalValue*]]
*/
string  requestTopic(string id,string topic,string prt="TCPROS");
string  test_requestResponse(); //テスト用XML-RPCレスポンス生成関数
//現状prt = "TCPROS"のみを考える．引数も特になし

//local function
string	addHttpPost(string xml);
string	addHttpOK(string xml);
string  makexmlcall(string name,vector<string> params,int pnum); 

//タスク用
string registerSubtask(string func,string port);

#endif
