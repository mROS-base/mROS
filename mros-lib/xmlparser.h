#ifndef _XMLPARSER_H_
#define _XMLPARSER_H_

#include <string>
#include <iostream>
#include <vector>


using namespace std;


typedef struct xmlNode{
    string  methodName;
    vector<string> params;    
    int e_num;  
    bool fault;
    xmlNode(){
        e_num=0;
        fault = true;
    }
}xmlNode;


string getbool(string param);
string getint(string param);
string getdouble(string param);
string getstring(string param);
string getarray(string param);
string getdate(string param);
string getbinary(string param);
string getstruct(string param);
string gettagless(string param);

void paramparser(xmlNode *node,string params,int num);
void faultperser(xmlNode *node,string body);

void callparser(xmlNode *node,string xml);
void resparser(xmlNode *node,string xml);

bool parser(xmlNode *node,string xml);
int ParseReceiveMessage(string http,xmlNode *node);
int get_port(string http);
string get_port2(string http);
string get_ip(string ip);


string get_ttype(string xml);
string get_tname(string xml);
string get_cid(string xml);
string get_msgdef(string xml);
string get_fptr(string xml);
string req_topic_name(string xml);
#endif /* _XMLPARSER_H_ */  
