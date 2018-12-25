#include "mros.h"

/* XMLパーサ 使ってないので作ってください
#define SUCCESS_PARSING 1
#define HTTP_OK 2
#define NON_POST_METHOD -2
#define METHOD_ERROR -3
#define FAIL_TO_PARSE_XML -4

int err_status;


string getbool(string param){
    string val;
    for(int i = param.find("<boolean>") + sizeof("<boolean>") - 1 ; i < param.rfind("</boolean>") ; i++){
            val = val + param[i];
    }
    return val;
} 

string getint(string param){
    string val;
    if(param.find("<int>") != -1){
        for(int i = param.find("<int>") + sizeof("<int>") -1 ; i < param.rfind("</int>") ; i++){
            val = val + param[i];
        }
    }else if(param.find("<i4>") != -1){
        for(int i = param.find("<i4>") + sizeof("<i4>") -1 ; i < param.rfind("</i4>") ; i++){
        val = val + param[i];
        }
    }
    return val;
}

string getdouble(string param){
    string val;
    for(int i = param.find("<double>") + sizeof("<double>") -1 ; i < param.rfind("</double>") ; i++){
        val = val + param[i];
    }
    return val;
}

string getstring(string param){
    string val;
    for(int i = param.find("<string>") + sizeof("<string>") -1 ; i < param.rfind("</string>") ; i++){
        val = val + param[i];
    }
    return val;
    //vec->push_back(val);
    //cout << "can't reach here" << endl; 
}
string getarray(string param){
    string val;
    for(int i = param.find("<array>") + sizeof("<array>") - 1 ; i < param.rfind("</array>") ; i++){
        if(param[i] != ' '){
            val = val + param[i];
        }
    }
    return val;
}

string getdate(string param){
    string val;
    for(int i = param.find("<dateTime.iso8601>") + sizeof("<dateTime.iso8601>") -1 ; i < param.rfind("</dateTime.iso8601>") ; i++){
        val = val + param[i];
    }
    return val;
}

string getbinary(string param){
    string val;
    for(int i = param.find("<base64>") + sizeof("<base64>") -1 ; i < param.rfind("</base64>") ; i++){
        val = val + param[i];
    }
    return val;
}

string getstruct(string param){
    string val;
    for(int i = param.find("<struct>") + sizeof("<struct>") - 1 ; i < param.rfind("</struct>") ; i++){
        if(param[i] != ' '){
            val = val + param[i];
        }
    }
    return val;
}

string gettagless(string param){
    string val;
    for(int i = param.find("<value>") + sizeof("<value>") -1 ; i < param.find("</value") ; i++){
        val = val + param[i];
    }
    return val;
}


void valparse(xmlNode *node,string param){
    if((param.find("<array>") != -1 && (param.find("<array>") < param.find("<struct>"))) || (param.find("<struct>") == -1)){ 
        node->params.push_back(getarray(param));

    }else if(param.find("<struct>") != -1){
        node->params.push_back(getstruct(param));

    }else if(param.find("<string>") != -1){
        //cout << "get string!!" << endl;

        node->params.push_back(getstring(param));
    }else if(param.find("<int>") != -1 || param.find("<i4>") != -1){

        node->params.push_back(getint(param));
    }else if(param.find("<boolean>") != -1){

        node->params.push_back(getbool(param));

    }else if(param.find("<double>") != -1){

        node->params.push_back(getdouble(param));

    }else if(param.find("<dateTime.iso8601>") != -1){
       
        node->params.push_back(getdate(param));

    }else if(param.find("<base64>") != -1){

        node->params.push_back(getbinary(param));

    }else{
        //tagless string 
        node->params.push_back(gettagless(param));
    }
}

void paramparser(xmlNode *node,string params){
    string param;
    int phead,ptail;
    while((int)params.find("<param>") != -1){
        param = "";
        phead = (int)params.find("<param>");
        ptail = (int)params.find("</param>");
        if(phead == -1 || ptail == -1){
            err_status = FAIL_TO_PARSE_XML;
            return;
        }
        for(int i = phead + sizeof("<params>") - 1 ; i < ptail ; i ++ ){
            param = param + params[i];
        }
        node->e_num++;
        valparse(node,param);
        params = params.erase(phead,ptail);
    }
    //cout << "param parser end" << endl;
}

void faultparser(xmlNode *node,string body){
    node->params.push_back(getint(body));
    node->params.push_back(getstring(body));
}

void callparser(xmlNode *node,string xml){
    string  m_call;
    string  m_name;
    string  params;
    
    node->fault = true;
    int head = (int)xml.find("<methodCall>");
    int tail = (int)xml.find("</methodCall>");
    if(head == -1 || tail == -1){
        err_status = FAIL_TO_PARSE_XML;
        return;
    }
    for(int i = head + sizeof("<methodCall>") - 1 ; i < tail ; i ++ ){
        m_call = m_call + xml[i];
    }

    int mhead = (int)m_call.find("<methodName>");
    int mtail = (int)m_call.find("</methodName>");
    if(mhead == -1 || mtail == -1){
        err_status = FAIL_TO_PARSE_XML;
        return;
    }
    for(int i = mhead + sizeof("<methodName>") - 1 ; i < mtail ; i ++ ){
        m_name = m_name + m_call[i];
    }
    
    node->methodName = m_name;

    int phead = (int)m_call.find("<params>");
    int ptail = (int)m_call.find("</params>");
    if(phead == -1 || ptail == -1){
        err_status = FAIL_TO_PARSE_XML;
        return;
    }
    for(int i = phead + sizeof("<params>") - 1 ; i < ptail ; i ++ ){
        params = params + m_call[i];
    }
    
    paramparser(node,params);
}

void resparser(xmlNode *node,string xml){
    string resp;
    string body;
    string params;
    //Response
    int head = (int)xml.find("<methodResponse>");
    int tail = (int)xml.find("</methodResponse>");
    if(head == -1 || tail == -1){
        err_status = FAIL_TO_PARSE_XML;
        return;
    }else{
        for(int i = head + sizeof("<methodResponse>") - 1 ; i < tail ; i ++ ){
            body = body + xml[i];
        }
        node->methodName = "Response";
        if(body.find("<fault>") != -1){
            int fhead = (int)body.find("<fault>");
            int ftail = (int)body.find("</fault>");
            if(fhead != -1 && ftail != -1){
                for(int i = fhead + sizeof("<fault>") - 1; i < ftail ; i ++ ){
                    params = params + body[i];
                }
                node->fault = false;
                faultparser(node,params);
            }else{
                err_status = FAIL_TO_PARSE_XML;
                return;
            }
        }else{
            int phead = (int)body.find("<params>");
            int ptail = (int)body.rfind("</params>");
            if(phead != -1 && ptail != -1){
                for(int i = phead + sizeof("<params>") - 1 ; i < ptail ; i ++ ){
                    params = params + body[i];
                }
                node->fault = true;
                paramparser(node,params);
            }else{
                err_status = FAIL_TO_PARSE_XML;
                return;
            }
        }
    }
}

bool parser(xmlNode *node,string xml){
    if(xml.find("<methodCall>") != -1 && xml.find("</methodCall>") != -1){
        callparser(node,xml);
    }else if(xml.find("<methodResponse>") != -1 && xml.find("</methodResponse>") != -1){
        resparser(node,xml);
    }else{
        cout << "invalid XML-RPC !" << endl;
        err_status = METHOD_ERROR;
        return false;
    }
    return true;
}


int ParseReceiveMessage(string http,xmlNode *node){
	err_status = SUCCESS_PARSING;
    if((int)http.find("POST") != -1){
        if(!parser(node,http)){  
        }else{
            err_status = SUCCESS_PARSING;
        }
    }else if((int)http.find("HTTP/1.0 200 OK") != -1){
        if(!parser(node,http)){
        }else{
            err_status = HTTP_OK;
        }
    }else{
        err_status = NON_POST_METHOD;
    }
    cout << "ERROR_STATUS: " << err_status << endl;
    return err_status;
}
*/

int get_port(string http){
	string val;
	int head = (int)http.find(":",http.find("http:")+6);
	int tail = (int)http.find("/",head);
	for(int i = head + 1; i < tail ; i++){
        val = val + http[i];
    }
    return atoi(val.c_str());
}

string get_port2(string http){
	string val;
	int head = (int)http.find("<i4>",http.find("TCPROS"));
	int tail = (int)http.find("</i4>",head + sizeof("<i4>") -1);
    for(int i = head + sizeof("<i4>") -1 ; i < tail ; i++){
        val = val + http[i];
    }

    return val;
}

string get_ip(string xml){
	int head,tail;
			string body;
			head = (int)xml.find("<string>http://");
			tail = (int)xml.find(":",head+sizeof("<string>http://"));
			for(int i = head + sizeof("<string>http://")-1;i < tail; i++){
				body = body + xml[i];
			}
			return body;
}


string get_ttype(string xml){
	int head,tail;
	string body;
	head = (int)xml.find("<topic_type>");
	tail = (int)xml.find("</topic_type>");
	for(int i = head + sizeof("<topic_type>") -1;i < tail; i++){
		body = body + xml[i];
	}
	return body;
}

string get_tname(string xml){
	int head,tail;
		string body;
		head = (int)xml.find("<topic_name>");
		tail = (int)xml.find("</topic_name>");
		for(int i = head + sizeof("<topic_name>")-1;i < tail; i++){
			body = body + xml[i];
		}
		return body;
}

string get_cid(string xml){
	int head,tail;
		string body;
		head = (int)xml.find("<caller_id>");
		tail = (int)xml.find("</caller_id>");
		for(int i = head + sizeof("<caller_id>")-1;i < tail; i++){
			body = body + xml[i];
		}
		return body;
}

string get_msgdef(string xml){
	int head,tail;
		string body;
		head = (int)xml.find("<message_definition>");
		tail = (int)xml.find("</message_definition>");
		for(int i = head + sizeof("<message_definition>")-1;i < tail; i++){
			body = body + xml[i];
		}
		return body;
}

string get_fptr(string xml){
	int head,tail;
		string body;
		head = (int)xml.find("<fptr>");
		tail = (int)xml.find("</fptr>");
		for(int i = head + sizeof("<fptr>")-1;i < tail; i++){
			body = body + xml[i];
		}
		return body;
}

string req_topic_name(string xml){
	int ini,head,tail;
				string body;
				ini = (int)xml.find("<value>");
				head = (int)xml.find("<value>",ini + sizeof("<value><string>"));
				tail = (int)xml.find("</value>",head);
				for(int i = head + sizeof("<value>")-1;i < tail; i++){
					body = body + xml[i];
				}
				return body;
}


