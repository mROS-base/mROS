#include "xmlparser.h"

string getbool(string param){
    string val;
    for(int i = param.find("<boolean>") + sizeof("<boolean>") - 1 ; i < param.find("</boolean>") ; i++){
            val = val + param[i];
    }
    return val;
} 

string getint(string param){
    string val;
    if(param.find("<int>") != -1){
        for(int i = param.find("<int>") + sizeof("<int>") -1 ; i < param.find("</int>") ; i++){
            val = val + param[i];
        }
    }else if(param.find("<i4>") != -1){
        for(int i = param.find("<i4>") + sizeof("<i4>") -1 ; i < param.find("</i4>") ; i++){
        val = val + param[i];
        }
    }
    return val;
}

string getdouble(string param){
    string val;
    for(int i = param.find("<double>") + sizeof("<double>") -1 ; i < param.find("</double>") ; i++){
        val = val + param[i];
    }
    return val;
}

string getstring(string param){
    string val;
    for(int i = param.find("<string>") + sizeof("<string>") -1 ; i < param.find("</string>") ; i++){
        val = val + param[i];
    }
    return val;
    //vec->push_back(val);
    //cout << "can't reach here" << endl; 
}

string getarray(string param){
    string val;
    for(int i = param.find("<array>") + sizeof("<array>") - 1 ; i < param.find("</array>") ; i++){
        if(param[i] != ' '){
            val = val + param[i];
        }
    }
    return val;
}

string getdate(string param){
    string val;
    for(int i = param.find("<dateTime.iso8601>") + sizeof("<dateTime.iso8601>") -1 ; i < param.find("</dateTime.iso8601>") ; i++){
        val = val + param[i];
    }
    return val;
}

string getbinary(string param){
    string val;
    for(int i = param.find("<base64>") + sizeof("<base64>") -1 ; i < param.find("</base64>") ; i++){
        val = val + param[i];
    }
    return val;
}

string getstruct(string param){
    string val;
    for(int i = param.find("<struct>") + sizeof("<struct>") - 1 ; i < param.find("</struct>") ; i++){
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
    //cout << "value::" << param << endl;
    //arrayとstructの先に来る方の検出
    if(param.find("<array>") != -1 && (param.find("<array>") < param.find("<struct>") || (param.find("<struct>") == -1))){ 
        cout << "in array " << endl;
        node->params.push_back(getarray(param));
        node->ptype.push_back(5);

    }else if(param.find("<struct>") != -1){
        cout << "in struct " << endl;
        node->params.push_back(getstruct(param));
        node->ptype.push_back(8);

    }else if(param.find("<string>") != -1){
        //cout << "get string!!" << endl;

        node->params.push_back(getstring(param));
        node->ptype.push_back(4);

    }else if(param.find("<int>") != -1 || param.find("<i4>") != -1){

        node->params.push_back(getint(param));
        node->ptype.push_back(2);

    }else if(param.find("<boolean>") != -1){

        node->params.push_back(getbool(param));
        node->ptype.push_back(1);

    }else if(param.find("<double>") != -1){

        node->params.push_back(getdouble(param));
        node->ptype.push_back(3);

    }else if(param.find("<dateTime.iso8601>") != -1){
       
        node->params.push_back(getdate(param));
        node->ptype.push_back(6);

    }else if(param.find("<base64>") != -1){

        node->params.push_back(getbinary(param));
        node->ptype.push_back(7);

    }else{
        //tagless string 
        node->params.push_back(gettagless(param));
        node->ptype.push_back(4);
    }
}

void paramparser(xmlNode *node,string params){
    string param;
    int phead,ptail;
    int elements = 0;
    //<param>がなくなるまで回す
    while((int)params.find("<param>") != -1){
        param = "";
        phead = (int)params.find("<param>");
        ptail = (int)params.find("</param>");
        for(int i = phead + sizeof("<params>") - 1 ; i < ptail ; i ++ ){
            param = param + params[i];
        }
        elements ++;
        //<value></value>の取得
        node->e_num = elements;
        valparse(node,param);
        params = params.erase(phead,ptail);
    }
    //cout << "param parser end" << endl;
}

void faultparser(xmlNode *node,string body){
    //faultCode検出
    node->params.push_back(getint(body));
    node->ptype.push_back(2);
    node->params.push_back(getstring(body));
    node->ptype.push_back(4);
}

//methodCallのパース
void callparser(xmlNode *node,string xml){
    string  m_call;
    string  m_name;
    string  params;
    
    node->fault = true;
    //<methodCall></methodCall>を分割
    int head = (int)xml.find("<methodCall>");
    int tail = (int)xml.find("</methodCall>");
    for(int i = head + sizeof("<methodCall>") - 1 ; i < tail ; i ++ ){
        m_call = m_call + xml[i];
    }

    //<methodName></methodName>の取り出し
    int mhead = (int)m_call.find("<methodName>");
    int mtail = (int)m_call.find("</methodName>");
    for(int i = mhead + sizeof("<methodName>") - 1 ; i < mtail ; i ++ ){
        m_name = m_name + m_call[i];
    }
    
    node->methodName = m_name;

    //<params></params>の取り出し
    int phead = (int)m_call.find("<params>");
    int ptail = (int)m_call.find("</params>");
    for(int i = phead + sizeof("<params>") - 1 ; i < ptail ; i ++ ){
        params = params + m_call[i];
    }
    
    //<param></param>の取り出し
    paramparser(node,params);
}

//methodResponseのパース
void resparser(xmlNode *node,string xml){
    string resp;
    string  body;
    string params;
    //Response
    int head = (int)xml.find("<methodResponse>");
    int tail = (int)xml.find("</methodResponse>");
    for(int i = head + sizeof("<methodResponse>") - 1 ; i < tail ; i ++ ){
        body = body + xml[i];
    }
    node->methodName = "Response";
    //
    if(body.find("<fault>") != -1){
        int fhead = (int)body.find("<fault>");
        int ftail = (int)body.find("</fault>");
        for(int i = fhead + sizeof("<fault>") - 1; i < ftail ; i ++ ){
            params = params + body[i];
        }
        node->fault = false;
        //faultCode,faultStringの取り出し
        faultparser(node,params);
    }else{
        int phead = (int)body.find("<params>");
        int ptail = (int)body.find("</params>");
        for(int i = phead + sizeof("<params>") - 1 ; i < ptail ; i ++ ){
            params = params + body[i];
        }
        node->fault = true;
        //<params>の取り出し
        paramparser(node,params);
    }
}


//XMLのメソッドの判別
bool parser(xmlNode *node,string xml){

    if(xml.find("<methodCall>") != -1){
        callparser(node,xml);
    }else if(xml.find("<methodResponse>") != -1){
        resparser(node,xml);
    }else{
        cout << "invalid XML-RPC !" << endl;
        return false;
    }
    return true;
}
