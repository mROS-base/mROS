#include "ros_emb.h"

string addHttpPost(string xml){
	//なんか汚いし効率悪そうだから変更の余地あり
	stringstream ss;
	ss <<  xml.size();
	string xml2;
	xml2 += "POST /RPC2 HTTP/1.1\n";
	xml2 += "Host: \n";
	xml2 += "Accept-Encoding: \n";
	xml2 += "User-Agent: \n";
	xml2 += "Content-Type: \n";
	xml2 += "Content-Length: ";
	xml2 += ss.str();
	xml2 += "\n\n";
	xml2 += xml;
	return xml2;
}

string addHttpOK(string xml){
	stringstream ss;
		ss <<  xml.size();
		string xml2;
		xml2 += "HTTP/1.1 200 OK\n";
		xml2 += "Host: \n";
		xml2 += "Accept-Encoding: \n";
		xml2 += "User-Agent: \n";
		xml2 += "Content-Type: \n";
		xml2 += "Content-Length: ";
		xml2 += ss.str();
		xml2 += "\n\n";
		xml2 += xml;
		return xml2;
}


string  makexmlcall(string name,vector<string> params,int pnum){
    string m;
    m += "<?xml version='1.0'?>\n";
    m += "<methodCall>\n";
    m += "<methodName>";
    m += name;
    m += "</methodName>\n";
    m += "<params>\n";
    for(int i=0; i < pnum;i++){
        m += "<param>\n<value>";
        m += params[i];
        m += "</value>\n</param>\n";
    }
    m += "</params>";
    m += "</methodCall>";
    return m;
}

string  unregisterService(string id,string srv,string s_uri,string c_uri){
    string xml;
    vector<string> params;
    params.push_back(id);
    params.push_back(srv);
    params.push_back(s_uri);
    params.push_back(c_uri);
    xml = makexmlcall("unregisterService",params,4);
    xml = addHttpPost(xml);
    return xml;
}
string  registerService(string id,string srv,string c_uri){
    string xml;
    vector<string> params;
    params.push_back(id);
    params.push_back(srv);
    params.push_back(c_uri);
    xml = makexmlcall("registerService",params,3);
    xml = addHttpPost(xml);
    return xml;

}
string  unregisterSubscriber(string id,string topic,string c_uri){
    string xml;
    vector<string> params;
    params.push_back(id);
    params.push_back(topic);
    params.push_back(c_uri);
    xml = makexmlcall("unregisterSubscriber",params,3);
    xml = addHttpPost(xml);
    return xml;

}
string  registerSubscriber(string id,string topic,string type,string c_uri){
    string xml;
    vector<string> params;
    params.push_back(id);
    params.push_back(topic);
    params.push_back(type);
    params.push_back(c_uri);
    xml = makexmlcall("registerSubscriber",params,4);
    xml = addHttpPost(xml);
    return xml;
}
string  unregisterPublisher(string id,string topic,string c_uri){
    string xml;
    vector<string> params;
    params.push_back(id);
    params.push_back(topic);
    params.push_back(c_uri);
    xml = makexmlcall("unregisterPublisher",params,3);
    xml = addHttpPost(xml);
    return xml;
}
string  registerPublisher(string id,string topic,string type,string c_uri){
    string xml;
    vector<string> params;
    params.push_back(id);
    params.push_back(topic);
    params.push_back(type);
    params.push_back(c_uri);
    xml = makexmlcall("registerPublisher",params,4);
    xml = addHttpPost(xml);
    return xml;
}

string requestTopic(string id,string topic,string prt="TCPROS"){
    string xml;
    string tcpros;
    vector<string>  params;
    tcpros += "<array>\n<data><value><array>\n<data><value>";
    tcpros += prt;
    tcpros += "</value></data>\n</array></value></data>\n</array>";
    params.push_back(id);
    params.push_back(topic);
    params.push_back(tcpros);
    xml = makexmlcall("requestTopic",params,3);
    xml = addHttpPost(xml);
    return xml;
}

//test用methodResponse生成
string test_requestResponse(string ip){
    string res;
    string xml;
    xml += "<?xml version='1.0'?>\n";
    xml += "<methodResponse>\n";
    xml += "<params>\n";
    xml += "<param>\n";
    xml += "<value><array><data><value><i4>1</4></value>\n";
    xml += "<value></value>\n";
    xml += "<value><array>\n";
    xml += "<data><value>TCPROS</value>\n";
    xml += "<value>";
    xml += ip;
    xml += "</value>\n";
    xml += "<value><i4>11511</i4></value>\n</data>";
    xml += "</array></value>\n";
    xml += "</data></array></value>";
    xml += "</param>\n";
    xml += "</params>\n";
    xml += "</methodResponse>\n";
    res = addHttpOK(xml);
    //syslog(LOG_NOTICE,"LOG_INFO: xml [%s]",xml.c_str());
    return res;
}


string registerSubtask(string func,string port){
	string body;
	body += "<fptr>";
	body += func;
	body += "</fptr>";
	body += "<port>";
	body += port;
	body += "</port>";
	return body;
}
