#include "xmlcall.h"

string addHttpheader(string xml){
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


string  makexmlcall(string name,vector<string> params,int pnum){
    string m;
    m += "<?xml version='1.0'?>\n";
    m += "<methodCall>\n";
    m += "<methodName>";
    m += name;
    m += "</methodName>\n";
    m += "<params>\n";
    for(int i=0; i < pnum;i++){
        m += "<param>\n<value><string>";
        m += params[i];
        m += "</string></value>\n</param>\n";
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
    xml = addHttpheader(xml);
    return xml;
}
string  registerService(string id,string srv,string c_uri){
    string xml;
    vector<string> params;
    params.push_back(id);
    params.push_back(srv);
    params.push_back(c_uri);
    xml = makexmlcall("registerService",params,3);
    xml = addHttpheader(xml);
    return xml;

}
string  unregisterSubscriber(string id,string topic,string c_uri){
    string xml;
    vector<string> params;
    params.push_back(id);
    params.push_back(topic);
    params.push_back(c_uri);
    xml = makexmlcall("unregisterSubscriber",params,3);
    xml = addHttpheader(xml);
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
    xml = addHttpheader(xml);
    return xml;
}
string  unregisterPublisher(string id,string topic,string c_uri){
    string xml;
    vector<string> params;
    params.push_back(id);
    params.push_back(topic);
    params.push_back(c_uri);
    xml = makexmlcall("unregisterPublisher",params,3);
    xml = addHttpheader(xml);
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
    xml = addHttpheader(xml);
    return xml;
}

string requestTopic(string id,string topic,string prt){
    string xml;
    string tcpros;
    vector<string>  params;
    tcpros += "<array>\n<data><value><array>\n<data><value>";
    tcpros += prt;
    tcpros += "</value></data>\n</array></value></data>\n</array>";
    params.push_back(id);
    params.push_back(topic);
    params.push_back(tcpros);
    xml = makexmlcall("registerPublisher",params,3);
    xml = addHttpheader(xml);
    return xml;
}
