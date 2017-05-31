#include "xmlcall.h"


string  makexmlcall(string name,vector<string> params,int pnum){
    string m;
    m += "<methodCall>\n";
    m += "<methodName>";
    m += name;
    m += "</methodName>\n";
    m += "<params>\n";
    for(int i=0; i < pnum;i++){
        m += "<param><value>";
        m += params[i];
        m += "</value></param>\n";
    }
    m += "</params>";
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
    return xml;
}
string  registerService(string id,string srv,string c_uri){
    string xml;
    vector<string> params;
    params.push_back(id);
    params.push_back(srv);
    params.push_back(c_uri);
    xml = makexmlcall("registerService",params,3);
    return xml;

}
string  unregisterSubscriber(string id,string topic,string c_uri){
    string xml;
    vector<string> params;
    params.push_back(id);
    params.push_back(topic);
    params.push_back(c_uri);
    xml = makexmlcall("unregisterSubscriver",params,3);
    return xml;

}
string  registerSubscriber(string id,string topic,string type,string c_uri){
    string xml;
    vector<string> params;
    params.push_back(id);
    params.push_back(topic);
    params.push_back(type);
    params.push_back(c_uri);
    xml = makexmlcall("registerSubscriver",params,4);
    return xml;
}
string  unregisterPublisher(string id,string topic,string c_uri){
    string xml;
    vector<string> params;
    params.push_back(id);
    params.push_back(topic);
    params.push_back(c_uri);
    xml = makexmlcall("unregisterPublisher",params,3);
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
    return xml;
}
