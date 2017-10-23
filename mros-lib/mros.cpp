#include "mros.h"

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"
#include "mbed.h"
#include "EthernetInterface.h"
#include "syssvc/logtask.h"
#include "md5.h"


/***** congiuration ros master ******/
const char *m_ip = "192.168.11.4";	//ros master IP
const int m_port = 11311;	//ros master xmlrpc port

/*********global variables***************/
/***** shared memory *****/
char mem[1024*1024*2];
extern std::vector<ID> IDv;
int ros_sem =0;	//mROS resource semapho
int count=1;	//for assign node ID



/******* function to operate user task  ******/
void sus_all(){
	for(unsigned int i =0;i < IDv.size();i++){
		sus_tsk(IDv[i]);
		syslog(LOG_NOTICE,"sus task [%d]",IDv[i]);
	}
}

void rsm_all(){
	for(unsigned int i=0;i < IDv.size();i++){
		rsm_tsk(IDv[i]);
		syslog(LOG_NOTICE,"rsm task [%d]",IDv[i]);
	}
	ros_sem = 0;
}

/*****************************************
 * Node information structure for XML-MAS TASK        *
 * ***************************************/
typedef struct node{
		bool node_type;						//true->subscriber false->publisher
		bool stat = true;
		char ID;							//for mROS
		std::string topic_name;				//ROS
		std::string topic_type;				//ROS
		std::string callerid;				//ROS
		std::string message_definition;		//ROS
		std::string uri;					//ROS
		std::string port;					//for sub 
		std::string fptr;					//for taks

public:
		void set_node_type(bool type){this->node_type=type;};
		void init(){this->stat = false;};
		void set_ID(char c){this->ID = c;};
		void set_topic_name(string t){this->topic_name=t;};
		void set_topic_type(string t){this->topic_type=t;};
		void set_callerid(string t){this->callerid=t;};
		void set_message_definition(string t){this->message_definition=t;};
		void set_uri(string t){this->uri=t;};
		void set_port(string t){this->port=t;};
		void set_fptr(string t){this->fptr=t;};
}node;

/******mROS node list***********/
std::vector<node> node_lst;

int find_node(vector<node> list,string topic){for(unsigned int i=0;i < list.size();i++){if(list[i].topic_name == topic){return i;}}return -1;}
int find_id(vector<node> list,char ID){for(unsigned int i=0;i < list.size();i++){if(list[i].ID == ID){return i;}}return -1;}

/***variables for evaluation***/
SYSUTM time1;
SYSUTM time2;

/* Initialize Network Configuration */
#define USE_DHCP (1)
#if(USE_DHCP == 0)
	#define IP_ADDRESS  	("192.168.0.10")	/*IP address */
	#define SUBNET_MASK		("255.0.0.0")	/*Subset mask */
	#define DEFAULT_GATEWAY	("")	/*Default gateway */
#endif

EthernetInterface network;


void network_init(){
#if (USE_DHCP == 1)
	if(network.init() != 0) {
#else
	if(network.init(IP_ADDRESS, SUBNET_MASK, DEFAULT_GATEWAY) != 0) {
#endif
		syslog(LOG_NOTICE, "Network Initialize Error\r\n");
	return;
	}
		syslog(LOG_NOTICE, "Network Initialized successfully");
	while (network.connect(5000) != 0){
		syslog(LOG_NOTICE, "LOG_NOTICE: Network Connect Error");
	}
		syslog(LOG_NOTICE,"MAC Address is %s\r\n", network.getMACAddress());
		syslog(LOG_NOTICE,"IP Address is %s\r\n", network.getIPAddress());
		syslog(LOG_NOTICE,"NetMask is %s\r\n", network.getNetworkMask());
		syslog(LOG_NOTICE,"Gateway Address is %s\r\n", network.getGateway());
}

/*********************************
Main Task
***********************************/

void main_task(){
	
	syslog(LOG_NOTICE, "**********mROS main task start**********");
	syslog(LOG_NOTICE, "LOG_INFO: network initialize...");
	network_init();
	syslog(LOG_NOTICE, "LOG_INFO: SUCCESS INITIALIZATION");
	syslog(LOG_NOTICE, "**********Activate Main task**********");

//activate mROS communication library 
	act_tsk(PUB_TASK);
	act_tsk(SUB_TASK);
	act_tsk(XML_SLV_TASK);
	act_tsk(XML_MAS_TASK);
	act_tsk(USR_TASK2);
	//	syslog(LOG_NOTICE,"**********mROS Main task finish**********");
}


 /******************************************************************************************************************************************************************
 * PUB TASK            ||
  ******************************************************************************************************************************************************************/
void pub_task(){
#ifndef _PUB_
#define _PUB_
syslog(LOG_NOTICE, "========Activate mROS PUBLISH========");
	//node and socket information in PUB_TASK
	struct pub_list{
		public:
			std::vector<TCPSocketConnection> sock_vec;
			std::vector<char> id_vec;
			int find(char c){
				for(int i=0;i < this->id_vec.size();i++){
					if(c == this->id_vec[i]){
						return i;
					}
				}
				return -1;
			}
			void add(TCPSocketConnection sock,char ID){this->sock_vec.push_back(sock);this->id_vec.push_back(ID);} //add pair
			void del(int num){this->sock_vec.erase(this->sock_vec.begin() + num - 1);this->id_vec.erase(this->id_vec.begin() + num - 1);}
	};
	TCPSocketServer srv; 	//for TCPROS connection
	srv.bind(11511);		//port binding
	pub_list lst;
	char *pdqp,*rbuf;
	intptr_t *dqp;	
	dqp = (intptr_t *)malloc(sizeof(char)*4);
	//buf = (char *)malloc(sizeof(char)*1024*50);
	char buf[1024*1024];
	char snd_buf[512];
	rbuf = &buf[8];
#endif 	//_PUB_
	while(1){
		//syslog(LOG_NOTICE,"PUB_TASK:enter loop");
		rcv_dtq(PUB_DTQ,dqp);
		pdqp = (char *)dqp;
		int num = lst.find(pdqp[0]);  
		int node_num = find_id(node_lst,pdqp[0]);			
		int size;
		size = pdqp[1];
		size += pdqp[2]*256;
		size += pdqp[3]*65536;
		//syslog(LOG_NOTICE,"PUB_TASK:operate node [%d]",num);
		if(num == -1){ 									
			//initialization
			syslog(LOG_NOTICE,"PUB_TASK:publisher initialize");
			static TCPSocketConnection sock;
			lst.add(sock,pdqp[0]);
		}else if((num != -1) && (size == 0)){
			//receive request topic
			bool connect_status = false;
			//wait TCPROS connection
			syslog(LOG_NOTICE,"PUB_TASK:request Topic node[%x]",node_lst[node_num].ID);
			srv.listen();
			syslog(LOG_NOTICE,"PUB_TASK:Listening");
			if(srv.accept(lst.sock_vec[num]) == 0){
				syslog(LOG_NOTICE,"PUB_TASK:Conneted");
				connect_status = true;
				while(connect_status){
					//to do: check if data is TCPROS header
					syslog(LOG_NOTICE,"PUB_TASK: TCPROS connection receiving");
					int stat = lst.sock_vec[num].receive(rbuf,512);
					syslog(LOG_NOTICE,"PUB_TASK: TCPROS connection received");
					switch(stat){
					case 0:
					case -1:
						connect_status = false;
						break;
					default:
						if(check_head(rbuf)){
						int len = pub_gen_header(snd_buf,node_lst[node_num].callerid,node_lst[node_num].message_definition,node_lst[node_num].topic_name,node_lst[node_num].topic_type,"992ce8a1687cec8c8bd883ec73ca41d1");	//test function
						//int len = pub_gen_header(snd_buf,"mros_node2","string data\n","mros_msg","std_msgs/String","992ce8a1687cec8c8bd883ec73ca41d1");	//test function
						//syslog(LOG_NOTICE,"PUB_TASK: TCPROS connection header[%s]",snd_buf[8]);
						lst.sock_vec[num].send(snd_buf,len);
						connect_status = false;
						}else{
							syslog(LOG_NOTICE,"PUB_TASK: not TCPROS connection header");
						}
					}
				}
				syslog(LOG_NOTICE,"PUB_TASK: publisher connected");
				rsm_all();
			}
		}else{
		//publish phase
		memcpy(rbuf,mem,size);
		rbuf[size] = '\0';	//cutting data end
		int l = pub_gen_msg(buf,rbuf);
		//publish
		lst.sock_vec[num].send(buf,l);
		}
	}
}

/******************************************************************************************************************************************************************************************************
* SUB TASK        ||
 ************************************************************************************************************************************************************************************/
void sub_task(){
#ifndef _SUB_
#define _SUB_
syslog(LOG_NOTICE, "========Activate mROS SUBSCRIBE========");
	//socket information in SUB_TASK
	struct sub_list{
	public:
		std::vector<TCPSocketConnection> sock_vec; 	//TCP socket
		std::vector<char> id_vec; 					//mROSID
		std::vector<intptr_t> func_vec; 			//callback function pointer
		std::vector<bool> stat_vec;					//state vector
		int find(char c){
					for(int i=0;i < this->id_vec.size();i++){
						if(c == this->id_vec[i]){
							return i;
						}
					}
					return -1;
				}
		void add(TCPSocketConnection sock,char ID,intptr_t func){this->sock_vec.push_back(sock);this->id_vec.push_back(ID);this->func_vec.push_back(func);this->stat_vec.push_back(false);}
		void set_stat(bool stat,int idx){this->stat_vec[idx] = stat;}
	};
	sub_list lst;
	intptr_t *dqp,*snd_dqp;
	dqp = (intptr_t *)malloc(sizeof(char)*4);
	char *sdq,*ip,*rptr; //data queue pointer,IP address,temporary buffer,receive buffer,memory address
	char tmp[256];
	char rbuf[1024*512];
	int port;	//port number
	bool init = false;
	bool rcv_flag = true;
	int len,msg_size,data_size;
#endif //_SUB_

	while(1){
		//syslog(LOG_NOTICE,"SUB_TASK:enter loop");
		slp_tsk();
	    int t = trcv_dtq(SUB_DTQ,dqp,1);	//if queue is empty, go subscribe loop  
	    if(t == 0){
	    	sdq = (char *)dqp;
			int num = lst.find(sdq[0]);
			if(num == -1){
				int node_num = find_id(node_lst,sdq[0]);
	    		int idx = lst.id_vec.size();
	    		//initialize
				syslog(LOG_NOTICE,"SUB_TASK:subscriber initialize");
				static TCPSocketConnection sock;
				static intptr_t funcp;
				int size = sdq[1];
				size += sdq[2]*256;
				size += sdq[3]*65536;
				memcpy(&tmp,&mem[SUB_ADDR],size);
				//get function pointer address
				string str = tmp;
				funcp = (intptr_t)atoi(get_fptr(str).c_str());
				lst.add(sock,sdq[0],funcp);
				syslog(LOG_NOTICE,"SUB_TASK:fptr [%d]",funcp);
				//send requestTopic
				str = "<methodCall>requestTopic</methodCall>";
				size = str.size();
				memcpy(&mem[XML_ADDR],str.c_str(),size);
				char buf[3];
				buf[0] = sdq[0];
				buf[1] = size;
				buf[2] = size/256;
				buf[3] = size/65536;
				snd_dqp = (intptr_t) &buf;
				snd_dtq(XML_DTQ,*snd_dqp);
				rcv_dtq(SUB_DTQ,dqp);
				sdq = (char *)dqp;
				size = sdq[1];
				size += sdq[2]*256;
				size += sdq[3]*65536;
				memcpy(&tmp,&mem[SUB_ADDR],size);				//get date
				str = tmp;
				port = atoi(get_port2(str).c_str());	//test function
				ip = m_ip;
				//syslog(LOG_NOTICE,"SUB_TASK:port [%d],IP [%s]",port,ip);
				//send TCPROS connection header
				size = sub_gen_header(tmp,node_lst[node_num].callerid,"0",node_lst[node_num].topic_name,node_lst[node_num].topic_type,"992ce8a1687cec8c8bd883ec73ca41d1");
				lst.sock_vec[idx].connect(ip,port);
				lst.sock_vec[idx].send(tmp,size);
				wait_ms(500);
				lst.sock_vec[idx].receive(tmp,256);
				lst.set_stat(true,idx);
				init = true;
				syslog(LOG_NOTICE,"SUB_TASK: subscriber connected");
				rsm_all();
			}
	    }

	    //end initialize

//subscribe and callback loop
	    if(init){
		for(unsigned int i=0;i < lst.id_vec.size();i++){
			while(rcv_flag){
				rptr = &rbuf[0];
				if(lst.stat_vec[i]){
					int n = lst.sock_vec[i].receive(rptr,5000);
					if(n < 0){
						syslog(LOG_NOTICE,"SUB_TASK: No data");
					}
					if(rcv_flag){
						msg_size = (int)rbuf[0] + (int)rbuf[1]*256 + rbuf[2]*65536 + rbuf[3]*16777216;
						data_size = (int)rbuf[4] + (int)rbuf[5]*256 + rbuf[6]*65536 + rbuf[7]*16777216;
						rcv_flag = false;
					}
					len += n;
					if(len == msg_size +4){
						//correct data received
						rbuf[len] = '\0';
						syslog(LOG_NOTICE,"SUB_TASK:data length [%d]",len);
						void (*fp)(string);
						fp = lst.func_vec[i];
						fp(&rbuf[8]);
						rptr = &rbuf[0];
						rcv_flag = true;
						len = 0;
					}else if(len > msg_size + 4){
						//data overflow
						syslog(LOG_NOTICE,"invalid header[%d][%d] [%d]",msg_size,len,n);
						rptr = &rbuf[0];
						rcv_flag = true;
						len = 0;
					}else{
					//data receiving
						rptr = &rbuf[n];
					}
				}
			}
		}
	}
	}
}


/******************************************************************************************************************************************************************
* XML-RPC SALAVE TASK ||
 ******************************************************************************************************************************************************************/

void xml_slv_task(){
#ifndef _XML_SLAVE_
#define _XML_SLAVE_
	syslog(LOG_NOTICE,"========Activate mROS XML-RPC Slave========");
	TCPSocketConnection xml_slv_sock;
	TCPSocketServer xml_slv_srv;
	xml_slv_srv.set_blocking(true,1000);	//Don't know blocking or non blocking
	int port = 11411;
	char rbuf[512];
	char data[3];
	string str,meth;
	intptr_t *dqp;
	bool connect_status = false;
	//rbuf = (char *)malloc(sizeof(char)*512);
	if(xml_slv_srv.bind(port) == -1){
			syslog(LOG_NOTICE,"Failed bind");
			exit(1);
		}
#endif

	while(1){
		//syslog(LOG_NOTICE,"XML_SLV_TASK:enter loop");
		slp_tsk();
		xml_slv_srv.listen();
		//syslog(LOG_NOTICE,"XML_SLV_TASK: Listening...");
		//syslog(LOG_NOTICE,"XML_SLV_TASK: Waiting...");
		if(xml_slv_srv.accept(xml_slv_sock) == 0){
			connect_status = true;
			syslog(LOG_NOTICE,"XML_SLV_TASK: Connected");
			while(connect_status){
			int stat = xml_slv_sock.receive(rbuf,512);
			switch(stat){
			case 0:
			case -1:
				connect_status = false;
				//syslog(LOG_NOTICE,"XML_SLV_TASK:disconnected");
				break;
			default:
				str = rbuf;
				int mh,mt;
				//get method
				mh = (int)str.find("<methodName>");
				mt = (int)str.find("</methodName>");
				for(int i = mh + sizeof("<methodName>") -1;i < mt ; i++){
					meth = meth + str[i];
				}
				if(meth == "requestTopic"){
					syslog(LOG_NOTICE,"XML_SLV_TASK:request Topic method");
					string topic_name = req_topic_name(str);
					//syslog(LOG_NOTICE,"XML_SLV_TASK:topic name[%s]",topic_name.c_str());
					int num = find_node(node_lst,topic_name);
					if(num == -1){
						syslog(LOG_NOTICE,"XML_SLV_TASK:No Existing node!");
						//to Do:send fault response
					}else{
						data[2] = 0;
						data[3] = 0;
						data[1] = 0;
						data[0] = node_lst[num].ID;
						dqp = (intptr_t) &data;
						string tmp;
						tmp = network.getIPAddress();
						str = test_requestResponse(tmp);				//test function
						xml_slv_sock.send(str.c_str(),str.size());
						connect_status = false;
						snd_dtq(PUB_DTQ,*dqp);
					}
					xml_slv_sock.close();
					}
				}
			}
		}
	}
}


void get_node(node *node,std::string *xml,bool type){
	string c;
	node->set_node_type(type);
	node->set_topic_type(get_ttype(*xml));
	node->set_topic_name(get_tname(*xml));
	node->set_callerid(get_cid(*xml));
	node->set_message_definition(get_msgdef(*xml));
	c += "http://";
	c += network.getIPAddress();
	c += ":11411";
	node->set_uri(c);
	if(type){
		node->set_fptr(get_fptr(*xml));
	}
}
/******************************************************************************************************************************************************************
* XML-RPC MASTER TASK  ||
 ******************************************************************************************************************************************************************/
void xml_mas_task(){
#ifndef _XML_MASTER_
#define _XML_MASTER_
	syslog(LOG_NOTICE,"========Activate mROS XML-RPC Master========");
	intptr_t *dq,*sdata;
	dq = (intptr_t *)malloc(sizeof(char)*4);
	char data[3];
	char *xdq;
	char rbuf[512];
	//rbuf = (char *)malloc(sizeof(char)*512);
#endif
	while(1){
		syslog(LOG_NOTICE,"XML_MAS_TASK:enter loop");
		TCPSocketConnection xml_mas_sock;	
		xml_mas_sock.connect(m_ip,m_port);
		rcv_dtq(XML_DTQ,dq);
		syslog(LOG_NOTICE,"XML_MAS_TASK:receive dtq");
		xdq = (char *)dq;
		int size = xdq[1];
		size += xdq[2]*256;
		size += xdq[3]*65536;
		memcpy(&rbuf,&mem[XML_ADDR],size);
		std::string str,meth;
		str = rbuf;
		int mh,mt;
		mh = (int)str.find("<methodCall>");
		mt = (int)str.find("</methodCall>");
		for(int i = mh + sizeof("<methodCall>") -1;i < mt ; i++){
			meth = meth + str[i];
		}
//===registerSubscriber=========================================================================================================================
		if(meth == "registerSubscriber"){
			node sub;
			string xml;
			syslog(LOG_NOTICE,"XML_MAS_TASK:regiester Subscriber");
			sub.ID = xdq[0];
			get_node(&sub,&str,true);														
			xml = registerSubscriber(sub.callerid,sub.topic_name,sub.topic_type,sub.uri); 	
			xml_mas_sock.send(xml.c_str(),strlen(xml.c_str()));
			xml_mas_sock.receive(rbuf,sizeof(char)*512);
			xml = rbuf;
			sub.set_port(get_port(xml));	
			node_lst.push_back(sub);
			xml = registerSubtask(sub.fptr,sub.port);
			int size = strlen(xml.c_str());
			memcpy(&mem[SUB_ADDR],xml.c_str(),size);
			mem[strlen(xml.c_str())+1] == '\0';
			data[0] = sub.ID;
			data[1] = size; 
			data[2] = size/256;
			data[3] = size/65536;
			sdata = (intptr_t) &data;
			snd_dtq(SUB_DTQ,*sdata);
//===registerPublisher===========================================================================================================================
		}else if(meth == "registerPublisher"){
			syslog(LOG_NOTICE,"XML_MAS_TASK:regiester Publisher");
			node pub;
			std::string xml;
			pub.ID = xdq[0];
			get_node(&pub,&str,false);														
			node_lst.push_back(pub);
			xml = registerPublisher(pub.callerid,pub.topic_name,pub.topic_type,pub.uri); 	
			xml_mas_sock.send(xml.c_str(),strlen(xml.c_str())); 							
			xml_mas_sock.receive(rbuf,sizeof(char)*512);
			data[0] = pub.ID;
			data[1] = 0;
			data[2] = 0;
			data[3] = 0;
			sdata = (intptr_t) &data; //日本語
			snd_dtq(PUB_DTQ,*sdata);	
//===requestTopic================================================================================================================================
		}else if(meth == "requestTopic"){	
			syslog(LOG_NOTICE,"XML_MAS_TASK:send request topic");
				int num = find_id(node_lst,xdq[0]);
				//syslog(LOG_NOTICE,"XML_MAS_TASK: node num [%d]",num);
				if(num != -1){
				//syslog(LOG_NOTICE,"XML_MAS_TASK:request node ID [%x]",node_lst[num].ID);
				string body = requestTopic(node_lst[num].callerid,node_lst[num].topic_name);
				TCPSocketConnection tmpsock;
				if(tmpsock.connect(m_ip,atoi(node_lst[num].port.c_str())) == -1){
					exit(1);
				}
				tmpsock.send(body.c_str(),strlen(body.c_str()));
				tmpsock.receive(rbuf,sizeof(char)*512);
				tmpsock.close();
				int size = strlen(rbuf);
				memcpy(&mem[SUB_ADDR],&rbuf,size);
				data[0] = node_lst[num].ID;
				data[1] = size;
				data[2] = size/256;
				data[3] = size/65536;
				sdata = (intptr_t) &data;
				snd_dtq(SUB_DTQ,*sdata);
			}else{
				//syslog(LOG_NOTICE,"XML_MAS: Can't find node! [request topic]");
			}

//===else========================================================================================================================================
		}else{	
		}
		xml_mas_sock.close();
	} //end while loop
}

//cyclic handler
void cyclic_handler(intptr_t exinf){
	//iwup_tsk():wake up tasks -> wup_tsk() causes context error
	iwup_tsk(SUB_TASK);
	iwup_tsk(XML_SLV_TASK);
}



