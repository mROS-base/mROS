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

//extern int rcv_count;
char evl_flag = 0;

/***** congiuration ros master ******/
const char *m_ip = "192.168.0.20";	//ros master IP
const int m_port = 11311;	//ros master xmlrpc port

/*********global variables***************/
/***** shared memory *****/
char mem[1024*1024*2];
extern std::vector<ID> IDv;
int ros_sem =0;	//mROS resource semapho
int count=1;	//for assign node ID

/**state->意味を成してないかも
 * 0: initial state
 * 1: register subscriber state ::XML_MAS_TASK,SUB_TASK,XML_SLV_TASK
 * 2: register publisher state ::XML_MAS_TASK,PUB_TASK,XML_SLV_TASK
 * 3: machine running state ::PUB_TASK,SUB_TASK,XML_SLV_TASK
 */
int state = 0;

/******* function to operate user task  ******/
void sus_all(){
	for(unsigned int i =0;i < IDv.size();i++){
		sus_tsk(IDv[i]);
		syslog(LOG_NOTICE,"sus user task [%d]",IDv[i]);
	}
}

void wup_all(){
	ros_sem = 0;
	for(unsigned int i=0;i < IDv.size();i++){
		rsm_tsk(IDv[i]);
		wup_tsk(IDv[i]);
		syslog(LOG_NOTICE,"wake up user task [%d]",IDv[i]);
	}
	state = 3;
	syslog(LOG_NOTICE,"Change state [%d]",state);
}


/*** mROS node vector ***/
std::vector<node> node_lst;

/***variables for evaluation***/


/* Initialize Network Configuration */
#define USE_DHCP (1)
#if(USE_DHCP == 0)
	#define IP_ADDRESS  	("192.168.0.30")	/*IP address */
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

void tcpros_decoder(char* buf,sensor_msgs::Image& msg_buf){
	//TCPROSメッセージをデコードしてメッセージバッファに入れる
/*
	if(type == "std_msgs/String"){
		int l,p;
		l = rbuf[p] + rbuf[p+1]*256 + rbuf[p+2]*65536;
		p=p+4;
		msg_buf.string = rbuf[p];
		msg_buf.string.resize(l);
	}else if(type == "sensor_msgs/Image"){
	*/
		int l,p;
		p=0;
		msg_buf.header.seq = buf[p];
		p=p+4;
		msg_buf.header.sec = buf[p];
		p=p+4;
		msg_buf.header.nsec = buf[p];
		p=p+4;
		l = buf[p];
		l = l +buf[p+1]*256;
		l = l + buf[p+2]*65536;
		p=p+4;
		msg_buf.header.frame_id = &buf[p];
		msg_buf.header.frame_id.resize(l);
		p=p+l;
		msg_buf.height = buf[p] + buf[p+1]*256 + buf[p+2]*65536;
		p=p+4;
		msg_buf.width = buf[p] + buf[p+1]*256 + buf[p+2]*65536;
		p=p+4;
		l = buf[p] + buf[p+1]*256 + buf[p+2]*65536;
		p=p+4;
		msg_buf.encoding = &buf[p];
		msg_buf.encoding.resize(l);
		p=p+l;
		msg_buf.is_bigendian=buf[p];
		p=p+1;
		msg_buf.step = buf[p] + buf[p+1]*256 + buf[p+2]*65536;
		p=p+4;
		l = buf[p];
		l= l+ buf[p+1]*256;
		l= l+ buf[p+2]*65536;
		p=p+4;
		msg_buf.data = &buf[p];
}


/**********************************
* Main Task						  *
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

	act_tsk(USR_TASK1);
	act_tsk(USR_TASK2);
	syslog(LOG_NOTICE,"**********mROS Main task finish**********");
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
			std::vector<char> stat_vec;
			int find(char c){
				if(c == 0){
					return -2;
				}
				for(unsigned int i=0;i < this->id_vec.size();i++){
					if(c == this->id_vec[i]){
						return i;
					}
				}
				return -1;
			}
			void add(TCPSocketConnection sock,char ID){this->sock_vec.push_back(sock);this->id_vec.push_back(ID);this->stat_vec.push_back(0);} //add pair
			void del(int num){this->sock_vec.erase(this->sock_vec.begin() + num - 1);this->id_vec.erase(this->id_vec.begin() + num - 1);}
	};
	TCPSocketServer srv; 	//for TCPROS connection
	srv.bind(11511);		//port binding
	srv.listen();
	pub_list lst;
	char *pdqp,*rbuf;
	intptr_t *dqp;
	dqp = (intptr_t *)malloc(sizeof(char)*4);
	char buf[1024*512];
	char snd_buf[512];
	rbuf = &buf[8];
#endif 	//_PUB_
	while(1){
		//syslog(LOG_NOTICE,"PUB_TASK:enter loop");
		if(state == 1){
			syslog(LOG_NOTICE,"PUB_TASK:sleep task");
			slp_tsk();
		}
		//ROS_INFO("PUB_TASK: wait dtq");
		rcv_dtq(PUB_DTQ,dqp);
		//ROS_INFO("PUB_TASK: [%d]",dqp[0]);
		pdqp = (char *)dqp;
		int num = lst.find(pdqp[0]);
		//syslog(LOG_NOTICE,"PUB_TASK:operate node [%d]",pdqp[0]);
		int node_num = find_id(node_lst,pdqp[0]);			
		int size;
		size = pdqp[1];
		size += pdqp[2]*256;
		size += pdqp[3]*65536;
		if(num == -1){ 									
			//initialization
			syslog(LOG_NOTICE,"PUB_TASK:publisher initialization Node ID[%d]",pdqp[0]);
			static TCPSocketConnection sock;
			// set socket option NoDelay
			//int l = sock.set_option(IPPROTO_TCP,TCP_NODELAY,0,1);
			//ROS_INFO("PUB_TASK: SOCKET option [%d]",l);
			lst.add(sock,pdqp[0]);
			wup_all();
			rsm_tsk(SUB_TASK);
			state = 3;
			syslog(LOG_NOTICE,"Change state [%d]\n",state);
		}else if((num != -1) && (size == 0)){
			//receive request topic
			bool connect_status = false;
			bool svr_status = true;
			//wait TCPROS connection
			syslog(LOG_NOTICE,"PUB_TASK:request Topic node[%x]",node_lst[node_num].ID);
			while(svr_status){
				syslog(LOG_NOTICE,"PUB_TASK:Listening");
				if(srv.accept(lst.sock_vec[num]) == 0){
					svr_status = false;
					syslog(LOG_NOTICE,"PUB_TASK:Connected");
					connect_status = true;
					while(connect_status){
						//to do: check if data is TCPROS header
						int con_stat = lst.sock_vec[num].receive(snd_buf,512);
						syslog(LOG_NOTICE,"PUB_TASK: TCPROS connection received");
						switch(con_stat){
						case 0:
						case -1:
							connect_status = false;
							break;
						default:
#if 0
<<<<<<< HEAD
							int len;
							//if(check_head(rbuf)){
							if(node_lst[node_num].topic_type == "std_msgs/String"){
								len = pub_gen_header(snd_buf,node_lst[node_num].callerid,node_lst[node_num].message_definition,node_lst[node_num].topic_name,node_lst[node_num].topic_type,"992ce8a1687cec8c8bd883ec73ca41d1");
							}else if(node_lst[node_num].topic_type == "sensor_msgs/Image"){
								len = pub_gen_header(snd_buf,node_lst[node_num].callerid,node_lst[node_num].message_definition,node_lst[node_num].topic_name,node_lst[node_num].topic_type,"060021388200f6f0f447d0fcd9c64743");
							}
							//syslog(LOG_NOTICE,"PUB_TASK: TCPROS connection header[%s]",snd_buf[8]);
=======
#endif
							// TODO: check last arg
              /** for image data **/
							int len = pub_gen_header(snd_buf,node_lst[node_num].callerid,node_lst[node_num].message_definition,node_lst[node_num].topic_name,node_lst[node_num].topic_type,"060021388200f6f0f447d0fcd9c64743");	//test function
							/**for string data**/
							//int len = pub_gen_header(snd_buf,node_lst[node_num].callerid,node_lst[node_num].message_definition,node_lst[node_num].topic_name,node_lst[node_num].topic_type,"992ce8a1687cec8c8bd883ec73ca41d1");	//test function
//>>>>>>> mori_ws
							lst.sock_vec[num].send(snd_buf,len);
							node_lst[node_num].set_pub();
							connect_status = false;
						}
					}
					syslog(LOG_NOTICE,"PUB_TASK: publisher connected");
					wup_all();
					rsm_tsk(SUB_TASK);
				}
			}
		}else if(num == -2){
			//internal node request
			ROS_INFO("PUB_TASK:Internal request size:[%d]",size);
			string str;
			for(int i=0;i<size;i++){
				str += mem[PUB_ADDR2+i];
			}
			//ROS_INFO("PUB_TASK: topic:[%s]",str.c_str());
			int i = find_node(node_lst,str);
			int ii = lst.find(node_lst[i].ID);
			lst.stat_vec[ii] = 0;
			ROS_INFO("PUB_TASK:accept internal request [%d][%d]",i,ii);
		}else{
#if 0
<<<<<<< HEAD
		//publish phase
		//ROS_INFO("data publish phase");
		memcpy(rbuf,mem,size);
		rbuf[size] = '\0';	//cutting data end
		int l;
		if(node_lst[node_num].topic_type == "std_msgs/String"){
			l = pub_gen_msg(buf,rbuf);
		}else if(node_lst[node_num].topic_type == "sensor_msgs/Image"){
			//publish
			l = pub_gen_img_msg(buf,rbuf,size);
		}else{
			syslog(LOG_NOTICE,"PUB_TASK: NOT SUPPORTED TOPIC TYPE");
		}
		//ROS_INFO("data publish");
		int err = lst.sock_vec[num].send(buf,l);
		 //ROS_INFO("error code [%d]",err);
=======
#endif
			//publish phase
			if(lst.stat_vec[num] == 0){
			//	if(lst.sock_vec[num].is_connected()){
				//ROS_INFO("PUB_TASK: TOPIC send size[%d]",size);
				memcpy(rbuf,&mem[PUB_ADDR],size);
				//ROS_INFO("PUB_TASK: memcpy");
				rbuf[size] = '\0';	//cutting data end
				/**for string data**/
				int l = pub_gen_msg(buf,rbuf);	
				/**for image data**/
				//int la = pub_gen_img_msg(buf,rbuf,size);
				//ROS_INFO("PUB_TASK: generate TCPROS[%d]",l);
				//publish
				int err = lst.sock_vec[num].send(buf,l);
				int number = errno;
				//ROS_INFO("PUB_TASK: send[%d]",err);
				ROS_INFO(buf);
				if(err < 0)  ROS_INFO("PUB_TASK: PUBLISHING ERROR ! [%d] errno=%d %s",err,number,strerror(number));
			}else if(lst.stat_vec[num] == 1){
				syslog(LOG_NOTICE,"PUB_TASK: internal data publish");
			}
//>>>>>>> mori_ws
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
		std::vector<char> stat_vec;					//state vector 0:unregister state 1:prepared subscribe from external node 2:prepared subscribe internal node
		int find(char c){
					for(unsigned int i=0;i < this->id_vec.size();i++){
						if(c == this->id_vec[i]){
							return i;
						}
					}
					return -1;
				}
		void add(TCPSocketConnection sock,char ID,intptr_t func){this->sock_vec.push_back(sock);this->id_vec.push_back(ID);this->func_vec.push_back(func);this->stat_vec.push_back(0);}
		void set_stat(int stat,int idx){this->stat_vec[idx] = stat;}
	};
	sub_list lst;
	intptr_t *dqp,*snd_dqp;
	dqp = (intptr_t *)malloc(sizeof(char)*4);
	char *sdq,*rptr; //data queue pointer,receive buffer,memory address
	char rbuf[1024*350];
	int port;	//port number
	int left;   //receive length
	bool rcv_flag = true;
	bool init_flag = true;
	int len,msg_size;
	sensor_msgs::Image msg_buf;
#endif //_SUB_

	while(1){
		//syslog(LOG_NOTICE,"SUB_TASK:enter loop");
		slp_tsk();
		int t = trcv_dtq(SUB_DTQ,dqp,1);	//if queue is empty, go subscribe loop
	    if(t == 0){
	    	sdq = (char *)dqp;
			int num = lst.find(sdq[0]);
			//syslog(LOG_NOTICE,"SUB_TASK: node id [%d]",sdq[0]);
			if(num == -1){
				int node_num = find_id(node_lst,sdq[0]);
	    		int idx = lst.id_vec.size();
	    		//initialize
				syslog(LOG_NOTICE,"SUB_TASK: subscriber initialization　node ID:[%d] index:[%d]",sdq[0],idx);
				ROS_INFO("SUB_TASK: subscriber initialization　node ID:[%d] index:[%d]",sdq[0],idx);
				static TCPSocketConnection sock;
				static intptr_t funcp;
				int size = sdq[1];
				size += sdq[2]*256;
				size += sdq[3]*65536;
				memcpy(rbuf,&mem[SUB_ADDR],size);
				//get function pointer address
				string str = rbuf;
				funcp = (intptr_t)atoi(get_fptr(str).c_str());
				lst.add(sock,sdq[0],funcp);
#if 0
<<<<<<< HEAD
				//send requestTopic
				str = "<methodCall>requestTopic</methodCall>";
				size = str.size();
				memcpy(&mem[XML_ADDR],str.c_str(),size);
				char buf[3];
				buf[0] = sdq[0];
				buf[1] = size;
				buf[2] = size/256;
				buf[3] = size/65536;
				snd_dqp = (intptr_t)buf;
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
				//send TCPROS connection header
				//for image data
				//size = sub_gen_header(tmp,node_lst[node_num].callerid,"0",node_lst[node_num].topic_name,node_lst[node_num].topic_type,"060021388200f6f0f447d0fcd9c64743");
				//for string data
				size = sub_gen_header(tmp,node_lst[node_num].callerid,"0",node_lst[node_num].topic_name,node_lst[node_num].topic_type,"992ce8a1687cec8c8bd883ec73ca41d1");
				tmp[size]  = '0';
				lst.sock_vec[idx].connect(ip,port);
				lst.sock_vec[idx].send(tmp,size);
				wait_ms(10);
				lst.sock_vec[idx].receive(tmp,5000);
				lst.set_stat(true,idx);
				init = true;
=======
#endif
				syslog(LOG_NOTICE,"SUB_TASK:IP [%s][%s]",node_lst[node_num].ip.c_str(),network.getIPAddress());
				ROS_INFO("SUB_TASK:IP [%s][%s]",node_lst[node_num].ip.c_str(),network.getIPAddress());
				if(strcmp(node_lst[node_num].ip.c_str(),network.getIPAddress()) != 0){
//				if(strcmp(node_lst[node_num].ip.c_str(),network.getIPAddress()) == 0){
					//send requestTopic
					str = "<methodCall>requestTopic</methodCall>";
					size = str.size();
					memcpy(&mem[XML_ADDR],str.c_str(),size);
					char buf[3];
					buf[0] = sdq[0];
					buf[1] = size;
					buf[2] = size/256;
					buf[3] = size/65536;
					snd_dqp = (intptr_t)buf;
					snd_dtq(XML_DTQ,*snd_dqp);
					rcv_dtq(SUB_DTQ,dqp);
					sdq = (char *)dqp;
					size = sdq[1];
					size += sdq[2]*256;
					size += sdq[3]*65536;
					memcpy(&rbuf,&mem[SUB_ADDR],size);				//get date
					str = rbuf;
					port = atoi(get_port2(str).c_str());	//test function
					syslog(LOG_NOTICE,"SUB_TASK:port [%d]",port);
		//send TCPROS connection header
				/**for image data**/
					//size = sub_gen_header(tmp,node_lst[node_num].callerid,"0",node_lst[node_num].topic_name,node_lst[node_num].topic_type,"060021388200f6f0f447d0fcd9c64743");
				/**for string data**/
					size = sub_gen_header(rbuf,node_lst[node_num].callerid,"0",node_lst[node_num].topic_name,node_lst[node_num].topic_type,"1df79edf208b629fe6b81923a544552d");
					rbuf[size]  = '0';
					int err = lst.sock_vec[idx].connect(node_lst[node_num].ip.c_str(),port);
					if(err != 0){
					//if(lst.sock_vec[idx].connect(node_lst[node_num].ip.c_str(),port) != 0){
						ROS_INFO("SUB_TASK: TCPROS HEADER CONNECT ERROR");
						ROS_INFO("SUB_TASK: IP:[%s] PORT[%d]",node_lst[node_num].ip.c_str(),port);
						exit(1);
					}
					ROS_INFO("SUB_TASK: TCPROS HEADER CONNECT");
					err =  lst.sock_vec[idx].send(rbuf,size);
					//if(lst.sock_vec[idx].send(tmp,size) < 0){
					if(err < 0){
						ROS_INFO("SUB_TASK: TCPROS HEADER SEND ERROR err[%d]",err);
						exit(1);
					}
					ROS_INFO("SUB_TASK: SEND TCPROS HEADER");
					lst.sock_vec[idx].receive(rbuf,1024);
					lst.set_stat(1,idx);
				}else{
					//Internal request topic
					ROS_INFO("SUB_TASK: Internal request");
					lst.set_stat(2,idx);
					memcpy(&mem[PUB_ADDR2],node_lst[node_num].topic_name.c_str(),strlen(node_lst[node_num].topic_name.c_str()));
					char buf[3];
					buf[0] = 0;
					buf[1] = strlen(node_lst[node_num].topic_name.c_str());
					buf[2] = 0;
					buf[3] = 0;
					snd_dqp = (intptr_t)buf;
					snd_dtq(PUB_DTQ,*snd_dqp);
					}
//>>>>>>> mori_ws
				syslog(LOG_NOTICE,"SUB_TASK: subscriber connected");
				//rcv_count = 1;
				wup_all();
				//end initialize
				//sus_tsk(LOGTASK);
			}else if(num != -1 && lst.stat_vec[num] == 2){
				//方式3,4専用
				//syslog(LOG_NOTICE,"SUB_TASK: subscribe internal topic data");
				int size = sdq[1];
				size += sdq[2]*256;
				size += sdq[3]*65536;
				//ROS_INFO("SUB_TASK: data size[%d]",size);
				memcpy(rbuf,&mem[PUB_ADDR2],size);
				tcpros_decoder(rbuf,msg_buf);		//TCPROSからメッセージに戻す
				void (*fp)(sensor_msgs::Image&);
				fp = lst.func_vec[num];
				fp(msg_buf);
			}
	    }else{
#if 0
<<<<<<< HEAD
	    //subscribe and callback loop
	    	if(init){
	    		syslog(LOG_NOTICE,"SUB_TASK: subscribing");
	    		for(unsigned int i=0;i < lst.id_vec.size();i++){
					rptr = &rbuf[0];
					if(lst.stat_vec[i]){
						rcv_flag = true;
						while(rcv_flag){
							int n = lst.sock_vec[i].receive(rptr,512);
							if(n < 0){
								syslog(LOG_NOTICE,"SUB_TASK: No data");
							}else{
								if(init_flag){
									msg_size = (int)rbuf[0] + (int)rbuf[1]*256;// + rbuf[2]*65536 + rbuf[3]*16777216;
									data_size = (int)rbuf[4] + (int)rbuf[5]*256;// + rbuf[6]*65536 + rbuf[7]*16777216;
									init_flag = false;
								}
								len += n;
								///syslog(LOG_NOTICE,"SUB_TASK:data length [%d]",msg_size);
								if(len == msg_size +4){
									//correct data received
									rbuf[len] = '\0';
									//syslog(LOG_NOTICE,"SUB_TASK:data length [%d]",len);
									void (*fp)(string);
									fp = lst.func_vec[i];
									fp(&rbuf[8]);
									rptr = &rbuf[0];
									rcv_flag = false;
									init_flag = true;
									len = 0;
								}else if(len > msg_size + 4){
									//data overflow
									syslog(LOG_NOTICE,"invalid header[%d][%d] [%d]",msg_size,len,n);
									rptr = &rbuf[0];
									rcv_flag = false;
									init_flag = true;
									len = 0;
								}else{
									//syslog(LOG_NOTICE,"SUB_TASK: data long");
									//data receiving
									rptr = &rbuf[n];
								}
=======
#endif
 //subscribe and callback loop
			for(unsigned int i=0;i < lst.id_vec.size();i++){
				rptr = &rbuf[0];
				if(lst.stat_vec[i] == 1){
					rcv_flag = true;
					len = 0;
					while(rcv_flag){
						int n=0;
						if(init_flag){
							n = lst.sock_vec[i].receive(rptr,512);
							left = 0;
						}else{
							n = lst.sock_vec[i].receive(rptr,left);
						}
						if(n < 0){
							syslog(LOG_NOTICE,"SUB_TASK: No data");
						}else{
							if(init_flag){
								ROS_INFO("SUB_TASK: %x %x %x %x",rbuf[0],rbuf[1],rbuf[2],rbuf[3]);
								msg_size = (unsigned int)rbuf[0];
								msg_size += (unsigned int)rbuf[1]*256;
								msg_size += (unsigned int)rbuf[2]*65536;// + rbuf[3]*16777216;
								ROS_INFO("SUB_TASK: msg size [%d]B",msg_size);
								init_flag = false;
							}
							len += n;
							if(len >= msg_size +4){
				//data received
								rbuf[msg_size + 4] = '\0';
								syslog(LOG_NOTICE,"SUB_TASK:data length [%d]",len);
								void (*fp)(intptr_t);		//stringのみ対応
								fp = lst.func_vec[i];
								int hoge = (int)rbuf[4] + (int)rbuf[5]*256;
								fp(&hoge);
								syslog(LOG_NOTICE,"SUB_TASK:data recieved [%d]",hoge);
								//fp(&rbuf[8]);
								rptr = &rbuf[0];
								rcv_flag = false;
								init_flag = true;
								evl_flag = 0;
								len = 0;
							}else{
								syslog(LOG_NOTICE,"SUB_TASK: data long");
				//data receiving
								rptr = &rbuf[n];
								left = msg_size + 4 - len;
								ROS_INFO("SUB_TASK: left length [%d]",left);
								evl_flag = 1;
							}
//>>>>>>> mori_ws
						}
					}
				}else if(lst.stat_vec[i] == 2){

				}else if(lst.stat_vec[i] == 0){

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
	int port = 11411;
	TCPSocketConnection xml_slv_sock;
	TCPSocketServer xml_slv_srv;
	if(xml_slv_srv.bind(port) == -1){
		syslog(LOG_NOTICE,"Failed bind");
		exit(1);
	}
	xml_slv_srv.set_blocking(true,1500);	//blockingじゃないと周期的に取れない
	char rbuf[512];
	char data[3];
	string str,meth;
	intptr_t *dqp;
	bool connect_status = false;
#endif
	xml_slv_srv.listen();
	while(1){  // CYCLIC LOOP
		slp_tsk();
		ROS_INFO("XML_SLV_TASK: Listen");
		int err = xml_slv_srv.accept(xml_slv_sock);
		//ROS_INFO("XML_SLV_TASK: ACCEPT err [%d]",err);
		if(err == 0){
			connect_status = true;
			syslog(LOG_NOTICE,"XML_SLV_TASK: Connected");
			while(connect_status){
				int stat = xml_slv_sock.receive(rbuf,512);
				//ROS_INFO("XML_SLV_TASK:%s",rbuf);
				switch(stat){
					case 0:
					case -1:
						connect_status = false;
						syslog(LOG_NOTICE,"XML_SLV_TASK:disconnected");
						break;
					default:
						str = rbuf;
						int mh,mt;
						//get method
						mh = (int)str.find("<methodName>");
						mt = (int)str.find("</methodName>");
						meth = "";
						for(int i = mh + sizeof("<methodName>") -1;i < mt ; i++){
							meth = meth + str[i];
						}
						syslog(LOG_NOTICE,"XML_SLV_TASK: methodName [%s]",meth.c_str());
						if(meth == "requestTopic"){
							sus_all();
							sus_tsk(SUB_TASK);
							state = 2;
							syslog(LOG_NOTICE,"Change state [%d]",state);
							string topic_name = req_topic_name(str);
							syslog(LOG_NOTICE,"XML_SLV_TASK: request topic [%s]",topic_name.c_str());
							int num = find_node(node_lst,topic_name);
							if(num == -1){
								syslog(LOG_NOTICE,"XML_SLV_TASK: No Existing node!");
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
							state = 4;
							syslog(LOG_NOTICE,"Change state [%d]",state);
						}else{
							syslog(LOG_NOTICE,"XML_SLV_TASK: unknown method\n%s",str.c_str());
							connect_status = false;
						}
					}
				}
			}else{
				ROS_INFO("XML_SLV_TASK: ACCEPT ERROR!");
			}
		xml_slv_sock.close();
	}
}



// SET NODE INFORMATION
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
#endif
	while(1){
		syslog(LOG_NOTICE,"XML_MAS_TASK: enter loop");
		TCPSocketConnection xml_mas_sock;
		xml_mas_sock.set_blocking(true,1500);
		rcv_dtq(XML_DTQ,dq);
		syslog(LOG_NOTICE,"XML_MAS_TASK: receive dtq");
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
			xml_mas_sock.connect(m_ip,m_port);
			sus_all();
			node sub;
			string xml;
			syslog(LOG_NOTICE,"XML_MAS_TASK: regiester Subscriber ID:[%d]",xdq[0]);
			sub.ID = xdq[0];
			get_node(&sub,&str,true);														
			xml = registerSubscriber(sub.callerid,sub.topic_name,sub.topic_type,sub.uri);
			xml_mas_sock.send(xml.c_str(),xml.size());
			xml_mas_sock.receive(rbuf,sizeof(char)*512);
			xml = rbuf;
			//ROS_INFO("%s",xml.c_str());
			//怪しい
			if(strcmp(get_ip(xml).c_str(),network.getIPAddress()) != 0){
				sub.set_ip(get_ip(xml));
				//sub.set_ip(m_ip);
			}else{
				sub.set_ip(get_ip(xml));
			}
			sub.set_port(get_port(xml));
			ROS_INFO("XML_MAS_TASK: ip[%s],port[%d]",sub.ip.c_str(),sub.port);
			node_lst.push_back(sub);
			xml = registerSubtask(sub.fptr);
			int size = xml.size();
			memcpy(&mem[SUB_ADDR],xml.c_str(),size);
			mem[SUB_ADDR+size+1] = '\0';
			data[0] = sub.ID;
			data[1] = size; 
			data[2] = size/256;
			data[3] = size/65536;
			sdata = (intptr_t) &data;
			snd_dtq(SUB_DTQ,*sdata);

//===registerPublisher===========================================================================================================================
		}else if(meth == "registerPublisher"){
			xml_mas_sock.connect(m_ip,m_port);
			sus_tsk(SUB_TASK);
			syslog(LOG_NOTICE,"XML_MAS_TASK: Sleep SUB_TASK");
			wup_tsk(XML_SLV_TASK);
			node pub;
			std::string xml;
			syslog(LOG_NOTICE,"XML_MAS_TASK: regiester Publisher ID:[%d]",xdq[0]);
			pub.ID = xdq[0];
			get_node(&pub,&str,false);														
			node_lst.push_back(pub);
			xml = registerPublisher(pub.callerid,pub.topic_name,pub.topic_type,pub.uri);
			//ROS_INFO("%s",xml.c_str());
			xml_mas_sock.send(xml.c_str(),xml.size());
			xml_mas_sock.receive(rbuf,sizeof(char)*512);
			data[0] = pub.ID;
			data[1] = 0;
			data[2] = 0;
			data[3] = 0;
			sdata = (intptr_t) &data; 
			snd_dtq(PUB_DTQ,*sdata);	
//===requestTopic================================================================================================================================
		}else if(meth == "requestTopic"){	
			syslog(LOG_NOTICE,"XML_MAS_TASK: send request topic");
				int num = find_id(node_lst,xdq[0]);
				//syslog(LOG_NOTICE,"XML_MAS_TASK: node num [%d]",num);
				if(num != -1){
				syslog(LOG_NOTICE,"XML_MAS_TASK: request node [ID:%x, topic:%s]",node_lst[num].ID,node_lst[num].topic_name.c_str());
				string body = requestTopic(node_lst[num].callerid,node_lst[num].topic_name);
				node_lst[num].ip = "192.168.0.20";
				syslog(LOG_NOTICE,"XML_MAS_TASK: ip[%s],port[%d]",node_lst[num].ip.c_str(),node_lst[num].port);
				//syslog(LOG_NOTICE,"XML_MAS_TASK: %s",body.c_str());
				int le = xml_mas_sock.connect(node_lst[num].ip.c_str(),node_lst[num].port);
				if(le < 0){
					syslog(LOG_NOTICE,"XML_MAS_TASK: ip[%s],port[%d]:: err [%d]",node_lst[num].ip.c_str(),node_lst[num].port,le);
					syslog(LOG_NOTICE,"XML_MAS_TASK: cannot connect pub node");
					syslog(LOG_NOTICE,"XML_MAS_TASK: Terminated TASK");
					//エラーシグナルをサブタスクに送るようにする
					break;
				}
				//syslog(LOG_NOTICE,"XML_MAS_TASK: ip[%s],port[%d]:: err [%d]",node_lst[num].ip.c_str(),node_lst[num].port,le);
				//ROS_INFO("XML_MAS_TASK: connct is_connected [%s]",xml_mas_sock.is_connected()?"true":"false");
				le = xml_mas_sock.send(body.c_str(),body.size());
				if(le < 0){
					ROS_INFO("FAIL SEND REQUEST [%d]",le);
					break;
				}
				ROS_INFO("COMPLETE SEND REQUEST [%d]",le);
				xml_mas_sock.receive(rbuf,sizeof(char)*512);
				int size = strlen(rbuf);
				memcpy(&mem[SUB_ADDR],&rbuf,size);
				data[0] = node_lst[num].ID;
				data[1] = size;
				data[2] = size/256;
				data[3] = size/65536;
				sdata = (intptr_t) &data;
				snd_dtq(SUB_DTQ,*sdata);
			}else{
				syslog(LOG_NOTICE,"XML_MAS: Can't find node! [request topic]");
			}
//===else========================================================================================================================================
		/*機能追加はここ*/
		}else if(meth == "requestTopic_INT"){
			syslog(LOG_NOTICE,"XML_MAS_TASK:internal request topic");
		}else{
		}
		xml_mas_sock.close();
	} //end while loop
}

//cyclic handler
void cyclic_handler(intptr_t exinf){
	iwup_tsk(SUB_TASK);
	iwup_tsk(XML_SLV_TASK);
}


