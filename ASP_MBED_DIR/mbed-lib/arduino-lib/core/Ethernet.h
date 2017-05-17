#ifndef _ETHERNET_H_
#define _ETHERNET_H_

/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
* other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
* EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
* SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS
* SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
* this software. By using this software, you agree to the additional terms and conditions found by accessing the
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2014 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/
/***********************************************************************************************************************
* File Name	   : r_dhcp_client.h
* Version      : 1.00
* Description  :
***********************************************************************************************************************/
/***********************************************************************************************************************
* History : DD.MM.YYYY Version Description
*         : 15.03.2011 1.00    First Release
*         : 09.05.2014 1.01    Corresponded to FIT Modules.
***********************************************************************************************************************/
#ifndef	R_DHCP_CLIENT_H
#define	R_DHCP_CLIENT_H

#if !defined(__GNUC__) && !defined(GRSAKURA)
#pragma pack
#endif

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
#define EXPANSION_DHCP_PACKET_SIZE 300
#define TRANSACTION_ID	0x12345678

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/
typedef struct _dv_options
{
	uint32_t	magic_cookie;
	uint16_t	message_type1;
	uint8_t		message_type2;
	uint16_t	client_id1;
	uint8_t		client_id2;
	uint8_t		client_mac[6];
	uint8_t		dummy[48 + EXPANSION_DHCP_PACKET_SIZE];
#if defined(__GNUC__) || defined(GRSAKURA)
} __attribute__((__packed__)) DV_OPTIONS;
#else
}DV_OPTIONS;
#endif

typedef struct _dhcp_data
{
	uint8_t		opecode;
	uint8_t		hard_addr;
	uint8_t		hard_addr_len;
	uint8_t		hop_count;
	uint32_t	transaction_id;
	uint16_t	second;
	uint16_t	dummy;
	uint8_t		client_ip[4];
	uint8_t		user_ip[4];
	uint8_t		server_ip[4];
	uint8_t		gateway_ip[4];
	uint8_t		client_hard_addr[16];
	uint8_t		server_host_name[64];
	uint8_t		file_name[128];
	DV_OPTIONS	options;
#if defined(__GNUC__) || defined(GRSAKURA)
} __attribute__((__packed__)) DHCP_DATA;
#else
}DHCP_DATA;
#endif


typedef struct udp_packet
{

	uint16_t	source_port;
	uint16_t	destination_port;
	uint16_t	length;
	uint16_t	checksum;

#if defined(__GNUC__) || defined(GRSAKURA)
} __attribute__((__packed__)) UDP_PACKET;
#else
}UDP_PACKET;
#endif


typedef struct ipv4_packet
{

	uint8_t		version_and_length;
	uint8_t		differentiated_services_field;
	uint16_t	total_length;
	uint16_t	identification;
	uint16_t	flags_and_fragment_offset;
	uint8_t		time_to_live;
	uint8_t		protocol;
	uint16_t	checksum;
	uint8_t		source_ip[4];
	uint8_t		destination_ip[4];

#if defined(__GNUC__) || defined(GRSAKURA)
} __attribute__((__packed__)) IPV4_PACKET;
#else
}IPV4_PACKET;
#endif

typedef struct ether_packet
{

	uint8_t		destination_address[6];
	uint8_t		source_address[6];
	uint16_t	packet_type;

#if defined(__GNUC__) || defined(GRSAKURA)
} __attribute__((__packed__)) ETHER_PACKET;
#else
}ETHER_PACKET;
#endif

typedef struct _dhcp_packet
{
	ETHER_PACKET	ether;
	IPV4_PACKET		ipv4;
	UDP_PACKET		udp;
	DHCP_DATA		dhcp;
#if defined(__GNUC__) || defined(GRSAKURA)
} __attribute__((__packed__)) DHCP_PACKET;
#else
}DHCP_PACKET;
#endif

/***********************************************************************************************************************
Exported global variables
***********************************************************************************************************************/

/***********************************************************************************************************************
Exported global functions (to be accessed by other files)
***********************************************************************************************************************/
#if !defined(__GNUC__) && !defined(GRSAKURA)
static int32_t dhcp_discover(DHCP *dhcp, DHCP_PACKET *dhcp_packet);
static int32_t dhcp_wait_offer(DHCP *dhcp, DHCP_PACKET *dhcp_packet);
static int32_t dhcp_request(DHCP *dhcp, DHCP_PACKET *dhcp_packet);
static int32_t dhcp_wait_ack(DHCP *dhcp, DHCP_PACKET *dhcp_packet);
static int32_t dchp_release(DHCP *dhcp, DHCP_PACKET *dhcp_packet);
static uint16_t htons(uint16_t data);
static uint32_t htonl(uint32_t data);
static uint16_t checksum(uint16_t *data, int32_t len);
static uint16_t checksum_udp(uint16_t *pre_header, uint16_t *data, int32_t len);
#endif

void	 reset_timer(void);
uint16_t get_timer(void);

#endif	/* R_DHCP_CLIENT_H */
/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
* other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
* EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
* SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS
* SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
* this software. By using this software, you agree to the additional terms and conditions found by accessing the
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2014 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/
/***********************************************************************************************************************
* File Name	   : r_t4_dhcp_client_rx_if.h
* Version      : 1.01
* Description  :
***********************************************************************************************************************/
/***********************************************************************************************************************
* History : DD.MM.YYYY Version  Description
*         : 15.03.2011 1.00    First Release
*         : 09.05.2014 1.01    Corresponded to FIT Modules
***********************************************************************************************************************/
#ifndef	R_T4_DHCP_CLIENT_RX_IF_H
#define	R_T4_DHCP_CLIENT_RX_IF_H

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
/* Version Number of API. */
#define T4_DHCP_CLIENT_VERSION_MAJOR       (1)
#define T4_DHCP_CLIENT_VERSION_MINOR       (01)

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/
typedef struct _dhcp
{
	uint8_t	ipaddr[4];
	uint8_t maskaddr[4];
	uint8_t	gwaddr[4];
	uint8_t	dnsaddr[4];
	uint8_t	dnsaddr2[4];
#if defined(__GNUC__) || defined(GRSAKURA)
    uint32_t dhcpRenewalTimeValue;
    uint32_t dhcpRebindingTimeValue;
    uint32_t dhcpIPAddressLeaseTime;
#endif
	char	domain[20];
	uint8_t	macaddr[6];
}DHCP;

/***********************************************************************************************************************
Exported global variables
***********************************************************************************************************************/

/***********************************************************************************************************************
Exported global functions (to be accessed by other files)
***********************************************************************************************************************/
#if defined(__GNUC__) || defined(GRSAKURA)
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */
int32_t  r_dhcp_open(DHCP *dhcp, uint8_t *work, uint8_t *mac_addr);
uint32_t R_T4_DHCP_CLIENT_GetVersion(void);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif

#endif	/* R_T4_DHCP_CLIENT_RX_IF_H */

//typedef signed long ID;
//typedef unsigned char UB;
//typedef unsigned short UH;
//typedef int ER;

#if 0
#include "utility/T4_src/type.h"
#include "utility/T4_src/IPAddress.h"
#include "utility/T4_src/r_t4_itcpip.h"
#include "utility/T4_src/r_t4_dhcp_client_rx_if.h"
#include "utility/T4_src/r_dhcp_client.h"
#include "utility/T4_src/r_t4_dns_client_rx_if.h"
#include "utility/T4_src/r_dns_client.h"
#include "utility/T4_src/ether.h"
#include "utility/T4_src/ip.h"
#include "utility/T4_src/tcp.h"
#include "utility/driver/timer.h"
#include "utility/driver/r_ether.h"
#else
#include "IPAddress.h"
#endif


/******************************************************************************
Macro definitions
******************************************************************************/
#define ARDUINO_TCP_CEP          1
#define ARDUINO_UDP_CEP          1
#define T4_CLOSED               0
#define T4_TCPS_ESTABLISHED     2
#define T4_TCPS_CLOSE_WAIT      4
#define T4_TCPS_CLOSED          0
#define DHCP_NOTHING_HAPPEND    0
#define DHCP_RENEW_SUCCESS      2
#define TCPUDP_WORK                     1780/sizeof(UW)     /*20150520 wed review*/
#define UDP_RCV_DAT_DATAREAD_MAXIMUM    1472
#define UDP_RCV_BUFFER_SIZE             1024
#define TCP_MSS                         1460
#define UDP_TX_PACKET_MAX_SIZE          24                  /*Along with Arduino original code*/

#if 0
typedef struct _CEP{
    uint32_t    status;
    T_IPV4EP    dst_addr;
    T_IPV4EP    src_addr;
    int32_t     current_rcv_data_len;
    int32_t     total_rcv_len;
    UB          rcv_buf[TCP_MSS];
    UB          snd_buf[TCP_MSS];
    int32_t     _1sec_counter;
    int32_t     _1sec_timer;
    int32_t     pre_1sec_timer;
}CEP;

extern _TCB   *head_tcb;
extern UB _t4_channel_num;
extern T_TCP_CREP tcp_crep[];
extern T_TCP_CCEP tcp_ccep[];
extern T_UDP_CCEP udp_ccep[];
extern const H __tcprepn;
extern const H __tcpcepn;
extern const H __udpcepn;
extern uint8_t dnsaddr1[];
extern uint8_t dnsaddr2[];
extern UW tcpudp_work[TCPUDP_WORK];
extern TCPUDP_ENV tcpudp_env;
extern volatile UH wait_timer;
extern uint8_t     cepid_max;
extern NAME_TABLE  name_table;
extern DNS_MNG     dns_mng;

extern "C"{
    ER tcp_read_stat(ID cepid);
    ER tcp_force_clr(ID cepid);
    void ConfigurePortPins(void);
    void EnablePeripheralModules(void);
    ER http_callback(ID cepid, FN fncd , VP p_parblk);
    ER dns_callback(ID cepid, FN fncd , VP p_parblk);
    ER t4_tcp_callback(ID cepid, FN fncd , VP p_parblk);
}
extern "C" ER USB_dataReceive_callback(ID cepid, FN fncd,VP p_parblk);
extern "C" void queueInit(void);
void setup_terminal_wait();
#endif

class EthernetClass;
class EthernetClient;
extern class EthernetClass Ethernet;

class EthernetClass : public Print{
	private:
#if 0
        uint32_t    dhcpIPAddressLeaseTime_sec;
        uint32_t    dhcpUse;
        bool        tcp_acp_cep_call_flg;
        void dhcpSuccess(DHCP *tmpDhcpPt)
        {
            memcpy(tcpudp_env.ipaddr, tmpDhcpPt->ipaddr, 4);
#ifdef T4_ETHER_DEBUG
            Serial.print("ip = ");
            Serial.println(localIP());
#endif
            memcpy(tcpudp_env.maskaddr, tmpDhcpPt->maskaddr, 4);
#ifdef T4_ETHER_DEBUG
            Serial.print("snm = ");
            Serial.println(subnetMask());
#endif
            memcpy(tcpudp_env.gwaddr, tmpDhcpPt->gwaddr, 4);
#ifdef T4_ETHER_DEBUG
            Serial.print("gw = ");
            Serial.println(gatewayIP());
#endif
            memcpy((char *)dnsaddr1, (char *)tmpDhcpPt->dnsaddr, 4);
#ifdef T4_ETHER_DEBUG
            Serial.print("dns = ");
            Serial.println(dnsServerIP());
#endif
            memcpy(dhcpSvMac.bytes, ((DHCP_PACKET*)tcpudp_work)->ether.source_address, EP_ALEN);
#ifdef T4_ETHER_DEBUG
            Serial.print("dhcpSvmac = ");
            Serial.print(dhcpSvMac.bytes[0],HEX);
            Serial.print(":");
            Serial.print(dhcpSvMac.bytes[1],HEX);
            Serial.print(":");
            Serial.print(dhcpSvMac.bytes[2],HEX);
            Serial.print(":");
            Serial.print(dhcpSvMac.bytes[3],HEX);
            Serial.print(":");
            Serial.print(dhcpSvMac.bytes[4],HEX);
            Serial.print(":");
            Serial.println(dhcpSvMac.bytes[5],HEX);
#endif
            memcpy(dhcpSvIp.bytes, ((DHCP_PACKET*)tcpudp_work)->ipv4.source_ip, IP_ALEN);
#ifdef T4_ETHER_DEBUG
            Serial.print("dhcpSvIP = ");
            Serial.println(dhcpSvIp.dword,HEX);
#endif
            dhcpUse = true;
            Ethernet.dhcpIPuse_sec = 0;                 /*dhcp lease time local countup start*/
            Ethernet.fromSystemGetLastTime = millis();
            dhcpIPAddressLeaseTime_sec = tmpDhcpPt->dhcpIPAddressLeaseTime;     /*ip lease limit from dhcpSv*/
#ifdef T4_ETHER_DEBUG
            Serial.print("dhcpIPAddressLeaseTime_sec = ");
            Serial.println(dhcpIPAddressLeaseTime_sec);
#endif
        }
#endif
        void dhcpLeaseTimeCopy(DHCP *);
        int32_t dhcp_release(DHCP *dhcp, DHCP_PACKET *dhcp_packet);
        void startLANController(void);
#if 0
        {
            ER  ercd;
            ercd = lan_open();
#ifdef T4_ETHER_DEBUG
            Serial.print("lan_open() = ");
            Serial.println(ercd);
#endif
            if (ercd != E_OK){
                while(1);
            }
        }
#endif
#if 0
        void initialize_TCP_IP(void)
        {
            UW          size;
            ER          ercd;

            size = tcpudp_get_ramsize();
#ifdef T4_ETHER_DEBUG
            Serial.print("tcpudp_get_ramsize() = ");
            Serial.println(size);
#endif
            if (size > (sizeof(tcpudp_work))){
                while(1);
            }
            ercd = tcpudp_open(tcpudp_work);
#ifdef T4_ETHER_DEBUG
            Serial.print("tcpudp_open() = ");
            Serial.println(ercd);
#endif
            if (ercd != E_OK){
                while(1);
            }
        }
#endif

	public:
        EthernetClass(){
#if 0
            dhcpIPAddressLeaseTime_sec = 0;
            dhcpUse = false;
            dhcpIPuse_sec = 0;
            fromSystemGetLastTime = 0;
            tcp_acp_cep_call_flg = 0;
#endif
		}
        virtual ~EthernetClass(){}
        int  begin(byte* mac);
		void begin(byte* mac, IPAddress local_ip);
		void begin(byte* mac, IPAddress local_ip, IPAddress dns_server);
		void begin(byte* mac, IPAddress local_ip, IPAddress dns_server, IPAddress gateway);
		void begin(byte* mac, IPAddress local_ip, IPAddress dns_server, IPAddress gateway, IPAddress subnet);
		IPAddress localIP(void);
		IPAddress subnetMask(void);
		IPAddress gatewayIP(void);
		IPAddress dnsServerIP(void);
		int maintain(void);
		void maininit(void);
        void mainloop(void);
#if 0
        {
            R_ETHER_LinkProcess();
            if(dhcpUse){
                uint32_t    sec,ms;
                ms = millis();
                sec = (ms - Ethernet.fromSystemGetLastTime)/1000;
                if(sec){
                    Ethernet.fromSystemGetLastTime = ms;
                    Ethernet.dhcpIPuse_sec += sec;
                }
            }
        }
#endif
        size_t write(uint8_t){
            return 1;
        }
        size_t write(const uint8_t *buffer, size_t size){
            return 1;
        }
        uint32_t    dhcpIPuse_sec;
        uint32_t    fromSystemGetLastTime;

	protected:
#if 0
        union _dhcp_sv_ip{
            uint8_t bytes[IP_ALEN];
            uint32_t dword;
        } dhcpSvIp;

        union _dhcp_sv_mac{
                uint8_t bytes[EP_ALEN+2];
                uint64_t lword;
        } dhcpSvMac;
        void stop(void);
        bool get_tcp_acp_cep_call_flg(){
            return Ethernet.tcp_acp_cep_call_flg;
        }
        void set_tcp_acp_cep_call_flg(){
            Ethernet.tcp_acp_cep_call_flg = true;
        }
        void clr_tcp_acp_cep_call_flg(){
            Ethernet.tcp_acp_cep_call_flg = false;
        }
#endif
};

class EthernetServer : public EthernetClass{
	private:
		uint16_t _port;
	
	public:
		EthernetServer(){
		    _port = 0;
		}
        EthernetServer(uint16_t port);
		virtual ~EthernetServer(){}
		size_t write(){return 0;};
        size_t write(uint8_t b);
        size_t write(const uint8_t *buffer, size_t size);
		void begin(void);
		void begin(uint16_t port){
			_port = port;
			begin();
		}
		EthernetClient available(void);
        using Print::write;
        size_t println(const char c[]){     /* get the length of the string * s return. '\ 0' is not included in length. */
            size_t  n = strlen(c);
            char ch[n+3];
            strcpy(ch,c);
            ch[n+0] = '\r';
            ch[n+1] = '\n';
            ch[n+2] = 0;
            print(ch);
            n += 2;
            return n;
        }
        size_t println(void){
            char ch[3];
            ch[0] = '\r';
            ch[1] = '\n';
            ch[2] = 0;
            print(ch);
            return 2;
        }


	protected:
#if 0
        int16_t t4_set_tcp_crep(ID repid, UH portno);
        {
            if (repid == 0 || repid > __tcprepn){
                return -1;
            }
            tcp_crep[repid-1].myaddr.portno = portno;
            return 0;
        }
        int16_t t4_set_tcp_ccep(ID cepid, UB ch, INT rbufsz);
        {
            if (cepid == 0 || cepid > __tcpcepn){
                return -1;
            }
            tcp_ccep[cepid-1].cepatr = ch;
            tcp_ccep[cepid-1].rbufsz = rbufsz;
            return 0;
        }
#endif
};

class EthernetClient : public EthernetServer{
    public:
      EthernetClient(){
      }
      virtual ~EthernetClient(){}
      int read(void);
      int read(uint8_t *buf, size_t size);
      int8_t connected(void);
      int connect(IPAddress ip, uint16_t port);
      int connect(const char *host, uint16_t port);
      int available();
      void flush();
      void stop();
      operator bool(){
          return connected();
      }
      bool operator==(const bool value){
#ifdef T4_ETHER_DEBUG
          Serial.print("t4:EthernetClient:==:");
          Serial.println(bool() == value);
#endif
          return bool() == value;
      }
      bool operator!=(const bool value){
#ifdef T4_ETHER_DEBUG
          Serial.print("t4:EthernetClient:!=:");
          Serial.println(bool() != value);
#endif
          return bool() != value;
      }
      bool operator==(const EthernetClient& rhs){
#ifdef T4_ETHER_DEBUG
          Serial.println("t4:EthernetClient:==:true");
#endif
          return true;
      }
      bool operator!=(const EthernetClient& rhs){
#ifdef T4_ETHER_DEBUG
          Serial.print("t4:EthernetClient:!=:");
          Serial.println(!this->operator==(rhs));
#endif
          return !this->operator==(rhs);
      }
    private:
};

class EthernetUDP : public EthernetClass{
    private:
#if 0
      uint16_t  _port;
      uint16_t  _offset;
      int       _remaining;

      T_IPV4EP  _sendIPV4EP;
      uint8_t   _sendBuf[UDP_RCV_DAT_DATAREAD_MAXIMUM+1];
      uint8_t   _recvBuf[UDP_RCV_BUFFER_SIZE];
#endif

    public:
      EthernetUDP();
#if 0
      {
          _remaining = 0;
          _port=0;
          _offset=0;

          _sendIPV4EP.ipaddr = 0;
          _sendIPV4EP.portno = 0;
      }
#endif
      virtual ~EthernetUDP(){}
      uint8_t begin(uint16_t);
      void stop();

      int beginPacket(IPAddress ip, uint16_t port);
      int endPacket();
      size_t write(uint8_t);
      size_t write(const uint8_t *buffer, size_t size);

      using Print::write;

      int parsePacket();
      int available();
      int read();
      int read(unsigned char* buffer, size_t len);
#if 0
      int read(char* buffer, size_t len) { return read((unsigned char*)buffer, len); };
#else
      int read(char* buffer, size_t len);
#endif

      IPAddress remoteIP();
      uint16_t remotePort();
};

#endif/*_ETHERNET_H_*/
