/* Copyright (C) 2012 mbed.org, MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include "mros_types.h"
#include "mros_comm_cimpl.h"
#include "mros_sys_config.h"
#ifndef TARGET_ATHRILL
#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include "syssvc/serial.h"
#include "syssvc/syslog.h"

#include "kernel_cfg.h"
#include "syssvc/logtask.h"

#include "lwip/inet.h"
#include "lwip/netif.h"
#include "netif/etharp.h"
#include "lwip/dhcp.h"
#include "eth_arch.h"
#include "lwip/tcpip.h"
/*
 * mbed_interface.h
 */
extern void mbed_mac_address(char *mac);
#endif /* TARGET_ATHRILL */

#include <string.h>
#include <stdio.h>

mRosCommConfigType mros_comm_config;

void mros_comm_inet_local_sockaddr_init(mRosSockAddrInType *addr, mros_int32 port)
{
    memset(addr, 0, sizeof(mRosSockAddrInType));

    addr->sin_family = AF_INET;
    addr->sin_port = htons(port);
    addr->sin_addr.s_addr = INADDR_ANY;
	return;
}

mRosReturnType mros_comm_inet_get_ipaddr(const char *hostname, mros_uint32 *ipaddr)
{
	mros_int32 result;
	mros_uint8 addr_array[5];
    mros_uint8 *paddr = addr_array;

    result = sscanf(hostname, "%hhu.%hhu.%hhu.%hhu",
    		(mros_uint8*)&addr_array[0],
			(mros_uint8*)&addr_array[1],
			(mros_uint8*)&addr_array[2],
			(mros_uint8*)&addr_array[3]);

    if (result != 4) {
    	mRosHostEntType *host_address = mros_comm_gethostbyname(hostname);
        if (host_address == MROS_NULL) {
    		ROS_ERROR("%s %s() %u hostname=%s ret=%d", __FILE__, __FUNCTION__, __LINE__, hostname, MROS_E_INVAL);
        	return MROS_E_INVAL;
        }
        paddr = (mros_uint8*)host_address->h_addr_list[0];
    }
    memcpy((void*)ipaddr, (void*)paddr, 4U);
    return MROS_E_OK;
}

void mros_comm_inet_remote_sockaddr_init(mRosSockAddrInType *addr, mros_int32 port, const char* ipaddrp)
{
	mros_int32 result;
	mros_uint8 addr_array[5];
    memset(addr, 0, sizeof(mRosSockAddrInType));
    mros_uint8 *paddr = addr_array;

    addr->sin_family = AF_INET;
    addr->sin_port = htons(port);
    result = sscanf(ipaddrp, "%hhu.%hhu.%hhu.%hhu",
    		(mros_uint8*)&addr_array[0],
			(mros_uint8*)&addr_array[1],
			(mros_uint8*)&addr_array[2],
			(mros_uint8*)&addr_array[3]);

    if (result != 4) {
    	mRosHostEntType *host_address = mros_comm_gethostbyname(ipaddrp);
        if (host_address == MROS_NULL) {
    		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_INVAL);
        	return;
        }
        paddr = (mros_uint8*)host_address->h_addr_list[0];
    }

    memcpy((void*)&addr->sin_addr.s_addr, (void*)paddr, 4U);
	return;
}
void mros_comm_inet_remote_sockaddr_ip32_init(mRosSockAddrInType *addr, mros_int32 port, mros_uint32 ipaddr)
{
    memset(addr, 0, sizeof(mRosSockAddrInType));

    addr->sin_family = AF_INET;
    addr->sin_port = htons(port);
    memcpy((void*)&addr->sin_addr.s_addr, (void*)&ipaddr, 4U);
	return;
}

void mros_comm_timeval_set(mros_uint32 sec, mros_uint32 usec, mRosTimeValType *tv)
{
	tv->tv_sec = sec;
	tv->tv_usec = usec;
	return;
}

#if(MROS_NODE_USE_DHCP == 0)
	#define IP_ADDRESS  	(MROS_NODE_IPADDR)	/*IP address */
	#define SUBNET_MASK		(MROS_NODE_SUBNET_MASK)	/*Subset mask */
	#define DEFAULT_GATEWAY	("")	/*Default gateway */
#endif

static mros_boolean use_dhcp;
static char ip_addr[17] = "\0";
static char mac_addr[19];
static char gateway[17] = "\0";
static char networkmask[17] = "\0";

static char* ethernet_getMACAddress() {
    return mac_addr;
}

static char* ethernet_getIPAddress() {
    return ip_addr;
}

static char* ethernet_getGateway() {
    return gateway;
}

static char* ethernet_getNetworkMask() {
    return networkmask;
}
static void mros_comm_config_init(void)
{
	int ret;

	mros_comm_config.use_dhcp = use_dhcp;
	mros_comm_config.mros_node_ipaddr = ethernet_getIPAddress();
	mros_comm_config.mros_gateway = ethernet_getGateway();
	mros_comm_config.mros_mac_addr = ethernet_getMACAddress();
	mros_comm_config.mros_networkmask = ethernet_getNetworkMask();
	ret = snprintf(&mros_comm_config.mros_uri_slave[0], MROS_URI_SLAVE_LEN, "http://%s:%u",
			mros_comm_config.mros_node_ipaddr, MROS_SLAVE_PORT_NO);
	if (ret < 0) {
		ROS_ERROR("ERROR: can not convert mros uri for slave\n");
	}
}


#ifndef TARGET_ATHRILL
/* TCP/IP and Network Interface Initialisation */
static struct netif netif;


static void tcpip_init_done(void *arg) {
	//    tcpip_inited.release();
	sig_sem(TCPIP_INITED);
}

static void netif_link_callback(struct netif *netif) {
    if (netif_is_link_up(netif)) {
        // netif_linked.release();
		sig_sem(NETIF_LINKED);
    }
}

static void netif_status_callback(struct netif *netif) {
    if (netif_is_up(netif)) {
        strcpy(ip_addr, inet_ntoa(netif->ip_addr));
        strcpy(gateway, inet_ntoa(netif->gw));
        strcpy(networkmask, inet_ntoa(netif->netmask));
        // netif_up.release();
		sig_sem(NETIF_UP);
    }
}

static void init_netif(ip_addr_t *ipaddr, ip_addr_t *netmask, ip_addr_t *gw) {
    tcpip_init(tcpip_init_done, NULL);
	// tcpip_inited.wait();
	wai_sem(TCPIP_INITED);

    memset((void*) &netif, 0, sizeof(netif));
    netif_add(&netif, ipaddr, netmask, gw, NULL, eth_arch_enetif_init, tcpip_input);
    netif_set_default(&netif);

    netif_set_link_callback  (&netif, netif_link_callback);
    netif_set_status_callback(&netif, netif_status_callback);
}

static void set_mac_address(void) {
#if (MBED_MAC_ADDRESS_SUM != MBED_MAC_ADDR_INTERFACE)
    snprintf(mac_addr, 19, "%02x:%02x:%02x:%02x:%02x:%02x", MBED_MAC_ADDR_0, MBED_MAC_ADDR_1, MBED_MAC_ADDR_2,
             MBED_MAC_ADDR_3, MBED_MAC_ADDR_4, MBED_MAC_ADDR_5);
#else
    char mac[6];
    mbed_mac_address(mac);
    snprintf(mac_addr, 19, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
#endif
}

#if (MROS_NODE_USE_DHCP == 1)
static int ethernet_init(void)
{
    use_dhcp = true;
    set_mac_address();
    init_netif(NULL, NULL, NULL);
    return 0;
}
#endif

static int ethernet_ip_init(const char* ip, const char* mask, const char* gateway)
{
    use_dhcp = false;

    set_mac_address();
    strcpy(ip_addr, ip);

    ip_addr_t ip_n, mask_n, gateway_n;
    inet_aton(ip, &ip_n);
    inet_aton(mask, &mask_n);
    inet_aton(gateway, &gateway_n);
    init_netif(&ip_n, &mask_n, &gateway_n);

    return 0;
}

static int ethernet_connect(unsigned int timeout_ms) {
    eth_arch_enable_interrupts();

    int inited;
    if (use_dhcp) {
        dhcp_start(&netif);
        // Wait for an IP Address
        // -1: error, 0: timeout
		// inited = netif_up.wait(timeout_ms);
		inited = twai_sem(NETIF_UP, timeout_ms);
    } else {
        netif_set_up(&netif);
        // Wait for the link up
        // inited = netif_linked.wait(timeout_ms);
		inited = twai_sem(NETIF_LINKED, timeout_ms);
    }
	return (inited == E_OK) ? (0) : (-1);
	//    return (inited > 0) ? (0) : (-1);
}


static void network_init(){
#if (MROS_NODE_USE_DHCP == 1)
	if(ethernet_init() != 0) {
#else
	if(ethernet_ip_init(IP_ADDRESS, SUBNET_MASK, DEFAULT_GATEWAY) != 0) {
#endif
		syslog(LOG_NOTICE, "Network Initialize Error\r\n");
	return;
	}
		syslog(LOG_NOTICE, "Network Initialized successfully");
	while (ethernet_connect(5000) != 0){
		syslog(LOG_NOTICE, "LOG_NOTICE: Network Connect Error");
	}
	syslog(LOG_NOTICE,"MAC Address is %s\r\n", ethernet_getMACAddress());
	syslog(LOG_NOTICE,"IP Address is %s\r\n", ethernet_getIPAddress());
	syslog(LOG_NOTICE,"NetMask is %s\r\n", ethernet_getNetworkMask());
	syslog(LOG_NOTICE,"Gateway Address is %s\r\n", ethernet_getGateway());
	mros_comm_config_init();
	return;
}
#else
static void network_init(void)
{
	int len = strlen(MROS_NODE_IPADDR);
	memcpy(ip_addr, MROS_NODE_IPADDR, len);
	ip_addr[len] = '\0';
	mros_comm_config_init();
	lwip_init();
	return;
}
#endif /* TARGET_ATHRILL */


void mros_comm_init(void)
{
	network_init();
	return;
}
mros_int32 mros_comm_accept(mros_int32 s, mRosSockAddrType *addr, mRosSizeType *addrlen)
{
	return lwip_accept(s, addr, (socklen_t*)addrlen);
}
mros_int32 mros_comm_bind(mros_int32 s, const mRosSockAddrType *name, mRosSizeType namelen)
{
	return lwip_bind(s, name, namelen);
}
mros_int32 mros_comm_shutdown(mros_int32 s, mros_int32 how)
{
	return lwip_shutdown(s, how);
}
mros_int32 mros_comm_getpeername (mros_int32 s, mRosSockAddrType *name, mRosSizeType *namelen)
{
	return lwip_getpeername(s, name, (socklen_t*)namelen);
}
mros_int32 mros_comm_getsockname (mros_int32 s, mRosSockAddrType *name, mRosSizeType *namelen)
{
	return lwip_getsockname(s, name, (socklen_t*)namelen);
}
mros_int32 mros_comm_getsockopt (mros_int32 s, mros_int32 level, mros_int32 optname, void *optval, mRosSizeType *optlen)
{
	return lwip_getsockopt(s, level, optname, optval, (socklen_t*)optlen);
}
mros_int32 mros_comm_setsockopt (mros_int32 s, mros_int32 level, mros_int32 optname, const void *optval, mRosSizeType optlen)
{
	return lwip_setsockopt(s, level, optname, optval, optlen);
}
mros_int32 mros_comm_close(mros_int32 s)
{
	return lwip_close(s);
}
mros_int32 mros_comm_connect(mros_int32 s, const mRosSockAddrType *name, mRosSizeType namelen)
{
	return lwip_connect(s, name, namelen);
}
mros_int32 mros_comm_listen(mros_int32 s, mros_int32 backlog)
{
	return lwip_listen(s, backlog);
}
mros_int32 mros_comm_recv(mros_int32 s, void *mem, mRosSizeType len, mros_int32 flags)
{
	return lwip_recv(s, mem, len, flags);
}
mros_int32 mros_comm_read(mros_int32 s, void *mem, mRosSizeType len)
{
	return lwip_read(s, mem, len);
}
mros_int32 mros_comm_recvfrom(mros_int32 s, void *mem, mRosSizeType len, mros_int32 flags, mRosSockAddrType *from, mRosSizeType *fromlen)
{
	return lwip_recvfrom(s, mem, len, flags, from, (socklen_t*)fromlen);
}
mros_int32 mros_comm_send(mros_int32 s, const void *dataptr, mRosSizeType size, mros_int32 flags)
{
	return lwip_send(s, dataptr, size, flags);
}
mros_int32 mros_comm_sendto(mros_int32 s, const void *dataptr, mRosSizeType size, mros_int32 flags, const mRosSockAddrType *to, mRosSizeType tolen)
{
	return lwip_sendto(s, dataptr, size, flags, to, tolen);
}
mros_int32 mros_comm_socket(mros_int32 domain, mros_int32 type, mros_int32 protocol)
{
	return lwip_socket(domain, type, protocol);
}
mros_int32 mros_comm_write(mros_int32 s, const void *dataptr, mRosSizeType size)
{
	return lwip_write(s, dataptr, size);
}
mros_int32 mros_comm_select(mros_int32 maxfdp1, mRosFdSetType *readset, mRosFdSetType *writeset, mRosFdSetType *exceptset, mRosTimeValType *timeout)
{
	return lwip_select(maxfdp1, readset, writeset, exceptset, timeout);
}
mros_int32 mros_comm_ioctl(mros_int32 s, mros_int32 cmd, void *argp)
{
	return lwip_ioctl(s, cmd, argp);
}
mros_int32 mros_comm_fcntl(mros_int32 s, mros_int32 cmd, int val)
{
	return lwip_fcntl(s, cmd, val);
}
mRosHostEntType *mros_comm_gethostbyname(const char *name)
{
	return lwip_gethostbyname(name);
}
