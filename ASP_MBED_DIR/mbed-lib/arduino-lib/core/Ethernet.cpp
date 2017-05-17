/* Ethernet.cpp */
/* Copyright (C) 2016 Nozomu Fujita, MIT License
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
#include <Arduino.h>
#include <Ethernet.h>
#include <EthernetInterface.h>
#include <api/mbed_interface.h>
#include <lwip/dns.h>
#include <stdio.h>

static EthernetInterface eth;
static TCPSocketConnection sock;

static void setMacAddress(byte* mac)
{
    char* mac_addr = eth.getMACAddress();
    if (mac_addr != NULL) {
        snprintf(mac_addr, 19, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
}

int EthernetClass::begin(byte* mac)
{
    setMacAddress(mac);
    if (eth.init() < 0) return 0;
    if (eth.connect() < 0) return 0;
    return 1;
}

void EthernetClass::begin(byte* mac, IPAddress local_ip)
{
    IPAddress dns_server(local_ip[0], local_ip[1], local_ip[2], 1);
    begin(mac, local_ip, dns_server);
}

void EthernetClass::begin(byte* mac, IPAddress local_ip, IPAddress dns_server)
{
    IPAddress gateway(local_ip[0], local_ip[1], local_ip[2], 1);
    begin(mac, local_ip, dns_server, gateway);
}

void EthernetClass::begin(byte* mac, IPAddress local_ip, IPAddress dns_server, IPAddress gateway)
{
    IPAddress subnet(255, 255, 255, 0);
    begin(mac, local_ip, dns_server, gateway, subnet);
}

// under construction
void EthernetClass::begin(byte* mac, IPAddress local_ip, IPAddress dns_server, IPAddress gateway, IPAddress subnet)
{
#if 0 // under construction
    setMacAddress(mac);

    char _ip[3 + 1 + 3 + 1 + 3 + 1 + 3 + 1];
    char _mask[3 + 1 + 3 + 1 + 3 + 1 + 3 + 1];
    char _gateway[3 + 1 + 3 + 1 + 3 + 1 + 3 + 1];
    sprintf(_ip, "%u.%u.%u.%u", local_ip[0], local_ip[1], local_ip[2], local_ip[3]);
    sprintf(_mask, "%u.%u.%u.%u", subnet[0], subnet[1], subnet[2], subnet[3]);
    sprintf(_gateway, "%u.%u.%u.%u", gateway[0], gateway[1], gateway[2], gateway[3]);
Serial.print("_ip = "); Serial.println(_ip);
Serial.print("_mask = "); Serial.println(_mask);
Serial.print("_gateway = "); Serial.println(_gateway);
    eth.init(_ip, _mask, _gateway);

    char _dns[3 + 1 + 3 + 1 + 3 + 1 + 3 + 1];
    sprintf(_dns, "%u.%u.%u.%u", dns_server[0], dns_server[1], dns_server[2], dns_server[3]);
    ip_addr_t _dnsserver = {uint32_t(dns_server)};
Serial.print("_dns = "); Serial.println(_dns);
    dns_setserver(0, &_dnsserver);
#endif
}

IPAddress EthernetClass::localIP()
{
    int first_octet;
    int second_octet;
    int third_octet;
    int fourth_octet;
    sscanf(eth.getIPAddress(), "%d.%d.%d.%d", &first_octet, &second_octet, &third_octet, &fourth_octet);
    IPAddress ip(first_octet, second_octet, third_octet, fourth_octet);
    return ip;
}

size_t EthernetServer::write(uint8_t b)
{
    return sock.send((char*)&b, 1);
}

size_t EthernetServer::write(const uint8_t *buffer, size_t size)
{
    return sock.send_all((char*)buffer, size);
}

int EthernetClient::connect(const char *host, uint16_t port)
{
    return sock.connect(host, port) == 0 ? 1 : -4;
}

int EthernetClient::connect(IPAddress ip, uint16_t port)
{
    char host[3 + 1 + 3 + 1 + 3 + 1 + 3 + 1];
    sprintf(host, "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
    return connect(host, port);
}

int8_t EthernetClient::connected(void)
{
    return sock.is_connected();
}

void EthernetClient::stop()
{
    sock.close();
    eth.disconnect();
}

// under construction
static char data = -1;
int EthernetClient::available()
{
    if (data != -1) {
        return 1;
    } else if (sock.receive(&data, 1) >= 1) {
        return 1;
    } else {
        return 0;
    }
}

// under construction
int EthernetClient::read()
{
    int d;
    if (data != -1) {
        d = data;
    } else if (sock.receive(&data, 1) >= 1) {
        d = data;
    } else {
        d = -1;
    }
    data = -1;
    return d;
}

// under construction
void EthernetClient::flush()
{
}

EthernetClass Ethernet;
