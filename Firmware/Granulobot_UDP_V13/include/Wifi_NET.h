/*
  Wifi local network parameters
  Created by B Saintyves, December 2021.
*/

#ifndef Wifi_NET_h
#define Wifi_NET_h

/* WiFi network name and password */
const char * ssid = "NETGEAR21";
const char * pwd = "strongmoon546";

// IP address to send UDP data to.
// it can be ip address of the server or 
// a network broadcast address
//const char * udpAddress = "192.168.1.3";//netgear router broadcast
const char * udpAddress = "192.168.1.255";//netgear router broadcast

//UDP port
const int localPort = 6808;//4210;
const int outPort = 6807;

#endif