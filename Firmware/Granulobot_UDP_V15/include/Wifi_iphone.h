/*
  Wifi local network parameters
  Created by B Saintyves, December 2021.
*/

#ifndef Wifi_iphone_h
#define Wifi_iphone_h

/* WiFi network name and password */
const char * ssid = "Baudouin's iPhone";
const char * pwd = "12345678";

// IP address to send UDP data to.
// it can be ip address of the server or 
// a network broadcast address
//const char * udpAddress = "192.168.1.3";//netgear router broadcast
//char * udpAddress = "192.168.1.255";//netgear router broadcast
char udpAddress[50];
char * udpAddress0 = "169.254.157.255";// init with broadcast address

//UDP port
const int localPort = 6808;//4210;
const int outPort = 6807;

#endif