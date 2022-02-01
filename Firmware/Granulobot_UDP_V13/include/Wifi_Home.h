/*
  Wifi local network parameters
  Created by B Saintyves, December 2021.
*/

#ifndef Wifi_Home_h
#define Wifi_Home_h

/* WiFi network name and password */
const char * ssid = "HOME-26E2";
const char * pwd = "baked7744borrow";

// IP address to send UDP data to.
// it can be ip address of the server or 
// a network broadcast address
const char * udpAddress = "10.0.0.255";//router home

//UDP port
const int localPort = 6808;//4210;
const int outPort = 6807;

#endif