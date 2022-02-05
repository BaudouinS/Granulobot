/*
  Wifi local network parameters
  Created by B Saintyves, December 2021.
*/

#ifndef Wifi_Marc_h
#define Wifi_Marc_h

/* WiFi network name and password */
const char * ssid = "gbots";
const char * pwd = "gbotsRus";

// IP address to send UDP data to.
// it can be ip address of the server or 
// a network broadcast address
const char * udpAddress = "192.168.0.255"; //broadcasting address= the program will read from this address

//UDP port
const int localPort = 6808;//4210;
const int outPort = 6807;

#endif