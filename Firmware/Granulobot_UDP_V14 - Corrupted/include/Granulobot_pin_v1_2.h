/*
  Granulobot_pin_v1-2_h - PINOUTS for Granulobot PCB V1.2 Nov2021 
  Created by B Saintyves, December 15th, 2021.
  
  Update in V1.2 FROM v1.1:
  * AIN1 moved from IO25 to IO32
  * 2 pads added for DAC out 1 and DAC out 2 (IO25 and IO26) + RTC-IO6/RTC-IO7
  * ENC_OUTA moved from IO5 to IO4 for access to RTCIO10 in sleep mode and ADC
  * 2 pads added for Motor B IN 1&2
  * 2 pads added for Motor B OUT 1&2
  * 2 pads added for GPIO 37&38
  * 2 X 2 pads added for +3.3V&GND
  * 2 pads added for SCL&SDA access (I2C)
  * Switch reset path rerouted
  * Pads spacing adjusted for arduino nano type pin header
  * SLEEP pin (motor enable) moved from GPIO18 to GPIO16 (to free up SPI pins for futur update)
*/

#ifndef Granulobot_pin_v1_2_h
#define Granulobot_pin_v1_2_h

////FIXED OUTPUT-INPUT
// Battery
#define BAT_CHARGE 34
#define BAT_VOLTAGE 35

//Encoder
#define ENCODER_A 4//RTC IO 10
#define ENCODER_B 14//RTC IO 16

//MOTOR
// Define the pin numbers on which the outputs are generated.
#define MOT_A1_PIN 32
#define MOT_A2_PIN 27
#define MOT_SLP_PIN 16

//Boost 3.7 to 6V
#define EN_BOOST 33//SLEEP pin on BOOST
/////////////////////////////////////

/////EXTRA OUTPUT-INPUT
//IO + DAC + RTC
#define DAC_1 25//RTC-IO 6
#define DAC_2 26//RTC-IO 7
//REGULAR IO
#define IO1 37
#define IO2 38

#endif