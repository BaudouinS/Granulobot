/*Granulobot OSC Firmware V10
 * Encoder position measurement and conversion in angle and angular velocity
 * Magnet's magnetic field measurement
 * accel + gyr measurement
 * PWM Control of N20 geared motors
 * 2 ways UDP wifi communication with broadcasting address
 * PID on position encoder  https://github.com/DonnyCraft1/PIDArduino
 * https://circuitdigest.com/microcontroller-projects/arduino-based-encoder-motor-using-pid-controller
 *  OTA wireless firmware upload: https://randomnerdtutorials.com/esp32-over-the-air-ota-programming/

*      autoselect particule address from database defined in header 
*      use mac addred for ID number instead of non static IP {MAC address(last digits),paticule adress number} 
*      
*New on V10:
*      PID on speed
*      for new OSC interface Granulobot_V9.JSON
*      
*components controlled by the firmware:
*
*accel+gyro: LSM6DSOX 
*magnetic sensor: Tlv493d (+-150mT)
*magnetic encoder: Pololu
*Motor driver: DRV8833
*Microcontroller: ESP32
*StepUp motor: TPS61178 (with Enable Pin)
*
*OpenStageControl ressources:
*https://openstagecontrol.ammd.net/docs/custom-module/custom-module/
*
*OSC addresses:
    Particule:
      "/" + String(Motor.Address) + "/Bat" //Battery indicator (passive fader)
      "/" + String(Motor.Address) + "/PWM" //PWM value indicator (passive fader) for force proxy
      "/" + String(Motor.Address) + "/IP"  // particule IP address
      
      "/" + String(Motor.Address) + "/F/Vs" //Speed shift fader and indicator
      "/" + String(Motor.Address) + "/F"
      "/" + String(Motor.Address) + "/F/V"
      
      "/" + String(Motor.Address) + "/S1" //modes Soft,Stiff,CW,CCW
      "/" + String(Motor.Address) + "/S2"
      "/" + String(Motor.Address) + "/S3"
      "/" + String(Motor.Address) + "/S4"
    
      "/" + String(Motor.Address) + "/P"); //Push button to turn incrementally(slowly) clockwise
      "/" + String(Motor.Address) + "/M"   //Push button to turn incrementally(slowly) counter clockwise
      
      "/" + String(Motor.Address) + "/Sto"
      "/" + String(Motor.Address) + "/Po"  //Turn on blue led to see particule
    
      "/" + String(Motor.Address) + "/A"   //  knob indicator (passive) for motor angle
      "/" + String(Motor.Address) + "/G"   //  knob indicator (passive)for angle of particule ref(USB plug down) with gravity
  
    General
      "/P1"
      "/P2"
      "/P3"
      "/P4"
      "/F"
      "/F/V"
      "/Sto"
      "/Sl"
      "/U"
************************
*
*by Baudouin Saintyves
*April 30th 2021
************************/

////////////////////////////////////
/////USER PARAMETERS//////////////////

/* WiFi network name and password */

//dedicated local network
const char * ssid = "TP-Link_C1BA";
const char * pwd = "93409582";
// Broadcast IP address to send UDP data to.
const char * udpAddress = "192.168.0.255"; //broadcasting address= the program will read from this address

//Home wifi
//const char * ssid = "HOME-26E2";
//const char * pwd = "baked7744borrow";
//// Broadcast IP address to send UDP data to.
//const char * udpAddress = "10.0.0.255"; //broadcasting address= the program will read from this address


const int localPort = 8000;//4210;
const int outPort = 9000;
//byte local[6];   //store mac address

////add here new two last characters of Mac Address. //Particule number 
 char *ADDRESS_DATA_BASE[] = {  //
    "20",//1
    "54",//2
    "4C",//3
    "38",//4
    "78",//5
    "E8",//6
    "60",//7
    "FC",//8
    "08",//9
    "A4",//10
    "A0",//11
    "E4",//12
    "9C",//13
    //add new particule here
    "00"
    };
  
//choose here the motor's gear ratio
 const int ratio=380;

//choose here the motor's PID parameters
#define __Kp 260//260 // Proportional constant
#define __Ki 2.7 // Integral Constant
//position 
#define __Kd 2000//2000 // Derivative Constant
//speed 
//#define __Kds 0//2000 // Derivative Constant
#define __Kds 2000//2000 // Derivative Constant

//choose here motor's PWM frequency
#define FREQ 100
////////////////////////////////////
////////////////////////////////////

////////////////////////////////////
/////LIBRARIES//////////////////////

 //Arduino
#include <Arduino.h>

  //Wifi
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h> //library for Over The Air programming

  //sensors
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DSOX.h>//accel+gyro
#include <Tlv493d.h>//magnetic field sensor

  //parsing for serial com
#include <Regexp.h>

  //TinyPico Helper Library
#include <TinyPICO.h>

   //OSC library
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCData.h>

  //PID library
#include <PIDController.h> // https://github.com/DonnyCraft1/PIDArduino

  //Math
#include <math.h>

  
////////////////////////////////////
/////PIN DEFINE/////////////////////

// Battery
#define BAT_CHARGE 34
#define BAT_VOLTAGE 35

//Encoder
#define ENCODER_A 5
#define ENCODER_B 14

//MOTOR
// Define the pin numbers on which the outputs are generated.
#define MOT_A1_PIN 25//15
#define MOT_A2_PIN 27//14
#define MOT_SLP_PIN 18

//Boost 3.7 to 6V
#define EN_BOOST 33//SLEEP pin on BOOST
/////////////////////////////////////

////////////////////////////////////
/////VARIABLES//////////////////////



////particule address number
//int ADDRESS_NUM;

 //Timers
unsigned long previousMillis = 0; // store previous millis readout (for PID)
unsigned long previousMillis2 = 0; // store previous millis readout (to send motor.angle)
unsigned long previousMillis3 = 0; // store previous millis readout (to send Bat level)
unsigned long previousMillis4 = 0; // store previous millis readout (to send Bat level)
unsigned long previousMillis5 = 0; // store previous millis readout (to send Bat level)
unsigned long previousMillis6 = 0; // store previous millis readout (to send Bat level)
unsigned long previousMillis7 = 0; // store previous millis readout (to send Bat level)

//Encoder
volatile int encoder_count=0;
int encoder_count_prev=0;
int encoder_start;
volatile bool CW;
volatile bool Soft;
const int tour=ratio*12;
const int Res_speed=30;//in ms
int prev_pos=0;
long int speed_tick;
float speed_rad;
int angle_tick=0;
float angle_rad=0;
int speed_target=0;
int direct=0;
int previous=0;
int direct_rec=0;

unsigned int angle_target = 0; // stores the incoming serial value. Max value is 65535
char incomingByte; // parses and stores each individual character one by one
//int motor_pwm_value = 255; // after PID computation data is stored in this variable.


struct Button {
  const uint8_t PIN;
  //uint32_t encoder_count;
  bool detect;
};

Button MA = {ENCODER_A, false};//Encoder OUT A
Button MB = {ENCODER_B, false};//Encoder OUT B


//Motor control

//global variables
bool MoveStop=false;
// setting PWM properties
const int freq = FREQ;
const int MotChannel1 = 1;
const int MotChannel2 = 2;
const int resolution = 8;

//for PID
//int motor_pwm_value = 0; // after PID computation data is stored in this variable.
unsigned int integerValue = 0;

bool Poke=false;

//for OSC Com
typedef struct Message {
  int Address;
  int Mode;
  int Torque;
  int Speed;
  int Speed_target;
  int Vshift;
  int State;
  //int Displacement;
  int Angle;
  int Angle_target;
  bool Direction;//true=CW or false=CCW
  int PWM; // after PID computation data is stored in this variable.
  bool Minus_State;
  bool Plus_State;
} Message;

int P=0;
int angle_interval;
int test_prev;
int test;

float ThetaG;

Message Motor;

///deep sleep
//#define BUTTON_PIN_BITMASK 0x200000000 // 2^33 in hex / for deep sleep on pin 33 with ext1
RTC_DATA_ATTR int bootCount = 0; // for deep sleep
/////////////////////////////////////

//INITIALISE
 //Initialise sensors
    // LSM6DSOX Object
Adafruit_LSM6DSOX lsm6dsox;
    // Tlv493d Object
Tlv493d Tlv493dMagnetic3DSensor = Tlv493d();
    //create PID object
PIDController pidcontroller;
PIDController pidcontroller_speed;

//create UDP instance
WiFiUDP Udp;
OSCErrorCode error;

// Initialise TinyPICO library
TinyPICO tp = TinyPICO();


////////////////////////////////////
/////FUNCTIONS//////////////////////

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}


/////////////////////////////////////
//attach interrupt functions for encoder
void IRAM_ATTR isr_A() {
if (digitalRead(MA.PIN) == HIGH){
    if (digitalRead(MB.PIN) == HIGH){encoder_count++;CW=true;}
    else {encoder_count--;CW=false;}
    }
else{ if (digitalRead(MB.PIN) == LOW){encoder_count++;CW=true;}
    else {encoder_count--;CW=false;}
    }       
MA.detect = true;
}


void IRAM_ATTR isr_B() {
if (digitalRead(MB.PIN) == HIGH){
    if (digitalRead(MA.PIN) == LOW){encoder_count++;CW=true;}
    else {encoder_count--;CW=false;}
    }
else{
    if (digitalRead(MA.PIN) == HIGH){encoder_count++;CW=true;}
    else {encoder_count--;CW=false;}
    }
MB.detect = true;
}


void setup(){

  log_d("Total heap: %d", ESP.getHeapSize());//core debug level to "verbose"
  log_d("Free heap: %d", ESP.getFreeHeap());
  log_d("Total PSRAM: %d", ESP.getPsramSize());
  log_d("Free PSRAM: %d", ESP.getFreePsram());
  
  Serial.begin(115200);
  Serial.println("RESET");
  Serial.println("Firmware Granulobot OSC V10");


//WIFI UDP SETUP
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  //Connect to the WiFi network
  WiFi.begin(ssid, pwd);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  //This initializes udp and transfer buffer
  Udp.begin(localPort);
  
  ///////


//////ARDUINO OTA 
ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
  /////////////////////////////////////
//  
////  //particule address selection  
std::string EndMAC{WiFi.macAddress()[15],WiFi.macAddress()[16]};
  int index=0;
  for (int i=0; i<(sizeof(ADDRESS_DATA_BASE) / sizeof(ADDRESS_DATA_BASE[0])); i++) {
     if (EndMAC == ADDRESS_DATA_BASE[i]) {
       index = i;
       // Serial.print(local[5],HEX);
        Serial.print((sizeof(ADDRESS_DATA_BASE) / sizeof(ADDRESS_DATA_BASE[0])));Serial.print(" adresses to choose from, index: ");Serial.println(index);    
       break;
     }
  }
  IPAddress local=WiFi.localIP();

//Serial.print("Two last char of MAC address: ");Serial.println(EndMAC);

// 
 //intitialisation of Motor structure
  //Motor.Address=ADDRESS_DATA_BASE[index][1];//ADDRESS
  Motor.Address=index+1;//ADDRESS
  Motor.Mode=0;
  Motor.Speed=0;
  Motor.Speed_target=0;
  Motor.Torque=0;
  Motor.Angle=0;
  Motor.Angle_target=0;
  Motor.Direction=true;
  Motor.State=0;
  Motor.PWM=0;
  Motor.Vshift=0;
  Motor.Minus_State=false;
  Motor.Plus_State=false;
 Serial.print("Motor address:");Serial.println(Motor.Address);
  
/////deep sleep mode setup
++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  /*
  First we configure the wake up source
  We set our ESP32 to wake up for an external trigger.
  There are two types for ESP32, ext0 and ext1 .
  ext0 uses RTC_IO to wakeup thus requires RTC peripherals
  to be on while ext1 uses RTC Controller so doesnt need
  peripherals to be powered on.
  Note that using internal pullups/pulldowns also requires
  RTC peripherals to be turned on.
  */
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_5,1); //1 = High, 0 = Low

  /////////

  tp.DotStar_SetPower(true);
  tp.DotStar_SetPixelColor(0,30,0);
  tp.DotStar_Show();
  delay(2000);
  ///turn off RGB LED power to save battery life
  tp.DotStar_SetPower(false);
  
  
  ////SETUP SENSORS
  //Tlv493d
  Tlv493dMagnetic3DSensor.begin();
  Serial.println("Tlv493d Found!");

    //LSM6DSOX

    lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
    lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS); 
    //lsm6dsox.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
    //lsm6dsox.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);
    lsm6dsox.setAccelDataRate(LSM6DS_RATE_416_HZ);
    lsm6dsox.setGyroDataRate(LSM6DS_RATE_416_HZ);

  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens 

  Serial.println("Adafruit lsm6dsox test!");

  if(!lsm6dsox.begin_I2C(0x6A)){
    Serial.println("Failed first attempt");
    if(!lsm6dsox.begin_I2C(0x6B)){
      Serial.println("Failed to find lsm6dsox chip");
      while (1) delay(10);
    }
  }

  Serial.println("lsm6dsox Found!");

  delay(500);

    //Encoder
  pinMode(MA.PIN, INPUT);
  attachInterrupt(MA.PIN, isr_A, CHANGE);
  pinMode(MB.PIN, INPUT);
  attachInterrupt(MB.PIN, isr_B, CHANGE);
  Serial.print("ticks per tour=");Serial.println(tour);
  ////////


  //ENABLE BOOST
  pinMode(EN_BOOST, OUTPUT);
  digitalWrite(EN_BOOST, HIGH);// Start with Boost on.  

  //MOTOR SETUP
    //Initialize the motor driver switch pin.
  pinMode(MOT_SLP_PIN, OUTPUT);
  digitalWrite(MOT_SLP_PIN, HIGH);// Start with drivers on.
    //PWM setup 
  ledcSetup(MotChannel1, freq, resolution);
  ledcSetup(MotChannel2, freq, resolution);

  ledcAttachPin(MOT_A1_PIN, MotChannel1);
  ledcAttachPin(MOT_A2_PIN, MotChannel2);
  ///////
  
  //PID
  pidcontroller.begin(); // initialize the PID instance
  pidcontroller.tune(__Kp,__Ki,__Kd); // Tune the PID, arguments: kP, kI, kD
  pidcontroller.limit(-255, 255); // Limit the PID output between -255 to 255, this is important to get rid of integral windup!

  //PID speed
  pidcontroller_speed.begin(); // initialize the PID instance
  pidcontroller_speed.tune(__Kp,__Ki,__Kds); // Tune the PID, arguments: kP, kI, kD
  pidcontroller_speed.limit(-255, 255); // Limit the PID output between -255 to 255, this is importa

//// Get a *rough* estimate of the current battery voltage
  // If the battery is not present, the charge IC will still report it's trying to charge at X voltage
  // so it will still show a voltage.
  Serial.print("Battery level: ");
  Serial.println(tp.GetBatteryVoltage());
  Serial.print("Battery charging in progress: ");
  Serial.println(tp.IsChargingBattery());  
  delay(2000);
  Serial.print("battery level= ");Serial.print(((tp.GetBatteryVoltage()-3.5)/0.70)*100);Serial.println("%");
  
////intialize OSC control surface
  
  // particule
  String buf1=String("/" + String(Motor.Address) + "/Bat");
  String buf2=String("/" + String(Motor.Address) + "/PWM");
  String buf3=String("/" + String(Motor.Address) + "/IP");
  
  String buf4=String("/" + String(Motor.Address) + "/F/Vs");
  String buf5=String("/" + String(Motor.Address) + "/F");
  String buf11=String("/" + String(Motor.Address) + "/F/V");
  
  String buf6=String("/" + String(Motor.Address) + "/S1");
  String buf7=String("/" + String(Motor.Address) + "/S2");
  String buf8=String("/" + String(Motor.Address) + "/S3");
  String buf9=String("/" + String(Motor.Address) + "/S4");  

  String buf10=String("/" + String(Motor.Address) + "/A");

  OSCMessage msgOUT_Fc1(buf1.c_str());  
  msgOUT_Fc1.add(((tp.GetBatteryVoltage()-3.5)/0.70)*100);
  Udp.beginPacket(udpAddress,outPort);
  msgOUT_Fc1.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT_Fc1.empty(); // free space occupied by message 

  OSCMessage msgOUT_Fc2(buf2.c_str());  
  msgOUT_Fc2.add(Motor.PWM);
  Udp.beginPacket(udpAddress,outPort);
  msgOUT_Fc2.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT_Fc2.empty(); // free space occupied by message 

  OSCMessage msgOUT_Fc3(buf3.c_str());  
  msgOUT_Fc3.add(local[3]);
  Udp.beginPacket(udpAddress,outPort);
  msgOUT_Fc3.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT_Fc3.empty(); // free space occupied by message 

  OSCMessage msgOUT_Fc4(buf4.c_str());  
  msgOUT_Fc4.add(0);
  Udp.beginPacket(udpAddress,outPort);
  msgOUT_Fc4.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT_Fc4.empty(); // free space occupied by message 

  OSCMessage msgOUT_Fc5(buf5.c_str());  
  msgOUT_Fc5.add(0);
  Udp.beginPacket(udpAddress,outPort);
  msgOUT_Fc5.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT_Fc5.empty(); // free space occupied by message 

  OSCMessage msgOUT_Fc6(buf6.c_str());  
  msgOUT_Fc6.add(0);
  Udp.beginPacket(udpAddress,outPort);
  msgOUT_Fc6.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT_Fc6.empty(); // free space occupied by message 
  
  OSCMessage msgOUT_Fc7(buf7.c_str());  
  delay(2000);
  msgOUT_Fc7.add(0);
  Udp.beginPacket(Udp.remoteIP(),outPort);
  msgOUT_Fc7.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT_Fc7.empty(); // free space occupied by message  
  
  OSCMessage msgOUT_Fc8(buf8.c_str());  
  msgOUT_Fc8.add(0);
  Udp.beginPacket(Udp.remoteIP(),outPort);
  msgOUT_Fc8.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT_Fc8.empty(); // free space occupied by message  

  OSCMessage msgOUT_Fc9(buf9.c_str());  
  msgOUT_Fc9.add(0); 
  Udp.beginPacket(Udp.remoteIP(),outPort);
  msgOUT_Fc9.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT_Fc9.empty(); // free space occupied by message

  OSCMessage msgOUT_Fc10(buf10.c_str());  
  msgOUT_Fc10.add(0); 
  Udp.beginPacket(Udp.remoteIP(),outPort);
  msgOUT_Fc10.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT_Fc10.empty(); // free space occupied by message

  OSCMessage msgOUT_Fc11(buf11.c_str());  
  msgOUT_Fc11.add(0); 
  Udp.beginPacket(Udp.remoteIP(),outPort);
  msgOUT_Fc11.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT_Fc11.empty(); // free space occupied by message
      
}//end setup loop

void loop(){
  ArduinoOTA.handle();
  OSCMsgReceive();

 if (encoder_count>tour-1){encoder_count=encoder_count-tour;}   
 if (encoder_count<0){encoder_count=tour+encoder_count;}

 if (Poke){ 
    tp.DotStar_SetPower(true);
    tp.DotStar_SetPixelColor(0,0,100);
    tp.DotStar_Show();
 }else{tp.DotStar_SetPower(false);}

 if (Motor.State==1){  //if "stiff" selected
     
      if (Motor.Plus_State || Motor.Minus_State){  
           if (Motor.Plus_State){
              pidcontroller_speed.setpoint(5);
           }      
          //Serial.println("+ Pushed");
          //CONTROL_SPEED(pidcontroller_speed, Motor.PWM, 5, encoder_count, encoder_count_prev);                     
           if (Motor.Minus_State){
              pidcontroller_speed.setpoint(-5);
          //Serial.println("- Pushed");
          //CONTROL_SPEED(pidcontroller_speed, Motor.PWM, -5, encoder_count, encoder_count_prev);
           }

           unsigned long currentMillis7 = millis();
           if (currentMillis7 - previousMillis7 >= 3){   //PID computed value updated every 3ms
               previousMillis7 = currentMillis7;    // Save timestamp   
               test=test+1;           
           }
           if (test<300){  
             unsigned long currentMillis = millis();
             if (currentMillis - previousMillis >= 10){   //PID computed value updated every 3ms
                previousMillis = currentMillis;    // Save timestamp   
                Motor.Speed=encoder_count-encoder_count_prev;
                CONTROL_SPEED(pidcontroller_speed, Motor.Speed);          
                encoder_count_prev=encoder_count;
                   }
            }                   
           else{
                  Motor.Plus_State=false;
                  Motor.Minus_State=false;
                  test=0;}
                   
      }else {
          unsigned long currentMillis2 = millis();
          if (currentMillis2 - previousMillis2 >= 3){   //PID computed value updated every 3ms
               previousMillis2 = currentMillis2;    // Save timestamp                      
               pidcontroller.setpoint(encoder_count_prev); // The "goal" the PID controller tries to "reach",
               Motor.PWM = pidcontroller.compute(encoder_count);  //Let the PID compute the value, returns the calculated optimal output                     
               if (Motor.PWM > 0) // if the motor_pwm_value is greater than zero we rotate the  motor in clockwise direction
                 motor_ccw(Motor.PWM);
               else // else we move it in a counter clockwise direction
                 motor_cw(abs(Motor.PWM));
           }
        } 
    } 
   
     if (Motor.State==0){ // soft mode
        MotorOFF();
        encoder_count_prev=encoder_count;
      }

     
     if (Motor.State==2 || Motor.State==3){//SPEED CONTROLLED
          //Serial.println(P);
                         
      //Serial.println("in state Stiff mode");
           if (Motor.State==2){
              pidcontroller_speed.setpoint(Motor.Vshift+Motor.Speed_target);
           }      
                   
           if (Motor.State==3){
              pidcontroller_speed.setpoint((Motor.Vshift+Motor.Speed_target)*(-1));
           }

           if (P==1){
           unsigned long currentMillis = millis();
           if (currentMillis - previousMillis >= 10){   //PID computed value updated every 3ms
              previousMillis = currentMillis;    // Save timestamp   
              Motor.Speed=encoder_count-encoder_count_prev;
              CONTROL_SPEED(pidcontroller_speed, Motor.Speed);          
              encoder_count_prev=encoder_count;
                 }
           } 
           if (P==0) {
              //Serial.println("stalling");
              unsigned long currentMillis2 = millis();       
              if (currentMillis2 - previousMillis2 >= 3){   //PID computed value updated every 3ms
                   previousMillis2 = currentMillis2;    // Save timestamp                      
                   pidcontroller.setpoint(encoder_count_prev); // The "goal" the PID controller tries to "reach",
                   Motor.PWM = pidcontroller.compute(encoder_count);  //Let the PID compute the value, returns the calculated optimal output                     
                   if (Motor.PWM > 0) // if the motor_pwm_value is greater than zero we rotate the  motor in clockwise direction
                     motor_ccw(Motor.PWM);
                   else // else we move it in a counter clockwise direction
                     motor_cw(abs(Motor.PWM));
                  }
                  
                 } 

           if (P==2){
             unsigned long currentMillis7 = millis();
             if (currentMillis7 - previousMillis7 >= 3){   //PID computed value updated every 3ms
                 previousMillis7 = currentMillis7;    // Save timestamp   
                 test=test+1;           
             }
              //Serial.println(test);
              if (test<angle_interval){
                 unsigned long currentMillis6 = millis();
                 if (currentMillis6 - previousMillis6 >= 10){   //PID computed value updated every 3ms
                    previousMillis6 = currentMillis6;    // Save timestamp   
                    Motor.Speed=encoder_count-encoder_count_prev;
                    CONTROL_SPEED(pidcontroller_speed, Motor.Speed);          
                    encoder_count_prev=encoder_count;
                 }
               } else {P=0;test=0;encoder_count_prev=encoder_count;}
                 Serial.println(test);  
             }
                             
         }
                   
    unsigned long currentMillis4 = millis();
    if (currentMillis4 - previousMillis4 >= 40){   //osc data sampling period
      previousMillis4 = currentMillis4;    // Save timestamp   
  
      Motor.Angle=map(encoder_count,round(tour/2),tour+round(tour/2),0,1023);
      Motor.Angle=(Motor.Angle+512)%1023;
      
      String buf4=String("/" + String(Motor.Address) + "/A");
      OSCMessage msgOUT_Fc4(buf4.c_str());  
      msgOUT_Fc4.add(Motor.Angle);
      Udp.beginPacket(Udp.remoteIP(),outPort);
      msgOUT_Fc4.send(Udp); // send the bytes
      Udp.endPacket(); // mark the end of the OSC Packet
      msgOUT_Fc4.empty(); // free space occupied by message 

      String buf8=String("/" + String(Motor.Address) + "/PWM");
      OSCMessage msgOUT_Fc8(buf8.c_str());  
      msgOUT_Fc8.add(Motor.PWM);
      Udp.beginPacket(Udp.remoteIP(),outPort);
      msgOUT_Fc8.send(Udp); // send the bytes
      Udp.endPacket(); // mark the end of the OSC Packet
      msgOUT_Fc8.empty(); // free space occupied by message 

      //Get a new normalized sensor event
      sensors_event_t accel;
      sensors_event_t gyro;
      sensors_event_t temp;
      lsm6dsox.getEvent(&accel, &gyro, &temp);
  
      ThetaG=atan2((double)accel.acceleration.y,(double)accel.acceleration.x)-(M_PI/4);
      if (ThetaG<=-M_PI){ThetaG=M_PI/4-abs(ThetaG+M_PI)+(3*M_PI/4);} //rotation of pi/4 to take the plug up as 0 reference
      //Serial.println(ThetaG);
      
      String buf9=String("/" + String(Motor.Address) + "/G");
      OSCMessage msgOUT_Fc9(buf9.c_str());  
      msgOUT_Fc9.add(ThetaG);
      Udp.beginPacket(Udp.remoteIP(),outPort);
      msgOUT_Fc9.send(Udp); // send the bytes
      Udp.endPacket(); // mark the end of the OSC Packet
      msgOUT_Fc9.empty(); // free space occupied by message     
    }

    unsigned long currentMillis5 = millis();
    if (currentMillis5 - previousMillis5 >= 10000){   //PID computed value updated every 3ms
        previousMillis5 = currentMillis5;    // Save timestamp   
    
    String buf7=String("/" + String(Motor.Address) + "/Bat");
    OSCMessage msgOUT_Fc7(buf7.c_str());  
    msgOUT_Fc7.add(((tp.GetBatteryVoltage()-3.5)/0.70)*100);
    Udp.beginPacket(Udp.remoteIP(),outPort);
    msgOUT_Fc7.send(Udp); // send the bytes
    Udp.endPacket(); // mark the end of the OSC Packet
    msgOUT_Fc7.empty(); // free space occupied by message 
    }

}//end void loop

//Functions
void MotorOFF(){
  ledcWrite(MotChannel1, 0);
  ledcWrite(MotChannel2, 0);
  }


void motor_cw(int power) {
  if (power > 20) {
    ledcWrite(MotChannel1, power);
    ledcWrite(MotChannel2, 0);
  }
  else {
    // both of the pins are set to low    
    ledcWrite(MotChannel1, 0);
    ledcWrite(MotChannel2, 0);
  }
}
void motor_ccw(int power) {
  if (power > 20) {
    ledcWrite(MotChannel2, power);
    ledcWrite(MotChannel1, 0);
  }
  else {
    ledcWrite(MotChannel1, 0);
    ledcWrite(MotChannel2, 0);
  }
}

void OSCMsgReceive(){
  OSCMessage msgIN;
  int size;

  if((size = Udp.parsePacket())>0){
   // Serial.print("OSC message received of size");Serial.println(size);
    while(size--)
    
      msgIN.fill(Udp.read());

    if(!msgIN.hasError()){ 

      String buf =  String("/" + String(Motor.Address) + "/Sto");
      msgIN.route(buf.c_str(),STOP);
      
      buf =  String("/" + String(Motor.Address) + "/F");
      msgIN.route(buf.c_str(),SPEED_SHIFT);

      buf =  String("/" + String(Motor.Address) + "/P");
      msgIN.route(buf.c_str(),MOVE_PLUS);

      buf =  String("/" + String(Motor.Address) + "/M");
      msgIN.route(buf.c_str(),MOVE_MINUS);
      
      buf =  String("/" + String(Motor.Address) + "/S1");
      msgIN.route(buf.c_str(),STATE1);
      
      buf =  String("/" + String(Motor.Address) + "/S2");
      msgIN.route(buf.c_str(),STATE2);
      
      buf =  String("/" + String(Motor.Address) + "/S3");
      msgIN.route(buf.c_str(),STATE3);
      
      buf =  String("/" + String(Motor.Address) + "/S4");
      msgIN.route(buf.c_str(),STATE4);
      
      buf =  String("/" + String(Motor.Address) + "/Po");
      msgIN.route(buf.c_str(),POKE);

      buf =  String("/" + String(Motor.Address) + "/U");
      msgIN.route(buf.c_str(),UPDATE_SPEED);

      //General
      msgIN.route("/Sto",STOP_ALL);   
      msgIN.route("/Sl",SLEEP);
      msgIN.route("/U",UPDATE_SPEED);
      msgIN.route("/F",SET_GLOBAL_SPEED);

      msgIN.route("/P1",CONT);
      msgIN.route("/P2",INCR_1);
      msgIN.route("/P3",INCR_2);
      msgIN.route("/P4",INCR_3);
      //Serial.println(buf.c_str());
    } 
   }
}

void STATE1(OSCMessage &msg, int addrOffset){   
    Motor.State=0;  
    //Serial.println(Motor.State);
    
    String buf=String("/" + String(Motor.Address) + "/S2");
    OSCMessage msgOUT_Fc1(buf.c_str());  
    msgOUT_Fc1.add(0);
    Udp.beginPacket(Udp.remoteIP(),outPort);
    msgOUT_Fc1.send(Udp); // send the bytes
    Udp.endPacket(); // mark the end of the OSC Packet
    msgOUT_Fc1.empty(); // free space occupied by message
    
    buf=String("/" + String(Motor.Address) + "/S3"); 
    OSCMessage msgOUT_Fc2(buf.c_str());  
    msgOUT_Fc2.add(0);
    Udp.beginPacket(Udp.remoteIP(),outPort);
    msgOUT_Fc2.send(Udp); // send the bytes
    Udp.endPacket(); // mark the end of the OSC Packet
    msgOUT_Fc2.empty(); // free space occupied by message 
    
    buf=String("/" + String(Motor.Address) + "/S4");
    OSCMessage msgOUT_Fc3(buf.c_str());  
    msgOUT_Fc3.add(0);
    Udp.beginPacket(Udp.remoteIP(),outPort);
    msgOUT_Fc3.send(Udp); // send the bytes
    Udp.endPacket(); // mark the end of the OSC Packet
    msgOUT_Fc3.empty(); // free space occupied by message     
  }
  
void STATE2(OSCMessage &msg, int addrOffset){
    Motor.State=1; 
    //Serial.println(Motor.State);

    String buf=String("/" + String(Motor.Address) + "/S1");
    OSCMessage msgOUT_Fc1(buf.c_str());  
    msgOUT_Fc1.add(0);
    Udp.beginPacket(Udp.remoteIP(),outPort);
    msgOUT_Fc1.send(Udp); // send the bytes
    Udp.endPacket(); // mark the end of the OSC Packet
    msgOUT_Fc1.empty(); // free space occupied by message
    
    buf=String("/" + String(Motor.Address) + "/S3"); 
    OSCMessage msgOUT_Fc2(buf.c_str());  
    msgOUT_Fc2.add(0);
    Udp.beginPacket(Udp.remoteIP(),outPort);
    msgOUT_Fc2.send(Udp); // send the bytes
    Udp.endPacket(); // mark the end of the OSC Packet
    msgOUT_Fc2.empty(); // free space occupied by message 
    
    buf=String("/" + String(Motor.Address) + "/S4");
    OSCMessage msgOUT_Fc3(buf.c_str());  
    msgOUT_Fc3.add(0);
    Udp.beginPacket(Udp.remoteIP(),outPort);
    msgOUT_Fc3.send(Udp); // send the bytes
    Udp.endPacket(); // mark the end of the OSC Packet
    msgOUT_Fc3.empty(); // free space occupied by message 
  }
  
void STATE3(OSCMessage &msg, int addrOffset){
    Motor.State=2;
    //Serial.println(Motor.State);

    String buf=String("/" + String(Motor.Address) + "/S1");
    OSCMessage msgOUT_Fc1(buf.c_str());  
    msgOUT_Fc1.add(0);
    Udp.beginPacket(Udp.remoteIP(),outPort);
    msgOUT_Fc1.send(Udp); // send the bytes
    Udp.endPacket(); // mark the end of the OSC Packet
    msgOUT_Fc1.empty(); // free space occupied by message
    
    buf=String("/" + String(Motor.Address) + "/S2"); 
    OSCMessage msgOUT_Fc2(buf.c_str());  
    msgOUT_Fc2.add(0);
    Udp.beginPacket(Udp.remoteIP(),outPort);
    msgOUT_Fc2.send(Udp); // send the bytes
    Udp.endPacket(); // mark the end of the OSC Packet
    msgOUT_Fc2.empty(); // free space occupied by message 
    
    buf=String("/" + String(Motor.Address) + "/S4");
    OSCMessage msgOUT_Fc3(buf.c_str());  
    msgOUT_Fc3.add(0);
    Udp.beginPacket(Udp.remoteIP(),outPort);
    msgOUT_Fc3.send(Udp); // send the bytes
    Udp.endPacket(); // mark the end of the OSC Packet
    msgOUT_Fc3.empty(); // free space occupied by message
  
  }
void STATE4(OSCMessage &msg, int addrOffset){
  
    Motor.State=3;
    //Serial.println(Motor.State);
    
    String buf=String("/" + String(Motor.Address) + "/S1");
    OSCMessage msgOUT_Fc1(buf.c_str());  
    msgOUT_Fc1.add(0);
    Udp.beginPacket(Udp.remoteIP(),outPort);
    msgOUT_Fc1.send(Udp); // send the bytes
    Udp.endPacket(); // mark the end of the OSC Packet
    msgOUT_Fc1.empty(); // free space occupied by message
    
    buf=String("/" + String(Motor.Address) + "/S2"); 
    OSCMessage msgOUT_Fc2(buf.c_str());  
    msgOUT_Fc2.add(0);
    Udp.beginPacket(Udp.remoteIP(),outPort);
    msgOUT_Fc2.send(Udp); // send the bytes
    Udp.endPacket(); // mark the end of the OSC Packet
    msgOUT_Fc2.empty(); // free space occupied by message 
    
    buf=String("/" + String(Motor.Address) + "/S3");
    OSCMessage msgOUT_Fc3(buf.c_str());  
    msgOUT_Fc3.add(0);
    Udp.beginPacket(Udp.remoteIP(),outPort);
    msgOUT_Fc3.send(Udp); // send the bytes
    Udp.endPacket(); // mark the end of the OSC Packet
    msgOUT_Fc3.empty(); // free space occupied by message  
  }

void MOVE_PLUS(OSCMessage &msg, int addrOffset){ 
    Motor.Plus_State=msg.getFloat(0);
    Motor.Minus_State=false;
    //Serial.print( "+ pushed ");Serial.println( Motor.Plus_State);
  }

void MOVE_MINUS(OSCMessage &msg, int addrOffset){ 
    Motor.Minus_State=msg.getFloat(0);
    Motor.Plus_State=false;
    //Serial.print( "- pushed ");Serial.println( Motor.Minus_State);
  }

void CONTROL_SPEED(PIDController PID, int Speed){     
   // Serial.println(encoder_count);
    int PWM=PID.compute(Speed);
    if (PWM > 0) // if the motor_pwm_value is greater than zero we rotate the  motor in clockwise direction
      motor_ccw(PWM);
    else // else we move it in a counter clockwise direction
      motor_cw(abs(PWM));  
    } 

void CONT(OSCMessage &msg, int addrOffset){
  P=msg.getFloat(0);
  //Serial.println("CONT");
  //Serial.println(P);

}
void INCR_1(OSCMessage &msg, int addrOffset){
  P=msg.getFloat(0);
  encoder_start=encoder_count;
  angle_interval=200;
  //Serial.println(encoder_start);
  //Serial.println("INCR 5");
  //Serial.println(P);
  

}
void INCR_2(OSCMessage &msg, int addrOffset){
  P=msg.getFloat(0);
  angle_interval=400;
  //Serial.println("INCR 10");
  //Serial.println(P);


}
void INCR_3(OSCMessage &msg, int addrOffset){
  P=msg.getFloat(0);
  angle_interval=1000;
  //Serial.println("INCR 20");
  //Serial.println(P);
}

void UPDATE_SPEED(OSCMessage &msg, int addrOffset){
  //Motor.Speed_target=msg.getFloat(0);
  }

void SPEED_SHIFT(OSCMessage &msg, int addrOffset){
   //Motor.Speed_target;
   Motor.Vshift=msg.getFloat(0);
   //Serial.println(Motor.Vshift);
   
   String buf=String("/" + String(Motor.Address) + "/F");
   OSCMessage msgOUT_Fc1(buf.c_str());  
   msgOUT_Fc1.add(Motor.Vshift);
   Udp.beginPacket(Udp.remoteIP(),outPort);
   msgOUT_Fc1.send(Udp); // send the bytes
   Udp.endPacket(); // mark the end of the OSC Packet
   msgOUT_Fc1.empty(); // free space occupied by message

   String buf2=String("/" + String(Motor.Address) + "/F/V");
   OSCMessage msgOUT_Fc2(buf2.c_str());  
   msgOUT_Fc2.add(Motor.Vshift+Motor.Speed_target);
   Udp.beginPacket(Udp.remoteIP(),outPort);
   msgOUT_Fc2.send(Udp); // send the bytes
   Udp.endPacket(); // mark the end of the OSC Packet
   msgOUT_Fc2.empty(); // free space occupied by message
  }

void SET_GLOBAL_SPEED(OSCMessage &msg, int addrOffset){
    Motor.Speed_target=msg.getFloat(0);
    
    String buf1=String("/F/V");
    OSCMessage msgOUT_Fc1(buf1.c_str());  
    msgOUT_Fc1.add(Motor.Speed_target);
    Udp.beginPacket(Udp.remoteIP(),outPort);
    msgOUT_Fc1.send(Udp); // send the bytes
    Udp.endPacket(); // mark the end of the OSC Packet
    msgOUT_Fc1.empty(); // free space occupied by message 

    String buf3=String("/" + String(Motor.Address) + "/F/V");
    OSCMessage msgOUT_Fc3(buf3.c_str());  
    msgOUT_Fc3.add(Motor.Vshift+Motor.Speed_target);
    Udp.beginPacket(Udp.remoteIP(),outPort);
    msgOUT_Fc3.send(Udp); // send the bytes
    Udp.endPacket(); // mark the end of the OSC Packet
    msgOUT_Fc3.empty(); // free space occupied by message
}

void STOP(OSCMessage &msg, int addrOffset){ 
    //ledcWrite(MotChannel1, 0);
    //ledcWrite(MotChannel2, 0);
    //MoveStop=false;    
    Motor.Minus_State=false;
    Motor.Plus_State=false;
}

void STOP_ALL(OSCMessage &msg, int addrOffset){
    //ledcWrite(MotChannel1, 0);
    //ledcWrite(MotChannel2, 0);
    //MoveStop=false;    
    Motor.State=0;  
    //Serial.println(Motor.State);
    
    String buf=String("/" + String(Motor.Address) + "/S2");
    OSCMessage msgOUT_Fc1(buf.c_str());  
    msgOUT_Fc1.add(0);
    Udp.beginPacket(Udp.remoteIP(),outPort);
    msgOUT_Fc1.send(Udp); // send the bytes
    Udp.endPacket(); // mark the end of the OSC Packet
    msgOUT_Fc1.empty(); // free space occupied by message
    
    buf=String("/" + String(Motor.Address) + "/S3"); 
    OSCMessage msgOUT_Fc2(buf.c_str());  
    msgOUT_Fc2.add(0);
    Udp.beginPacket(Udp.remoteIP(),outPort);
    msgOUT_Fc2.send(Udp); // send the bytes
    Udp.endPacket(); // mark the end of the OSC Packet
    msgOUT_Fc2.empty(); // free space occupied by message 
    
    buf=String("/" + String(Motor.Address) + "/S4");
    OSCMessage msgOUT_Fc3(buf.c_str());  
    msgOUT_Fc3.add(0);
    Udp.beginPacket(Udp.remoteIP(),outPort);
    msgOUT_Fc3.send(Udp); // send the bytes
    Udp.endPacket(); // mark the end of the OSC Packet
    msgOUT_Fc3.empty(); // free space occupied by message   
}

void SLEEP(OSCMessage &msg, int addrOffset){
   Serial.println("Going to sleep now");
   ledcWrite(MotChannel2,0);
   ledcWrite(MotChannel1,0);
    ///turn off RGB LED power to save battery life
    tp.DotStar_SetPower(true);
    tp.DotStar_SetPixelColor(30,0,0);
    tp.DotStar_Show();
    delay(2000);
    tp.DotStar_SetPower(false);
    esp_deep_sleep_start();
    delay(1000); 
}

//
//void ANGLE(OSCMessage &msg, int addrOffset){
//  Motor.Angle_target=msg.getFloat(0);
//  //Serial.print("Angle target= ");  Serial.println(Motor.Angle_target);
//
//  String buf6=String("/" + String(Motor.Address) + "/Knob/Angle_target");
//  OSCMessage msgOUT_Fc6(buf6.c_str());  
//  msgOUT_Fc6.add(Motor.Angle_target);
//  Udp.beginPacket(Udp.remoteIP(),outPort);
//  msgOUT_Fc6.send(Udp); // send the bytes
//  Udp.endPacket(); // mark the end of the OSC Packet
//  msgOUT_Fc6.empty(); // free space occupied by message 
//}

void POKE(OSCMessage &msg, int addrOffset){  
    Poke=msg.getFloat(0); 
}

void displaySensorDetailsISM(void) {
  Serial.print("Accelerometer range set to: ");
  switch (lsm6dsox.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  
  Serial.print("Gyro range set to: ");
  switch (lsm6dsox.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  }

  Serial.print("Accelerometer data rate set to: ");
  switch (lsm6dsox.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  Serial.print("Gyro data rate set to: ");
  switch (lsm6dsox.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }
}
