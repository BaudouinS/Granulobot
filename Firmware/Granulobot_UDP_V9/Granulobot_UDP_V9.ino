/*Granulobot UDP Firmware V9 
 * Encoder position measurement and conversion in angle and angular velocity
 * Magnet's magnetic field measurement
 * accel + gyr measurement
 * PWM Control of N20 geared motors
 * 2 ways UDP wifi communication with broadcasting address
 * PID on position encoder  https://github.com/DonnyCraft1/PIDArduino
 * https://circuitdigest.com/microcontroller-projects/arduino-based-encoder-motor-using-pid-controller
 *  OTA wireless firmware upload: https://randomnerdtutorials.com/esp32-over-the-air-ota-programming/

*      autoselect particule address from database defined in header {IP address number(last digits),paticule adress number}
* New on V9:
*      use mac addred for ID number instead of non static IP
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
*by Baudouin Saintyves
*April 30th 2021
*/

////////////////////////////////////
/////USER PARAMETERS//////////////////

/* WiFi network name and password */
const char * ssid = "TP-Link_C1BA";
const char * pwd = "93409582";
//const char * ssid = "HOME-26E2";
//const char * pwd = "baked7744borrow";

////add here new two last characters of Mac Address. //Particule number 
 char *ADDRESS_DATA_BASE[] = {  //
    "FC",//1
    "08",//2
    "20",//3
    "78",//4
    "38",//5
    "4C",//6
    "E8",//7
    "60",//8
    "54",//9
    //add new particule here
    "00"
    };
  
//choose here the motor's gear ratio
 const int ratio=380;

//choose here the motor's PID parameters
#define __Kp 260//260 // Proportional constant
#define __Ki 2.7 // Integral Constant
#define __Kd 2000//2000 // Derivative Constant

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
#include <Adafruit_LSM6DS.h>//accel+gyro
#include <Tlv493d.h>//magnetic field sensor

  //packet format and parsing for serial com
#include <Regexp.h>
#include <ArduinoJson.h> // tuto https://arduinojson.org/v6/doc/serialization/

  //TinyPico Helper Library
#include <TinyPICO.h>

   //OSC library
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCData.h>

  //PID library
#include <PIDController.h> // https://github.com/DonnyCraft1/PIDArduino

  
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


// IP address to send UDP data to.
// it can be ip address of the server or 
// a network broadcast address
// here is broadcast address
const char * udpAddress = "192.168.0.255"; //broadcasting address= the program will read from this address
//const char * PadAddress = "192.168.0.100";
//const char * udpAddress = "10.0.0.255"; //broadcasting address= the program will read from this address
//const char * PadAddress = "10.0.0.15";
const int localPort = 8000;//4210;
const int outPort = 9000;
//byte local[6];   //store mac address

////particule address number
//int ADDRESS_NUM;

 //Timers
unsigned long previousMillis = 0; // store previous millis readout (for PID)
unsigned long previousMillis2 = 0; // store previous millis readout (to send motor.angle)
unsigned long previousMillis3 = 0; // store previous millis readout (to send Bat level)
unsigned long previousMillis4 = 0; // store previous millis readout (to send Bat level)

//Encoder
volatile int encoder_count=0;
int prev_count=0;
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
  bool State;
  //int Displacement;
  int Angle;
  int Angle_target;
  bool Direction;//true=CW or false=CCW
  int PWM; // after PID computation data is stored in this variable.
} Message;

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

//create UDP instance
WiFiUDP Udp;
OSCErrorCode error;

// Initialise TinyPICO library
TinyPICO tp = TinyPICO();

//const int capacity=JSON_ARRAY_SIZE(3)+JSON_OBJECT_SIZE(1)+2*JSON_OBJECT_SIZE(3);
const int capacity = JSON_OBJECT_SIZE(9);
StaticJsonDocument<capacity> motion;

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

  Wire.setClock(400000); //test 
  
  log_d("Total heap: %d", ESP.getHeapSize());//core debug level to "verbose"
  log_d("Free heap: %d", ESP.getFreeHeap());
  log_d("Total PSRAM: %d", ESP.getPsramSize());
  log_d("Free PSRAM: %d", ESP.getFreePsram());
  
  Serial.begin(115200);
  Serial.println("RESET");
  Serial.println("Firmware Granulobot OSC V9");


//WIFI UDP SETUP

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
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
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
  Motor.Torque=0;
  Motor.Angle=0;
  Motor.Angle_target=0;
  Motor.Direction=true;
  Motor.State=0;
  Motor.PWM=0;

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

    lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
    lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS); 
    lsm6dsox.setAccelDataRate(LSM6DS_RATE_3_33K_HZ);
    lsm6dsox.setGyroDataRate(LSM6DS_RATE_3_33K_HZ);

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
  pidcontroller.limit(0, 0); // Limit the PID output between -255 to 255, this is important to get rid of integral windup!


//// Get a *rough* estimate of the current battery voltage
  // If the battery is not present, the charge IC will still report it's trying to charge at X voltage
  // so it will still show a voltage.
  Serial.print("Battery level: ");
  Serial.println(tp.GetBatteryVoltage());
  Serial.print("Battery charging in progress: ");
  Serial.println(tp.IsChargingBattery());  
  delay(2000);
  Serial.print("battery level= ");Serial.print(((tp.GetBatteryVoltage()-3.5)/0.70)*100);Serial.println("%");

// hello for UDP, no OSC version
//  Udp.beginPacket(udpAddress,outPort);
//  Udp.printf("Hello Python");
//  Udp.endPacket(); // mark the end of the OSC Packet
//  delay(5000);
  
//////intialize OSC control surface
//  String buf1=String("/" + String(Motor.Address) + "/Fader/Speed");
//  String buf2=String("/" + String(Motor.Address) + "/Fader");
//  String buf3=String("/" + String(Motor.Address) + "/State");
//  String buf4=String("/" + String(Motor.Address) + "/Knob/Angle");
//  String buf5=String("/" + String(Motor.Address) + "/Knob");
//  String buf6=String("/" + String(Motor.Address) + "/Knob/Angle_target");
//  String buf7=String("/" + String(Motor.Address) + "/Bat");
//  String buf8=String("/" + String(Motor.Address) + "/PWM");
//  String buf9=String("/" + String(Motor.Address) + "/IP");
//  //Serial.println( buf9);
//
//  OSCMessage msgOUT_Fc1(buf1.c_str());  
//  msgOUT_Fc1.add(Motor.Speed);
//  Udp.beginPacket(udpAddress,outPort);
//  msgOUT_Fc1.send(Udp); // send the bytes
//  Udp.endPacket(); // mark the end of the OSC Packet
//  msgOUT_Fc1.empty(); // free space occupied by message 
//
//  OSCMessage msgOUT_Fc2(buf2.c_str());  
//  msgOUT_Fc2.add(Motor.Speed);
//  Udp.beginPacket(udpAddress,outPort);
//  msgOUT_Fc2.send(Udp); // send the bytes
//  Udp.endPacket(); // mark the end of the OSC Packet
//  msgOUT_Fc2.empty(); // free space occupied by message 
//
//  OSCMessage msgOUT_Fc3(buf3.c_str());  
//  msgOUT_Fc3.add(Motor.State);
//  Udp.beginPacket(udpAddress,outPort);
//  msgOUT_Fc3.send(Udp); // send the bytes
//  Udp.endPacket(); // mark the end of the OSC Packet
//  msgOUT_Fc3.empty(); // free space occupied by message 
//
//  OSCMessage msgOUT_Fc4(buf4.c_str());  
//  msgOUT_Fc4.add(Motor.Angle);
//  Udp.beginPacket(udpAddress,outPort);
//  msgOUT_Fc4.send(Udp); // send the bytes
//  Udp.endPacket(); // mark the end of the OSC Packet
//  msgOUT_Fc4.empty(); // free space occupied by message 
//
//  OSCMessage msgOUT_Fc5(buf5.c_str());  
//  msgOUT_Fc5.add(Motor.Angle);
//  Udp.beginPacket(udpAddress,outPort);
//  msgOUT_Fc5.send(Udp); // send the bytes
//  Udp.endPacket(); // mark the end of the OSC Packet
//  msgOUT_Fc5.empty(); // free space occupied by message 
//
//  OSCMessage msgOUT_Fc6(buf6.c_str());  
//  msgOUT_Fc6.add(Motor.Angle_target);
//  Udp.beginPacket(udpAddress,outPort);
//  msgOUT_Fc6.send(Udp); // send the bytes
//  Udp.endPacket(); // mark the end of the OSC Packet
//  msgOUT_Fc6.empty(); // free space occupied by message 
//  
//  OSCMessage msgOUT_Fc7(buf7.c_str());  
//  delay(2000);
//  msgOUT_Fc7.add(((tp.GetBatteryVoltage()-3.5)/0.70)*100);
//  Udp.beginPacket(Udp.remoteIP(),outPort);
//  msgOUT_Fc7.send(Udp); // send the bytes
//  Udp.endPacket(); // mark the end of the OSC Packet
//  msgOUT_Fc7.empty(); // free space occupied by message  
//  
//  OSCMessage msgOUT_Fc8(buf8.c_str());  
//  msgOUT_Fc8.add(Motor.PWM);
//  Udp.beginPacket(Udp.remoteIP(),outPort);
//  msgOUT_Fc8.send(Udp); // send the bytes
//  Udp.endPacket(); // mark the end of the OSC Packet
//  msgOUT_Fc8.empty(); // free space occupied by message  
//
//  OSCMessage msgOUT_Fc9(buf9.c_str());  
//  msgOUT_Fc9.add(local[3]); 
//  Udp.beginPacket(Udp.remoteIP(),outPort);
//  msgOUT_Fc9.send(Udp); // send the bytes
//  Udp.endPacket(); // mark the end of the OSC Packet
//  msgOUT_Fc9.empty(); // free space occupied by message

}//end setup loop

void loop(){
  //Serial.print(micros());
  ArduinoOTA.handle();
  //OSCMsgReceive();
//
  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  //Serial.print(" "); Serial.println(micros());
//
  lsm6dsox.getEvent(&accel, &gyro, &temp);

  //Serial.println(millis());
 //Serial.println(lsm6dsox.rawAccX);
 if (encoder_count>tour-1){encoder_count=encoder_count-tour;}   
 if (encoder_count<0){encoder_count=tour+encoder_count;}
 //Serial.println(encoder_count);

 if (Poke){ 
    //Serial.println("POKE");
    tp.DotStar_SetPower(true);
    tp.DotStar_SetPixelColor(0,0,100);
    tp.DotStar_Show();
 }else{tp.DotStar_SetPower(false);}   

 if (Motor.State){  //if "stiff" selected
  
    integerValue=map(Motor.Angle_target,0,1023,0,tour);
    //if (integerValue+tour-encoder_count<round(tour/2)){integerValue=tour+integerValue;}
    if (tour-integerValue>round(tour/2)){integerValue=tour+integerValue;}
    if (integerValue-encoder_count>round(tour/2)){encoder_count=tour+encoder_count;}
  
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= 3){   //PID computed value updated every 3ms
          previousMillis = currentMillis;    // Save timestamp   
  
  
     pidcontroller.setpoint(integerValue); // The "goal" the PID controller tries to "reach",
  
   // Serial.println(integerValue); // print the incoming value for debugging
  //  Serial.println(encoder_count);
      
   
     Motor.PWM = pidcontroller.compute(encoder_count);  //Let the PID compute the value, returns the calculated optimal output
  
   // Serial.print(motor_pwm_value); // print the calculated value for debugging
   // Serial.print("   ");
  //  Serial.println();
    
     if (Motor.PWM > 0) // if the motor_pwm_value is greater than zero we rotate the  motor in clockwise direction
      motor_ccw(Motor.PWM);
     else // else we move it in a counter clockwise direction
      motor_cw(abs(Motor.PWM));
    //Serial.println(encoder_count);// print the final encoder count.
    // delay(1);
  //  if (encoder_count>tour-1){encoder_count=encoder_count-tour;}   
  //  if (encoder_count<0){encoder_count=tour+encoder_count;}
  //  Motor.Angle=encoder_count;
  //  Motor.Angle=map(encoder_count,0,tour,0,1023);

     }
    
  } else { //end if Motor.State
      Motor.Angle_target=Motor.Angle; 
      Motor.PWM=0;
//      unsigned long currentMillis4 = millis();
//      if (currentMillis4 - previousMillis4 >= 40){   //PID computed value updated every 3ms
//        previousMillis4 = currentMillis4;    // Save timestamp   
//        String buf6=String("/" + String(Motor.Address) + "/Knob/Angle_target");
//        OSCMessage msgOUT_Fc6(buf6.c_str());  
//        msgOUT_Fc6.add(Motor.Angle_target);
//        Udp.beginPacket(Udp.remoteIP(),outPort);
//        msgOUT_Fc6.send(Udp); // send the bytes
//        Udp.endPacket(); // mark the end of the OSC Packet
//        msgOUT_Fc6.empty(); // free space occupied by message 
//      }
  } //end else Motor.State




    unsigned long currentMillis2 = millis();
    if (currentMillis2 - previousMillis2 >= 4){   //1/this value=frequency at which value is send to broadcast
      previousMillis2 = currentMillis2;    // Save timestamp   
  
      Motor.Angle=map(encoder_count,round(tour/2),tour+round(tour/2),0,1023);
      Motor.Angle=(Motor.Angle+512)%1023;
      
//      String buf4=String("/" + String(Motor.Address) + "/Knob/Angle");
//      OSCMessage msgOUT_Fc4(buf4.c_str());  
//      msgOUT_Fc4.add(Motor.Angle);
//      Udp.beginPacket(Udp.remoteIP(),outPort);
//      msgOUT_Fc4.send(Udp); // send the bytes
//      Udp.endPacket(); // mark the end of the OSC Packet
//      msgOUT_Fc4.empty(); // free space occupied by message 
 // Serial.print(" ");Serial.println(micros());

// create JSON object 
      // Declare a buffer to hold the result
      char output[192];// recomended size. can be decreased down to 144 max. https://arduinojson.org/v6/assistant/
      StaticJsonDocument<192> motion;

//      JsonObject Angle = motion.createNestedObject();
//      Angle["A"] = Motor.Angle;    
//      JsonObject acc = motion.createNestedObject();
//      acc["x"] = accel.acceleration.x;
//      acc["y"] = accel.acceleration.y;
//      acc["z"] = accel.acceleration.z;     
//      JsonObject gyr = motion.createNestedObject();
//      gyr["u"] = gyro.gyro.x;
//      gyr["v"] = gyro.gyro.y;
//      gyr["w"] = gyro.gyro.z;

      // Compute the length of the minified JSON document
//      int len1=measureJson(motion);
//      Serial.print("JSON document size:");Serial.println(len1);



      motion["A"] = Motor.Angle;    
      motion["t"] = millis();
      motion["d"] = Motor.Address;
      
//      motion["x"] = lsm6dsox.rawAccX;
//      motion["y"] = lsm6dsox.rawAccY;
//      motion["z"] = lsm6dsox.rawAccZ;
//      motion["u"] = lsm6dsox.rawGyroX;
//      motion["v"] = lsm6dsox.rawGyroY;
//      motion["w"] = lsm6dsox.rawGyroZ;

      motion["x"] = accel.acceleration.x;
      motion["y"] = accel.acceleration.y;
      motion["z"] = accel.acceleration.z;
      motion["u"] = gyro.gyro.x;
      motion["v"] = gyro.gyro.y;
      motion["w"] = gyro.gyro.z;

//      Serial.println(lsm6dsox.rawAccX);
      
      serializeJson(motion,output);
      //serializeJson(motion,Serial);
      //Serial.println();
      //Serial.print(" ");Serial.println(micros());
      
      //char buffer1[128];
      //memset(buffer1, 0, 6);   
      Udp.beginPacket(udpAddress, outPort);   
      //itoa(Motor.Angle, buffer1,10);
      //Udp.print("A");
      Udp.print(output);
      Udp.endPacket();
 // Serial.print(" ");Serial.println(micros());

      //Serial.println(Motor.Angle);
      
//      String buf5=String("/" + String(Motor.Address) + "/Knob");
//      OSCMessage msgOUT_Fc5(buf5.c_str());  
//      msgOUT_Fc5.add(Motor.Angle);
//      Udp.beginPacket(Udp.remoteIP(),outPort);
//      msgOUT_Fc5.send(Udp); // send the bytes
//      Udp.endPacket(); // mark the end of the OSC Packet
//      msgOUT_Fc5.empty(); // free space occupied by message 
//
//      String buf8=String("/" + String(Motor.Address) + "/PWM");
//      OSCMessage msgOUT_Fc8(buf8.c_str());  
//      msgOUT_Fc8.add(Motor.PWM);
//      Udp.beginPacket(Udp.remoteIP(),outPort);
//      msgOUT_Fc8.send(Udp); // send the bytes
//      Udp.endPacket(); // mark the end of the OSC Packet
//      msgOUT_Fc8.empty(); // free space occupied by message 

    }///loop current millis

//    unsigned long currentMillis3 = millis();
//    if (currentMillis3 - previousMillis3 >= 10000){   //PID computed value updated every 3ms
//        previousMillis3 = currentMillis3;    // Save timestamp   
//    
//        String buf7=String("/" + String(Motor.Address) + "/Bat");
//        OSCMessage msgOUT_Fc7(buf7.c_str());  
//        msgOUT_Fc7.add(((tp.GetBatteryVoltage()-3.5)/0.70)*100);
//        Udp.beginPacket(Udp.remoteIP(),outPort);
//        msgOUT_Fc7.send(Udp); // send the bytes
//        Udp.endPacket(); // mark the end of the OSC Packet
//        msgOUT_Fc7.empty(); // free space occupied by message 
//    }
//  Serial.print(" ");Serial.println(millis());

}   //end void loop

//Functions

void motor_cw(int power) {
  if (power > 40) {
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
  if (power > 40) {
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

      String buf =  String("/" + String(Motor.Address) + "/Stop");
      msgIN.route(buf.c_str(),STOP);
      
      buf =  String("/" + String(Motor.Address) + "/Fader");
      msgIN.route(buf.c_str(),SPEED);

      buf =  String("/" + String(Motor.Address) + "/Knob");
      msgIN.route(buf.c_str(),ANGLE);
      
      buf =  String("/" + String(Motor.Address) + "/State");
      msgIN.route(buf.c_str(),STATE);

      buf =  String("/" + String(Motor.Address) + "/Poke");
      msgIN.route(buf.c_str(),POKE);
      
      msgIN.route("/StopAll",STOP_ALL);
      msgIN.route("/Sleep",SLEEP);

      //Serial.println(buf.c_str());
    }
  
   }
}

void STOP(OSCMessage &msg, int addrOffset){
  
    ledcWrite(MotChannel1, 0);
    ledcWrite(MotChannel2, 0);
    MoveStop=false;    
}

void STOP_ALL(OSCMessage &msg, int addrOffset){
  
    ledcWrite(MotChannel1, 0);
    ledcWrite(MotChannel2, 0);
    MoveStop=false;    
}

void STATE(OSCMessage &msg, int addrOffset){ 
    Motor.State=msg.getFloat(0);
    Serial.print("State: ");Serial.println(Motor.State);
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

void SPEED(OSCMessage &msg, int addrOffset){
  Motor.Speed=msg.getFloat(0);
  pidcontroller.limit(Motor.Speed*(-1),Motor.Speed); // Limit the PID output between -Motor.Speed to Motor.Speed, this is important to get rid of integral windup!
  //Serial.print("Speed= ");  Serial.println(Motor.Speed);

  String buf1=String("/" + String(Motor.Address) + "/Fader/Speed");
  OSCMessage msgOUT_Fc1(buf1.c_str());  
  msgOUT_Fc1.add(Motor.Speed);
  Udp.beginPacket(Udp.remoteIP(),outPort);
  msgOUT_Fc1.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT_Fc1.empty(); // free space occupied by message 

}

void ANGLE(OSCMessage &msg, int addrOffset){
  Motor.Angle_target=msg.getFloat(0);
  //Serial.print("Angle target= ");  Serial.println(Motor.Angle_target);

  String buf6=String("/" + String(Motor.Address) + "/Knob/Angle_target");
  OSCMessage msgOUT_Fc6(buf6.c_str());  
  msgOUT_Fc6.add(Motor.Angle_target);
  Udp.beginPacket(Udp.remoteIP(),outPort);
  msgOUT_Fc6.send(Udp); // send the bytes
  Udp.endPacket(); // mark the end of the OSC Packet
  msgOUT_Fc6.empty(); // free space occupied by message 
}

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
