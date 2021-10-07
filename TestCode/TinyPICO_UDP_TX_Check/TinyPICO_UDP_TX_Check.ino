/* Sketch example for using TinyPICO (UDP heavy)
 * 
 * 2021-9-9: Written for testing picos
 * 
 * 10k message in 10 (with Serial.print("Done UDP") statement) i.e. 1000/sec (use Punkt)
 * 100k message in 77 seconds i.e. 1300/sec (use Punkt)
 */

#include <TinyPICO.h>
#include <WiFi.h>
#include <WiFiUdp.h>

//const char* ssid = "Punkt";
//const char* passwd = "next";
//const char* udpaddr = "192.168.3.81";
const char* ssid = "gbots";
const char* passwd = "gbotsRus";
const char* udpaddr = "192.168.2.1";
const int udport = 4444;
String ipAddress;
byte mac[6]; // MAC address
String macs;
//String text="De Rerum Natura: Aeneadum genetrix, hominum divomque voluptas, alma Venus, caeli subter labentia signa quae mare navigerum, quae terras frugiferentis concelebras, per te quoniam genus omne animantum concipitur visitque exortum lumina solis:";
String text="De Rerum Natura: Aeneadum genetrix, hominum divomque voluptas, alma Venus,";
//           01234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890

int i;

// Set up variables
uint8_t buffer[50] = "";
int count = 0;

// Initialise the TinyPICO library
TinyPICO tp = TinyPICO();

// Set up a udp
WiFiUDP udp;

void setup()
{
  // Used for debug output only
  Serial.begin(115200);
  // Set Up wifi
  WiFi.begin(ssid, passwd);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi.");
  }
  Serial.println("WiFi connected");
  udp.begin(udport);
  // You can set the DotStar LED colour directly using r,g,b values
  tp.DotStar_SetPixelColor( 255, 128, 0 );
  delay(1000);
  tp.DotStar_SetPixelColor( 0x0099FF );
  delay(1000);
  Serial.println("Blinked");
  WiFi.macAddress(mac);
  macs = "Mac:"+String(mac[4], HEX)+":"+String(mac[5], HEX);
  Serial.println("IP = " + WiFi.localIP().toString() );
}

void loop()
{
  // Get IP Address
  ipAddress = WiFi.localIP().toString();
  // Send UDP
  //sprintf(buffer, "From %s UDP %d", count);
  udp.beginPacket(udpaddr, udport);
  //udp.write(buffer, 9);
  udp.printf("UDP ip:%s %s Cnt:%d %s", ipAddress.c_str(), macs, count, text.c_str());
  udp.endPacket();
  //Serial.println("Done UDP");
  count += 1;
  delay(1);
}
