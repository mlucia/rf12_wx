/*
RxNode for weather station 
includes RTC and baro sensor


*/

#include <Ports.h>
#include <RF12.h>
#include <Wire.h>
#include <RTClib.h>
#include <EtherCard.h>

// ethernet interface mac address
static byte mymac[] = { 0x74,0x69,0x69,0x2D,0x30,0x31 };
static byte ntpServer[] = {172, 16, 16, 1};

byte Ethernet::buffer[1000];
static BufferFiller bfill;  // used as cursor while filling the buffer

/*
unsigned int localPort = 8888;      // local port to listen for UDP packets
IPAddress timeServer(172, 16, 16, 1);
const int NTP_PACKET_SIZE= 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
EthernetUDP Udp;
*/

RTC_DS1307 RTC;

#define DEBUG 1
#define SERIAL 1

const uint8_t baroPin = A1;
const uint8_t ledPin = 4;

uint32_t ntpTime;
uint32_t *p_ntpTime = &ntpTime;

uint8_t rval = 0;

float mbarBaro;
float inHgBaro;


// DS1307 is using A0 and D3

//=======================================================
// data from remote node
//=======================================================

struct all_Data {
  int deviceId;
  unsigned long index;
  int tempC;
  int relHumi;
  int dewPt;
  int windSpd;
  int windBurst;
  int windDir;
  char windDirV;
  int battVolt;
  int pvVolt;  
} buf;


//=======================================================
// setup
//=======================================================


void setup() {

#if SERIAL
  Serial.begin(9600);
  Serial.println("\n[WX rx node]");
#endif

  Wire.begin();
  RTC.begin();

  pinMode(baroPin, INPUT);
  pinMode(ledPin, OUTPUT);
  
  digitalWrite(ledPin, LOW);
  
  rf12_initialize(31, RF12_915MHZ, 5);

  // following line sets the RTC to the date & time this sketch was compiled
  // RTC.adjust(DateTime(__DATE__, __TIME__));
  
  reportTimeStamp();
  
#if SERIAL
  if (ether.begin(sizeof Ethernet::buffer, mymac) == 0) 
    Serial.println( " Failed to access Ethernet controller");

  if (!ether.dhcpSetup())
    Serial.println(" DHCP failed");
  
  Serial.println(" ");
  ether.printIp("My IP: ", ether.myip);
  ether.printIp("Netmask: ", ether.mymask);
  ether.printIp("GW IP: ", ether.gwip);
  ether.printIp("DNS IP: ", ether.dnsip);


  Serial.println("Done with setup\n");
#endif

  ether.ntpRequest(ntpServer, 123);
  rval = ether.ntpProcessAnswer(p_ntpTime, 8888);
  Serial.print("NTP time: ");
  Serial.println(ntpTime);

}


//=======================================================
// loop
//=======================================================

void loop() {
  

    word len = ether.packetReceive();
    word pos = ether.packetLoop(len);
    // check if valid tcp data is received
    if (pos) 
        ether.httpServerReply(homePage()); // send web page data


  if (  rf12_recvDone() && rf12_crc == 0 && rf12_len == sizeof buf ) {
    memcpy(&buf, (byte*) rf12_data, sizeof buf);
    rf12_recvDone(); // re-enable reception right away
    reportTimeStamp();
    baroRead();
    sendToSerial();
  }
  
}

//=======================================================
// Read Baro and Calculate inHg
//=======================================================
void baroRead() {

  int adcBaro;
  
  adcBaro = analogRead(baroPin);
  
  mbarBaro = (((float) adcBaro / 1024.00) + 0.095) / 0.0009;
  inHgBaro = mbarBaro * 0.029533;
 
}

//=======================================================
// Report the timestamp
//=======================================================

void reportTimeStamp() {
 
    DateTime now = RTC.now();
//  Serial.println("Done with RTC call \n");
#if SERIAL    
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.print('  '); 
#endif
}


//=======================================================
// Home Page
//=======================================================

static word homePage() {
  bfill = ether.tcpOffset();
  bfill.emit_p(PSTR(
    "HTTP/1.0 200 OK\r\n"
    "Content-Type: text/html\r\n"
    "Pragma: no-cache\r\n"
    "\r\n"
    "<meta http-equiv='refresh' content='1'/>"
    "<title>Arduino Wx Station</title>"
    "<h2>Arduino Weather Station</h2>"
    "<body>Latest Reported Conditions for DeviceID $D"
    "<ul>"
    "<li>TempC: $D$D.$D C</li>"
    "<li>Rel Humidity: $D$D.$D %</li>"
    "<li>Dew Point: $D$D.$D C</li>"
    "<li>Wind Speed: $D$D$D MPH</li>"
    "<li>Wind Burst: $D$D$D MPH</li>"
    "<li>Wind Dir: $D$D$D</li>"
    "<li>Wind Variability: $S</li>"
    "<li>Battery Voltage: $D.$D VDC</li>"
    "<li>Photo Cell Voltage: $D.$D</li>"
    "</ul></body>"), buf.deviceId, 
    (buf.tempC/100) % 10, (buf.tempC/10) % 10 , buf.tempC % 10,
    (buf.relHumi/100) % 10, (buf.relHumi/10) % 10, buf.relHumi % 10,
    (buf.dewPt/100) % 10, (buf.dewPt/10) % 10, buf.dewPt % 10,
    (buf.windSpd/100) % 10, (buf.windSpd / 10) % 10, buf.windSpd % 10,
    (buf.windBurst/100) % 10, (buf.windBurst / 10) % 10, buf.windBurst % 10,
    (buf.windDir/100) % 10, (buf.windDir / 10) % 10, buf.windDir % 10,
    buf.windDirV,
    (buf.battVolt / 10) % 10, buf.battVolt % 10,
    (buf.pvVolt / 10) % 10, buf.pvVolt % 10);

  return bfill.position();

}


//=======================================================
// Report the timestamp
//=======================================================

void sendToSerial() {

    Serial.print(buf.deviceId);
    Serial.print(", ");
    Serial.print(buf.index);
    Serial.print(", ");
    Serial.print(buf.tempC/10.0,1);
    Serial.print(", ");
    Serial.print(buf.relHumi/10.0,1);
    Serial.print(", ");
    Serial.print(buf.dewPt/10.0,1);
    Serial.print(", ");
    Serial.print(buf.windSpd);
    Serial.print(", ");
    Serial.print(buf.windBurst);
    Serial.print(", ");
    Serial.print(buf.windDir/10.0,1);
    Serial.print(", ");
    Serial.print(buf.windDirV);
    Serial.print(", ");
    Serial.print(buf.battVolt/10.0,2);
    Serial.print(", ");
    Serial.print(buf.pvVolt/10.0,2);
    Serial.print(" ");
    Serial.println(inHgBaro,2);
    
  #if DEBUG
    Serial.print("Device Id: ");
    Serial.print(buf.deviceId);
    Serial.print(" Index: ");
    Serial.print(buf.index);
    Serial.print(" WX Temp: ");
    Serial.print(buf.tempC/10.0,1);
    Serial.print(" C RH: ");
    Serial.print(buf.relHumi/10.0,1);
    Serial.print(" % DP: ");
    Serial.print(buf.dewPt/10.0,1);
    Serial.print(" C ");
    Serial.print(buf.windSpd);
    Serial.print(" Mph ");
    Serial.print(buf.windBurst);
    Serial.print(" Mph ");
    Serial.print(buf.windDir);
    Serial.print(" Deg ");
    Serial.print(buf.windDirV);
    Serial.print(" Var ");    
    Serial.print(buf.battVolt/10.0,2);
    Serial.print(" V ");
    Serial.print(buf.pvVolt/10.0,2);
    Serial.print(" V");
    Serial.print(" ");
    Serial.println(inHgBaro,2);

  #endif
}


