/* TO BE DONE

3. Add code to reboot the whole thing daily - it stops sending every once in a while
4. Increase the wind speed buffers to get a more acurate averaged wind speed
5. Add detection of battery charging and report that value to home
*/

/*
Original Code from
http://home.comcast.net/~saustin98/misc/WeatherStationADC.txt

8Mhz crystal

Modifications include:
Addition of Rain Detector code
Addition of SHT-15 code for humidity and temp
Addition of RF12 from jeelab.org
Addition of anemomter debounce delay


====================================================================
Notes from original code - WeatherStationADC
====================================================================

Arduino sketch for Weather device from Sparkfun.
Uses only the wind direction vane and the anemometer (not the rain gauge).

Although the inclination for a weather logger is to run it for
a long time, due to the way Wiring.c implements the millis() function,
this should be restarted, oh, monthly. The millis() functions overflows
after about 49 days. We could allow for that here, and handle the
wraparound, but you've got bigger problems anyway with the delay()
function at an overflow, so it's best to "reboot".

=========================================================
ANEMOMETER
=========================================================
This is connected to Arduino ground on one side, and pin D2 (for the
attachInterrupt(0, ...) on the other.
Pin 2 is pulled up, and the reed switch on the anemometer will send
that to ground twice per revolution, which will trigger the interrupt.
We count the number of revolutions in 5 seconds, and divide by 5.
One Hz (rev/sec) = 1.492 mph.

=========================================================
WIND DIRECTION VANE
=========================================================
We use a classic voltage divider to measure the resistance in
the weather vane, which varies by direction.
Using a 10K resistor, our ADC reading will be:
   1023 * (R/(10000+R))
where R is the unknown resistance from the vane. We'll scale
the 1023 down to a 255 range, to match the datasheet docs.

                  +5V
                   |
                   <
                   >     10K
                   <   Resistor
                   <
                   >
                   |
 Analog Pin 5------|
                  |
                   -----------| To weather vane
                              | (mystery resistance)
                   -----------|
                   |
                   |
                 -----
                  ---
                   -
The ADC values we get for each direction (based on a 255 max)
follow, assuming that pointing away from the assembly center
is sector zero. The sector number is just which 45-degree sector
it is, clockwise from the "away" direction. The direction
shown is assuming that "away" is West. Depending how
you orient the system, you'll have to adjust the directions.

=========================================================
*/

#include <Sensirion.h>

#include <Ports.h>
#include <RF12.h>

#define SERIAL 1
#define DEBUG 0

const uint8_t dataPin =  8;            // SHT serial data
const uint8_t sclkPin =  9;            // SHT serial clock
const uint32_t MSECS_QUERY_SHT = 60000UL;     // Sensor query period and the radio tx period - 60000 = 60 seconds

const uint8_t aneoPin =  3;
const uint8_t mmpcPin =  5;
const uint8_t ledPin  =  7;

// analog inputs
const uint8_t solarPin = 0;
const uint8_t battPin =  1;
const uint8_t vanePin =  3;

const uint16_t MSECS_CALC_WIND_SPEED = 5000;
const uint16_t MSECS_CALC_WIND_DIR   = 5000;
const uint16_t MSECS_CALC_RAIN_AMT   = 5000;
const uint16_t MSECS_WAKE_RF12       = 1000;

Sensirion sht = Sensirion(dataPin, sclkPin);

uint16_t rawData;
float temperature;
float humidity;
float dewpoint;

int windSpdVals[12];
int windDirVals[12];
int idx = 0;

byte myState = 0;
byte myDeviceId = 1;

volatile int numRevsAnemometer = 0;   // Incremented in the interrupt
volatile int numBuckettip = 0;      // Incremented in the interupt

unsigned long time;                   // Time interval tracking
unsigned long nextQuerySht;
unsigned long nextCalcSpeed;          // When we next calc the wind speed
unsigned long nextCalcDir;            // When we next calc the direction
unsigned long nextCalcRain;           // When we next calc the rain
unsigned long nextWakeRF12;           // When to wake the radio

// ADC readings:
#define NUMDIRS 8
unsigned long   adc[NUMDIRS] = {26, 45, 77, 118, 161, 196, 220, 256};

// These directions match 1-for-1 with the values in adc, but
// will have to be adjusted as noted above. Modify 'dirOffset'
// to which direction is 'away' (it's West here).
//char *strVals[NUMDIRS] = {"W","NW","N","SW","NE","S","SE","E"};
int dirVals[NUMDIRS] = {270, 315, 0, 225, 45, 180, 135, 90};
byte dirOffset=0;


typedef struct {
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
} all_Data;

all_Data SensorData;


//=======================================================
// setup
//=======================================================

//ISR(WDT_vect) { Sleepy::watchdogEvent(); }

void setup()
{
#if SERIAL
  Serial.begin(9600);
#endif
  
  delay(15);                           // Wait at least 11 ms before first cmd
  rf12_initialize(myDeviceId, RF12_915MHZ, 5);

  initData();

  pinMode(aneoPin, INPUT);
  digitalWrite(aneoPin, HIGH);
  attachInterrupt(1, countAnemometer, FALLING);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  
  nextQuerySht  = millis() + MSECS_QUERY_SHT;
  nextCalcSpeed = millis() + MSECS_CALC_WIND_SPEED;
  nextCalcDir   = millis() + MSECS_CALC_WIND_DIR;
  nextCalcRain  = millis() + MSECS_CALC_RAIN_AMT;
  nextWakeRF12  = millis() + MSECS_WAKE_RF12;
  
//  sht.writeSR(LOW_RES);                // Set sensor to low resolution
  sht.measTemp(&rawData);              // Maps to: sht.meas(TEMP, &rawData, BLOCK)
  temperature = sht.calcTemp(rawData);
  sht.measHumi(&rawData);              // Maps to: sht.meas(HUMI, &rawData, BLOCK)
  humidity = sht.calcHumi(rawData, temperature);
  dewpoint = sht.calcDewpoint(humidity, temperature);

  rf12_sleep(RF12_SLEEP);
}

//=======================================================
// loop
//=======================================================

void loop()
{
  
  time = millis();

 
// if myDeviceId = 1 - Wind Speed and Dir sensors
//
  if (myDeviceId = 1) {
      if (time >= nextCalcSpeed) {
        calcWindSpeed();
        nextCalcSpeed = time + MSECS_CALC_WIND_SPEED;
      }
   
    if (time >= nextCalcDir) {
      calcWindDir();
      nextCalcDir = time + MSECS_CALC_WIND_DIR;
     }
  }
   
// if myDeviceId = 2 - Rain sensor
//
   if (myDeviceId = 2) {
     if (time >= nextCalcRain) {
        calcRainAmt();
        nextCalcRain = time + MSECS_CALC_RAIN_AMT;
     }
   }
  
  switch (myState) {
  case 0:
    if (time >= nextQuerySht) {      // Start new temp/humi measurement?
      sht.meas(TEMP, &rawData, NONBLOCK);
      myState++;
      nextQuerySht = time + MSECS_QUERY_SHT;
    }
    break;
  case 1:
    if (sht.measRdy()) {                         // Process temperature measurement?
      temperature = sht.calcTemp(rawData);
      sht.meas(HUMI, &rawData, NONBLOCK);
      myState++;
    }
    break;
  case 2:
    if (sht.measRdy()) {                         // Process humidity measurement?
      humidity = sht.calcHumi(rawData, temperature);
      dewpoint = sht.calcDewpoint(humidity, temperature);
      myState++;
    }
    break;

  case 3:                                        // measure batt
    adRead();
    myState++;
    break;  
    
  case 4:
    rf12_sleep(RF12_WAKEUP);            // wake the radio
    delay(5);
    while (!rf12_canSend())    // wait until I can send
      rf12_recvDone();
    logData();
    rf12_sleep(RF12_SLEEP);            // put the radio back to sleep
    myState = 0;
    delay(1);
    break;  

  default:
    #if SERIAL
      Serial.println("How did I get here?");
    #endif
    myState = 0;
    break;
  }

//  Sleepy::loseSomeTime(1000);
}

//=======================================================
// log the data/transmit the data
//=======================================================

void logData() {
  unsigned long tmpWindSpd = 0;
  unsigned char windDirVar = 0;
  
  float avgWindSpd = 0.0;
  
  SensorData.tempC = temperature * 10.0;
  SensorData.relHumi = humidity * 10.0;
  SensorData.dewPt = dewpoint * 10.0;

  for (idx = 0; idx < 12; idx++) {
    tmpWindSpd += windSpdVals[idx];
  }

// look for a variable wind - 
// not a vary good method - needs improvement
  for (idx = 0; idx < 11; idx++) {
    if (windDirVals[idx] != windDirVals[idx + 1])
      windDirVar++;
  }
  
  if (windDirVar > 4) {
    SensorData.windDirV = 'V';
  } else {
    SensorData.windDirV = 'S';
  } 
// end of variable wind section    
    
  idx = 0;
  
  avgWindSpd = float(tmpWindSpd) / float(12);
  
  SensorData.windSpd = int(avgWindSpd);

  rf12_sendStart(0, &SensorData, sizeof SensorData);
  rf12_sendWait(2);
  
  #if SERIAL
    Serial.print("Index = ");  Serial.print(SensorData.index);
    Serial.print(" Temp = ");   Serial.print(temperature);
    Serial.print(" Temp*10 = "); Serial.print(SensorData.tempC);  
    Serial.print(" Humi = ");  Serial.print(humidity);
    Serial.print(" Humi*10 = "); Serial.print(SensorData.relHumi);
    Serial.print(" Dewpt = ");  Serial.print(dewpoint);
    Serial.print(" DewPt*10 = "); Serial.print(SensorData.dewPt);
    Serial.println();
  #endif

  SensorData.index++;
  SensorData.windBurst = 0;
  initWindArray();
  idx = 0;  
}

//=======================================================
// init data structure
//=======================================================

void initData() {
  SensorData.deviceId = myDeviceId;
  SensorData.index = 0;
  SensorData.tempC = 0;
  SensorData.relHumi = 0;
  SensorData.dewPt = 0;
  SensorData.windSpd = 0;
  SensorData.windBurst = 0;
  SensorData.windDir = 0;
  SensorData.windDirV = 'S';
  SensorData.battVolt = 0;
  SensorData.pvVolt = 0;
  initWindArray();
}

//=======================================================
// init data structure
//=======================================================

void initWindArray() {
  int i;
  for (i = 0; i < 12; i++) {
    windSpdVals[i] = 0;
    windDirVals[1] = 0;
  }
}


//=======================================================
// Interrupt handler for anemometer. Called each time the reed
// switch triggers (two counts per revolution).
// added debounce delay
//=======================================================
void countAnemometer() {
   numRevsAnemometer++;
   delayMicroseconds(50);
}


//=======================================================
// Find vane direction.
//=======================================================
void calcWindDir() {
  int val;
  byte x, reading;

  val = analogRead(vanePin);
  val >>=2;                        // Shift to 255 range
  reading = val;

  // Look the reading up in directions table. Find the first value
  // that's >= to what we got.
  for (x=0; x<NUMDIRS; x++) {
     if (adc[x] >= reading)
        break;
  }
  //Serial.println(reading, DEC);
  x = (x + dirOffset) % 8;   // Adjust for orientation
  windDirVals[idx] = dirVals[x];
  SensorData.windDir = dirVals[x];
  
  #if DEBUG
    Serial.print("  Dir: ");
    Serial.println(dirVals[x]);
  #endif
}


//=======================================================
// Calculate the wind speed, and display it (or log it, whatever).
// 1 rev/sec = 1.492 mph   or 0.746 ??
//=======================================================
void calcWindSpeed() {
  int x, iSpeed = 0;
  // This will produce mph * 10
  // (didn't calc right when done as one statement)
  long speed = 746;  // 746 ?  14920
  speed *= numRevsAnemometer;
  speed /= MSECS_CALC_WIND_SPEED;
  iSpeed = speed;         // Need this for formatting below
  if ( idx < 12 )
    windSpdVals[idx++] = iSpeed;
  
  if (SensorData.windBurst < iSpeed) {
    SensorData.windBurst = iSpeed;
  }
  
  #if DEBUG
    Serial.print("Wind speed: ");
    x = iSpeed;
    Serial.print(x);
    Serial.print(" Count: ");
    Serial.print(numRevsAnemometer);
  #endif
  
   numRevsAnemometer = 0;        // Reset counter
}


//======================================================= 
// Interrupt handler for rainmeter. Called each time the reed 
// switch triggers (one tip of bucket). 
//======================================================= 
void countRainmeter() { 
  static unsigned long last_millis = 0; 
  unsigned long m = millis(); 
  if (m - last_millis < 200){ // ignore interrupt: probably a bounce problem 
  }else{ 
    numBuckettip++; 
    //Serial.print(numBuckettip); 
  } 
  last_millis = m; 
}

//=======================================================
// Calculate the rain amount 
//=======================================================
void calcRainAmt() {
//  SensorData.rainCnt = numBuckettip;
}

//=======================================================
// Calculate battery voltage
//=======================================================
void adRead() {
  int val, tmp = 0;
  
  val = analogRead(battPin);
  SensorData.battVolt = int(val * 0.064);      // batt voltage * 10  

  
  tmp = analogRead(solarPin);                  // read PV voltage before turning off mmpc
  pinMode(mmpcPin, OUTPUT);
  digitalWrite(mmpcPin, HIGH);                 // turn off mmpc - stops charger
  delay(1);
  val = analogRead(solarPin);                  // get the open circuit voltage of the PV panel
  digitalWrite(mmpcPin, LOW);
  pinMode(mmpcPin, INPUT);                     // enable mmpc on the charger

/*
  if (tmp < val) {
    SensorData.chState = 'C';
  } else {
    SensorData.chState = 'D';
  }
 */
 
  SensorData.pvVolt = int(val * 0.064);        // pv voltage * 10  

//  val = analogRead(lightPin);
//  SensorData.lightLvl = int(val * 0.064);     // light level voltage * 10  
}


