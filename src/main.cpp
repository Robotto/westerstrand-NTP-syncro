#include <TimeLib.h>              //https://github.com/PaulStoffregen/Time
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include "drv8871.h"              //https://www.adafruit.com/product/3190

#define DCDC_Enable_Pin  D2


//function prototypes:
time_t getNtpTime();
int getDst(time_t epoch);
void sendNTPpacket(IPAddress &address);

DRV8871 driver(D1,D2);

//Buffers to hold current state of actual clock hardware:
unsigned int clockHours = 0;
unsigned int clockMinutes = 0;

// NTP Servers:
static const char ntpServerName[] = "dk.pool.ntp.org";

const int timeZone = 1;     // Central European Time

WiFiUDP Udp;
unsigned int localPort = 9001;  // local port to listen for UDP packets

void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println("AP: " + myWiFiManager->getConfigPortalSSID());
  Serial.println("IP: " + WiFi.softAPIP().toString());
}

void tick(int minutes){
  static bool polarity = LOW; //the westerstrand hadware requires every other 24V pulse to have reverse polarity

  digitalWrite(DCDC_Enable_Pin, HIGH); //enable DCDC step-up converter
  //TODO: measure and calibrate DCDC startup time
  delay(100); //let the step-up converter settle in. 

  for (int n = 0; n < minutes; ++n) //Pulse n minutes
  {
    if(polarity) driver.forward();
    else driver.reverse();
    delay(100); //wait for the minute hand to move
    driver.brake(); //ground the outputs of the motor driver.
    delay(100); //wait for the hardware to calm down from a pulse
    polarity = !polarity; //flip the polarity.

    //update memory to reflect hardware state:
    clockMinutes++;
    clockMinutes = clockMinutes % 60; //minutes go from 0 to 59
    clockHours = clockHours % 12; //hours go from 0 to 11 
  }

  digitalWrite(DCDC_Enable_Pin, LOW);
  delay(1500); //let the DCDC calm down
}

unsigned int getTicksToTime(int h, int m){
  //12 hour format of hourFormat12() is 1-12
  //12 hour format of clockHours is 0-11
  h = h % 12;

  int deltaHours = h - clockHours;
  if(deltaHours<0) deltaHours = 12 + deltaHours; //if clock is ahead of time (adding a negative number)
  
  int deltaMinutes = m - clockMinutes;
  if(deltaMinutes<0) deltaMinutes = 60 + deltaMinutes;

  unsigned int ticks = deltaMinutes + 60 * deltaHours;
  return ticks;
}


void setup()
{
  Serial.begin(115200);

  pinMode(DCDC_Enable_Pin, OUTPUT);
  digitalWrite(DCDC_Enable_Pin, LOW); //default to Off

  delay(2500);

  tick(1); //tick one minute to make sure clock hardware polarity is known.
  
  delay(5000); //wait five seconds for user to readjust the minute hand, in case it moved.

  WiFi.hostname("NTPClock");

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  Serial.println("Connecting to wifi..");
  wifiManager.setAPCallback(configModeCallback); //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setConnectTimeout(30); //try to connect to known wifis for a long time before defaulting to AP mode
  
  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("westerstrand_wifi_config")) {
    Serial.println("failed to connect and hit timeout");
    ESP.restart(); //reset and try again, or maybe put it to deep sleep 
  }

  Serial.println();
  Serial.print("WiFi up!");
  Serial.print("  IPv4: ");
  Serial.println(WiFi.localIP());

  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(Udp.localPort());
  Serial.println("waiting for sync");
  setSyncProvider(getNtpTime);
  setSyncInterval(3600); //Sync every hour
}

time_t prevDisplay = 0; // when the digital clock was displayed

void loop()
{
  if (timeStatus() != timeNotSet) {
    if (minute() != minute(prevDisplay)) { //only change time if a minute has passed.

      unsigned int deltaTimeInMinutes = getTicksToTime(hourFormat12(),minute());

      tick(deltaTimeInMinutes);

      prevDisplay = now();
    }
  }
}




/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address

  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 3000) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];

      uint32_t epochUTC=secsSince1900 - 2208988800UL;
      return epochUTC + timeZone * SECS_PER_HOUR + getDst(epochUTC);
      //return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

/*
 * Edited from avr-libc/include/util/eu_dst.h
 * (c)2012 Michael Duane Rice All rights reserved.
*/

//DST goes true on the last sunday of march @ 2:00 (CET)
//DST goes false on the last sunday of october @ 3:00 (CEST)
 
#define MARCH 3
#define OCTOBER 10
#define SHIFTHOUR 2 

int getDst(time_t epoch) { //eats local time (CET/CEST)

  uint8_t mon, mday, hh, day_of_week, d;
  int n;

  mon = month(epoch);
  day_of_week = weekday(epoch)-1; //paul's library sets sunday==1, this code expects "days since sunday" http://www.cplusplus.com/reference/ctime/tm/
  mday = day(epoch)-1; //zero index the day as well
  hh = hour(epoch);

  if ((mon > MARCH) && (mon < OCTOBER)) return 3600;
  if (mon < MARCH) return 0;
  if (mon > OCTOBER) return 0;

  //determine mday of last Sunday 
  n = mday;
  n -= day_of_week;
  n += 7;
  d = n % 7;  // date of first Sunday
  if(d==0) d=7; //if the month starts on a monday, the first sunday is on the seventh.

  n = 31 - d;
  n /= 7; //number of Sundays left in the month 

  d = d + 7 * n;  // mday of final Sunday 

  //If the 1st of the month is a thursday, the last sunday will be on the 25th.
  //Apparently this algorithm calculates it to be on the 32nd...
  //Dirty fix, until something smoother comes along:
  if(d==31) d=24;

  if (mon == MARCH) {
    if (d > mday) return 0;
    if (d < mday) return 3600;
    if (hh < SHIFTHOUR) return 0;
    return 3600;
  }
  //the month is october:
  if (d > mday) return 3600; 
  if (d < mday) return 0; 
  if (hh < SHIFTHOUR+1) return 3600;
  return 0;
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

