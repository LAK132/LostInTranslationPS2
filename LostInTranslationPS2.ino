#define DEBUG false
/*
 * ESP8266 WiFi receiver for "Lost In Translation" beetleweight combat robot
 * Based on the code for the "Wheely Fast" antweight combat robot 
 */

#include "PS2X_lib.h"
#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <Ticker.h>
#include "MotorController.h"
#include "Servo.h"

extern "C" {
#include "user_interface.h"
}

//////////////////////
// WiFi Definitions //
//////////////////////
const char *password = "12345678";      // This is the Wifi Password (only numbers and letters,  not . , |)
String AP_Name = "WheelyFast";             // This is the Wifi Name(SSID), some numbers will be added for clarity (mac address)
byte vibrate = 0;
PS2X ps2x; // create PS2 Controller Class

void setupWiFi(void);

/////////////////////
// Pin Definitions //
/////////////////////

// stepper with direction and speed pins, don't use D0 for speed
const int motorLeftDir  = D2;
const int motorLeftSpd  = D3;
const int motorRightDir = D5;//D0;
const int motorRightSpd = D8;//D2;
//const int motorLeftA  = D5;
//const int motorLeftB  = D6;
//const int motorRightA = D3;
//const int motorRightB = D2;
motorController motors(motorLeftDir, motorLeftSpd, motorRightDir, motorRightSpd);
//motorController motors(motorLeftA,motorLeftB,motorRightA,motorRightB, true);
const int motorWeapon = D1;//D5;
const int motorWeaponMinMilli = 544;
const int motorWeaponMaxMilli = 2400;
const int weaponMax = 90;
const int weaponMin = 10;
const int weaponStep = 10;
int weaponSpeed = 20;
Servo weapon;

const int steeringSensitivity = 0.6;

// UDP variables
unsigned int localPort = 8888;
WiFiUDP UDP;
boolean udpConnected = false;
IPAddress remoteIPAddress;
int remotePortAddress = 0;

Ticker HeartBeatTicker;

bool HeartBeatRcvd = false;

void Stop(void)
{
  motors.update(0, 0);
}

void CheckHeartBeat(void)
{
  if (HeartBeatRcvd == true)
  {
    HeartBeatRcvd = false;
  }
  else
  {
    Stop();
  }
}

void setup()
{
  system_update_cpu_freq(80);        // set cpu to 160MHZ !
  Serial.begin(115200);
  delay(100);
  setupWiFi();
  HeartBeatTicker.attach_ms(500, CheckHeartBeat);
  Stop();
  //motors.setTrim(1.0,1.0);            // this setting is optional, it compensates for speed difference of motors eg (0.95,1.0), and it can reduce maximum speed of both eg (0.75,0.75);
  motors.setSteeringSensitivity(0.15);  // this setting is optional
  motors.setPWMFrequency(1000);           // this setting is optional, depending on power supply and H-Bridge this option will alter motor noise and torque.
  //motors.setMinimumSpeed(0.10);         // this setting is optional, default is 0.1(10%) to prevent motor from stalling at low speed
  pinMode(D7, OUTPUT);
  digitalWrite(D7, HIGH); // Turn on onboard LED

  weapon.attach(motorWeapon, motorWeaponMinMilli, motorWeaponMaxMilli);
  //weapon.writeMicroseconds(0);
  weapon.write(90); //Stop motor
}

void loop()
{
  if (!checkForIncomingData())return;
  if (!udpConnected || !remotePortAddress) return;
  processData();
}

void setupWiFi()
{
  WiFi.mode(WIFI_AP);
  char AP_NameChar[AP_Name.length() + 1];
  AP_Name.toCharArray(AP_NameChar, AP_Name.length() + 1);
  int channel = random(1, 13 + 1);              // have to add 1 or will be 1 - 12
  IPAddress subnet(255, 255, 255, 0);
  IPAddress apIP(192, 168, 1, 1);
  WiFi.softAPConfig(apIP, apIP, subnet);
  WiFi.softAP(AP_NameChar, password , channel , 0 );
  Serial.println("");
  if (UDP.begin(localPort) == 1) {
    Serial.println(F("Ready for Controller to connect"));
    udpConnected = true;
  }
  else {
    Serial.println(F("Connection failed"));
  }
}

boolean checkForIncomingData() 
{
  if (udpConnected) 
  {
    int packetSize = UDP.parsePacket();
    if (packetSize) 
    { // if there’s data available, read a packet
      char packetBuffer[packetSize];
      boolean firstPacketSkipped = remotePortAddress;
      if (!remotePortAddress || remotePortAddress != UDP.remotePort())Serial.println("Controller Connected, Port: " + String(UDP.remotePort()));
      if (packetSize) // have data
      {
        remoteIPAddress = UDP.remoteIP(); // update client details
        remotePortAddress = UDP.remotePort();
        // read the packet into packetBuffer
        UDP.read(packetBuffer, packetSize);
        //Serial.println(packetBuffer);
        // send to library
        for (int a = 0; a < 21; a++) 
        {
          ps2x.PS2data[a] = packetBuffer[a];
        }
        ps2x.last_buttons = ps2x.buttons; //store the previous buttons states
        ps2x.buttons = (ps2x.PS2data[4] << 8) + ps2x.PS2data[3]; //store as one value for multiple functions
        //Serial.print(F("Free Ram: "));
        //Serial.println(system_get_free_heap_size());
        if (!firstPacketSkipped)return false; // to set buttons to default skip first packet
        return true;
      }
    }
  }
  return false;
}

int translateStick(int value, int newRange)
{
  int newValue = 0;
  int deadZone = 30;//70;
  int upperZone = 127 + (deadZone * 0.5);
  if (value > upperZone) 
  {
    newValue = map(value , upperZone, 255 , 1 , newRange);
  }
  int lowerZone = 127 - (deadZone * 0.5);
  if (value < lowerZone) 
  {
    newValue = map(value , lowerZone, 1 , -1 , -newRange);
  }
  return newValue;
}

bool DUP = false;
bool DDOWN = false;

void processData()
{
  HeartBeatRcvd = true;
  /*if (ps2x.ButtonReleased(PSB_CROSS)) // Flip back up the right way
  {
    int dY = 500;
    int dX = 0;
    motors.update(dX, dY); //recieved data, must be connected
    delay(200);
  }*/
/*  else if (ps2x.Button(PSB_L1 ))
  {
    int range = 200;
    motors.setSteeringSensitivity(0.6);
    if (ps2x.Button(PSB_L2))
    {
      range = 500; // Boost drive mode
      motors.setSteeringSensitivity(0.2);
    }
    int dY = translateStick(ps2x.Analog(PSS_LY) , range); // Left stick forward and Back
    //if (ps2x.Button(PSB_R2))range = 500; // slow steering mode
    int dX = translateStick(ps2x.Analog(PSS_RX) , range); // Right Stick steering
    motors.update(dX, dY); //recieved data, must be connected
  }
  else*/
  /*{
    int range = 200;
    motors.setSteeringSensitivity(0.6);
    if (ps2x.Button(PSB_L2))
    {
      range = 500; // Boost drive mode
      motors.setSteeringSensitivity(0.2);
    }
    int dY = translateStick(ps2x.Analog(PSS_LY) , range); // Left stick forward and Back
    //if (ps2x.Button(PSB_R2))range = 500; // slow steering mode
    int dX = translateStick(ps2x.Analog(PSS_RX) , range); // Right Stick steering
    if (ps2x.Button(PSB_L1 ))motors.update(dX*-1, dY*-1); // reverse controls if upside down
    else   motors.update(dX, dY); //recieved data, must be connected
  }*/
  const int range = 500;
  motors.setSteeringSensitivity(steeringSensitivity);
  int dY = translateStick(ps2x.Analog(PSS_LY), range);
  int dX = translateStick(ps2x.Analog(PSS_RX), range);
  bool motorForward = ps2x.Button(PSB_R2);
  bool motorReverse = ps2x.Button(PSB_R1);
  bool L1 = ps2x.Button(PSB_L1);

  bool UP = ps2x.Button(PSB_PAD_UP);
  bool DOWN = ps2x.Button(PSB_PAD_DOWN);

  if(!DUP && UP)
  {
    DUP = true;
    int temp = weaponSpeed + weaponStep;
    weaponSpeed = (temp <= weaponMax ? (temp >= weaponMin ? temp : weaponMin) : weaponMax);
  }
  else if(!DDOWN && DOWN)
  {
    DDOWN = true;
    int temp = weaponSpeed - weaponStep;
    weaponSpeed = (temp <= weaponMax ? (temp >= weaponMin ? temp : weaponMin) : weaponMax);
  }
  else if (DUP || DDOWN)
  {
    DUP = UP;
    DDOWN = DOWN;
  }
  
  if(L1)
  {
    motors.update(dX*-1, dY*-1);
    if(motorReverse) weapon.write(90 - weaponSpeed);
    else if(motorForward) weapon.write(90 + weaponSpeed);
    else weapon.write(90);
  }
  else
  {
    motors.update(dX, dY);
    if(motorReverse) weapon.write(90 + weaponSpeed);
    else if(motorForward) weapon.write(90 - weaponSpeed);
    else weapon.write(90);
  }

  if(DEBUG)
  {
    if(L1)
    {
      Serial.print("Drive: "); Serial.print(dX*-1); Serial.print(" "); Serial.println(dY*-1);
      Serial.print("Weapon: ");
      if(motorReverse) Serial.println(90 - weaponSpeed);
      else if(motorForward) Serial.println(90 + weaponSpeed);
      else Serial.println(90);
    }
    else
    {
      Serial.print("Drive: "); Serial.print(dX); Serial.print(" "); Serial.println(dY);
      Serial.print("Weapon: ");
      if(motorReverse) Serial.println(90 + weaponSpeed);
      else if(motorForward) Serial.println(90 - weaponSpeed);
      else Serial.println(90);
    }
  }
}

