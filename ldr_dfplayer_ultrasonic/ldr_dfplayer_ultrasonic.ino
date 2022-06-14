
#define echoPin 25 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 26//attach pin D3 Arduino to pin Trig of HC-SR04
#define ldrPin 15//attach pin D3 Arduino to pin Trig of HC-SR04
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

#include <TinyGPS++.h>
TinyGPSPlus gps;

#include <SoftwareSerial.h>
#define MYPORT_TX 12
#define MYPORT_RX 13
SoftwareSerial gpsSerial;

#include "Arduino.h"
#include "DFRobotDFPlayerMini.h"

HardwareSerial mySerial(1); // RX, TX
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);
long timer1,timer2;
#define buzz 14

void setup()
{
  gpsSerial.begin(9600, SWSERIAL_8N1, MYPORT_RX, MYPORT_TX, false);
  mySerial.begin(9600,SERIAL_8N1,16,17);
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  pinMode(ldrPin, INPUT); // Sets the echoPin as an INPUT
  // pinMode(buzz, OUTPUT); // Sets the echoPin as an INPUT
  
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  
  if (!myDFPlayer.begin(mySerial)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true){
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }
  Serial.println(F("DFPlayer Mini online."));
  
  myDFPlayer.volume(50);  
  timer1=millis();
}

void loop()
{
  gps_connect();
  timer2=millis();
  int light;
  
  static unsigned long timer = millis();
  light =digitalRead(ldrPin);
  if(light==1)
  {
    if((timer2-timer1)>10000){
    timer1=timer2;
    Serial.println("dark");
    myDFPlayer.play(2);
    // digitalWrite(buzz,HIGH);
    delay(2000);
    // digitalWrite(buzz,LOW);

    }
  }
  else{
    if((timer2-timer1)>20000){
    timer1=timer2;
    Serial.println("light");
    // myDFPlayer.play(3);
    }
  }
  distance =get_distance();
  
    if (distance<10){
    myDFPlayer.play(1); 
    delay(3000); //Play next mp3 every 3 second.
    }
    else
    {
      myDFPlayer.stop();
    }
  
  
}

int get_distance(){
  int dis;
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  dis = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  // Serial.print("Distance: ");
  // Serial.print(dis);
  // Serial.println(" cm");
  return dis;
}
void gps_connect()
{
  boolean newData = false;
  // for (unsigned long start = millis(); millis() - start < 2000;)
  // {
    while (gpsSerial.available())
    {
      if (gps.encode(gpsSerial.read()))
      {

        newData = true;
      }
    }
  // }
  if (newData)   
  {
    unsigned long age;
    Serial.print("Latitude= ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(" Longitude= ");
    Serial.println(gps.location.lng(), 6);
    newData = false;

  }
}


