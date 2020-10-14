
#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include <Servo.h> //Servo library
//Include Firebase ESP8266 library
#include <FirebaseArduino.h>
//Include ESP8266WiFi.h
#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// Objects for playing music
SoftwareSerial mySoftwareSerial(10, 11); // RX, TX
DFRobotDFPlayerMini myDFPlayer;

//Create firebase object


// servo object to control a servo
Servo myservo;

int pos=90; // variable to store the servo position
int rotate=10; //variable to store the servo rotation time

const int buttonPin = 0; // the number of the pushbutton pin
int buttonState = 0; //variable for reading the pushbutton status

const int calPin = 1; // the number of the recalibration click switch pin
int calState = 0; //variable for reading the recalibration click switch status

// For amp meter
const int sensorIn = A0;
int mVperAmp = 185; 

double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;

// Time server code 
const char *ssid = "YOUR_SSID"; 
const char *password = "YOUR_PASS"; 
 
const unsigned long utcOffsetInSeconds = 3600; 
 
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"}; 
 
// Define NTP Client to get time 
WiFiUDP ntpUDP; 
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds); 

String path = "boxes/abc123/compartments";

enum States { firebase_observe, refill, fetch};
States currentState = firebase_observe;


void setup() {
  WiFi.begin(ssid, password);

  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 );
    Serial.print ( "." );
  }

  timeClient.begin();
  
  //Setup Firebase credential in setup:
  Firebase.begin("https://engineering-design-58c77.firebaseio.com/","qazuoL8dbsUovsdDGaU4LTLNiuDwyvkDMq8972kb");
  Firebase.stream(path);

  //Setup of music:
  mySoftwareSerial.begin(9600);
  
  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true);
  }
  myDFPlayer.volume(15);
  
  // initialize serial communication:
  Serial.begin(115200);
  
  // attaches the servo on pin 8 to the servo object
  myservo.attach(8);
  
  //initialize the pushbutton and calibration pin as an input:
  pinMode(buttonPin, INPUT);
  pinMode(calPin, INPUT);

  pinMode(A0, INPUT);

}

void loop() {
  if (Firebase.available()){
     FirebaseObject firebaseCompartments = Firebase.get();
  }
  if (currentState == firebase_observe) {
    
  } else if (currentState == refill) {
    
  } else if (currentState == fetch) {
    
  }
  
   /*
  Serial.println(timeClient.getFormattedDate());
  Serial.println(timeClient.getFormattedTime());
  Serial.println(timeClient.getSeconds());
  Serial.print(":");
  Serial.print(":");
  Serial.print(timeClient.getHours());
  Serial.print(timeClient.getMinutes());
  Serial.print(", ");
  Serial.print(daysOfTheWeek[timeClient.getDay()]);
  */
  timeClient.update();
  int med;
  // Check value from database to see if medicine needs to be taken
  //if (Firebase.getInt(firebaseData, )){
  //  if (firebaseData.dataType() == "int") {
  //    med = firebaseData.intData();
  //  }
  //}
  // read value from app database to know if medicine needs to be taken and play music.
  if (med=1) {
    myDFPlayer.play(1);
  }


  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);


  //Activate motor when button is pressed and app gives a signal to
  if (buttonState == HIGH and med == 1) {
    myservo.write(pos);           // tell servo to go to rotate at speed written in variable 'pos'
    delay(rotate);                // the servo will continue to rotate at this speed for 'rotate' seconds
    
  }

  // Calibrate the system by rotating until the calibration click switch is pressed
  buttonState = digitalRead(calPin);
  if (/*systeem zegt dat ie moet kalibreren, niet dit hier rechts*/buttonState == LOW) {
    while (buttonState == LOW) {
      myservo.write(pos);         // tell servo to rotate at speed 'pos'
      delay(5);                   // rotate for 5 ms and then read value again
      buttonState = digitalRead(calPin);  
    }
  }
  
  // AmpMeter below
  Voltage = getVVP();
  VRMS = (Voltage/2.0) * 0.707; //square root
  AmpsRMS = (VRMS * 1000) / mVperAmp;
  float Wattage = (220 * AmpsRMS)-18; // Observed 18-20 Watt when no load was connected, so substracting offset value to get real consumption.
  
  delay(5);                //delay in between for stability
}

float getVVP(){
  float result;

  int readValue; // value read from the sensor
  int maxValue = 0; 
  int minValue = 1024;

  uint32_t start_time = millis();

  while((millis()-start_time) < 1000) { //sample for 1 sec
    readValue = analogRead(sensorIn);
    //see if we have a new maxValue
    if (readValue > maxValue){
      //record the maximum sensor value
      maxValue = readValue;
    }
    if (readValue < minValue){
      //record the maximum sensor value
      minValue = readValue;
    }
  }
  //subtract min from max
  result = ((maxValue - minValue) * 5)/1024.0;

  return result;
}

struct CompartmentHolder {
  char *date;
  char *state;
};
