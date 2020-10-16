
#include "Arduino.h"
#include <SoftwareSerial.h>
#include <Servo.h> //Servo library
//Include Firebase ESP8266 library
#include <FirebaseArduino.h>
//Include ESP8266WiFi.h
#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <LinkedList.h>

////////////////////////////////////////////////////////////////////////////////////
//all the commands needed in the datasheet(http://geekmatic.in.ua/pdf/Catalex_MP3_board.pdf)
static int8_t Send_buf[8] = {0} ;//The MP3 player undestands orders in a 8 int string
                                 //0X7E FF 06 command 00 00 00 EF;(if command =01 next song order) 
#define NEXT_SONG 0X01 
#define PREV_SONG 0X02 

#define CMD_PLAY_W_INDEX 0X03 //DATA IS REQUIRED (number of song)

#define VOLUME_UP_ONE 0X04
#define VOLUME_DOWN_ONE 0X05
#define CMD_SET_VOLUME 0X06//DATA IS REQUIRED (number of volume from 0 up to 30(0x1E))
#define SET_DAC 0X17
#define CMD_PLAY_WITHVOLUME 0X22 //data is needed  0x7E 06 22 00 xx yy EF;(xx volume)(yy number of song)

#define CMD_SEL_DEV 0X09 //SELECT STORAGE DEVICE, DATA IS REQUIRED
                #define DEV_TF 0X02 //HELLO,IM THE DATA REQUIRED
                
#define SLEEP_MODE_START 0X0A
#define SLEEP_MODE_WAKEUP 0X0B

#define CMD_RESET 0X0C//CHIP RESET
#define CMD_PLAY 0X0D //RESUME PLAYBACK
#define CMD_PAUSE 0X0E //PLAYBACK IS PAUSED

#define CMD_PLAY_WITHFOLDER 0X0F//DATA IS NEEDED, 0x7E 06 0F 00 01 02 EF;(play the song with the directory \01\002xxxxxx.mp3

#define STOP_PLAY 0X16

#define PLAY_FOLDER 0X17// data is needed 0x7E 06 17 00 01 XX EF;(play the 01 folder)(value xx we dont care)

#define SET_CYCLEPLAY 0X19//data is needed 00 start; 01 close

#define SET_DAC 0X17//data is needed 00 start DAC OUTPUT;01 DAC no output
////////////////////////////////////////////////////////////////////////////////////

// Objects for playing music
SoftwareSerial mySerial(3, 1); // RX, TX
int played = 0;

// servo object to control a servo
Servo myservo;

int pos=90; // variable to store the servo position
int rotate=10; //variable to store the servo rotation time

const int buttonPin = 0; // the number of the pushbutton pin
int buttonState = 0; //variable for reading the pushbutton status

const int calPin = 1; // the number of the recalibration click switch pin

// For amp meter
const int sensorIn = A0;
int mVperAmp = 185; 

double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;

const int wattLimit = 200;

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

class CompartmentHolder {
  public:
    char *id;
    char *date;
    char *state;
};

LinkedList<CompartmentHolder> *compartments = new LinkedList<CompartmentHolder>();

void setup() {
  // initialize serial communication + MP3 module:
  Serial.begin(115200);
  mySerial.begin(115200);
  delay(500);
    sendCommand(CMD_SEL_DEV, DEV_TF);//select the TF card
  delay(200);
  
  WiFi.begin(ssid, password);

  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 );
    Serial.print ( "." );
  }

  timeClient.begin();
  
  //Setup Firebase credential in setup:
  Firebase.begin("https://engineering-design-58c77.firebaseio.com/","qazuoL8dbsUovsdDGaU4LTLNiuDwyvkDMq8972kb");
  Firebase.stream(path);
  
  // attaches the servo on pin 8 to the servo object
  myservo.attach(8);
  
  //initialize the pushbutton and calibration pin as an input:
  pinMode(buttonPin, INPUT);
  pinMode(calPin, INPUT);

  pinMode(A0, INPUT);

}

void loop() {
  //update current date + time
  timeClient.update();
  String dayStamp = timeClient.getFullFormattedTime();
  
  if (Firebase.available()){
     compartments->clear();
     JsonVariant compartmentsJSON = Firebase.get(path).getJsonVariant();
     if (Firebase.success()) {
      for( const auto& compartment : compartmentsJSON.as<JsonArray>() ) {

        for( const auto& kv : compartment.as<JsonObject>() ) {
          if (strcmp(kv.key, "id") == 0) {
            // id and the key are the same
            // id = kv.value.as<char>
          } else if (strcmp(kv.key, "date") == 0) {
            // date = kv.value.as<char>
          } else if (strcmp(kv.key, "state") == 0) {
            // state = kv.value.as<char*>
          }
        }
      }
     }

  }
  if (currentState == firebase_observe) {
    
  } else if (currentState == refill) {
    
  } else if (currentState == fetch) {
    // read the state of the pushbutton value:
    buttonState = digitalRead(buttonPin);
    //Activate motor to go to specific compartment
    myservo.write(pos);           // tell servo to go to rotate at speed written in variable 'pos'
    delay(rotate);                // the servo will continue to rotate at this speed for 'rotate' seconds
    if (played == 0){
      // play music.
      sendCommand(CMD_PLAY_WITHVOLUME, 0X0F01);//play the first song with volume 15 class
      played = 1;
    }
    if (buttonState == HIGH) {
      sendCommand(STOP_PLAY, 0X0F01);
      played = 0;

      // Calibrate the system by rotating until the calibration click switch is pressed
      buttonState = digitalRead(calPin);
      while (buttonState == LOW) {
        myservo.write(pos);         // tell servo to rotate at speed 'pos'
        delay(5);                   // rotate for 5 ms and then read value again
        buttonState = digitalRead(calPin);
        int wattage = getWatt();
        if (wattage > wattLimit) {
            break;
        }
      }
    }
    
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
  delay(5);                //delay in between for stability
}

// AmpMeter below
float getWatt (){
  Voltage = getVVP();
  VRMS = (Voltage/2.0) * 0.707; //square root
  AmpsRMS = (VRMS * 1000) / mVperAmp;
  float Wattage = (220 * AmpsRMS)-18; // Observed 18-20 Watt when no load was connected, so substracting offset value to get real consumption.
  return Wattage;
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


void sendCommand(int8_t command, int16_t dat)
{
 delay(20);
 Send_buf[0] = 0x7e; //starting byte
 Send_buf[1] = 0xff; //version
 Send_buf[2] = 0x06; //the number of bytes of the command without starting byte and ending byte
 Send_buf[3] = command; //
 Send_buf[4] = 0x00;//0x00 = no feedback, 0x01 = feedback
 Send_buf[5] = (int8_t)(dat >> 8);//datah
 Send_buf[6] = (int8_t)(dat); //datal
 Send_buf[7] = 0xef; //ending byte
 for(uint8_t i=0; i<8; i++)//
 {
   mySerial.write(Send_buf[i]) ;//send bit to serial mp3
   Serial.print(Send_buf[i],HEX);//send bit to serial monitor in pc
 }
 Serial.println();
}
