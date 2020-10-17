
#include "Arduino.h"
#include <Servo.h> //Servo library
#include <ESP8266WiFi.h>

// servo object to control a servo
Servo myservo;

int pos=1; // variable to store the servo position
int rotate=1000; //variable to store the servo rotation time

const int buttonPin = D7; // the number of the pushbutton pin
int buttonState = 0; //variable for reading the pushbutton status

const int calPin = D5; // the number of the recalibration click switch pin
const int ledPin = D8;

// Time server code 
const char *ssid = "Internet_kot"; 
const char *password = "teunsiem"; 

boolean isRotated = false;
 
enum States { firebase_observe, refill, fetch};
States currentState = refill;

int numberOfCompartmentsToFill = 5;
int currentCompartment = 0;

void setup() {
  // initialize serial communication:
  Serial.begin(115200);

  WiFi.begin(ssid, password);

  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 );
    Serial.print ( "." );
  }

  Serial.println("Connected!");

  myservo.attach(D6);
  
  //initialize the pushbutton and calibration pin as an input:
  pinMode(buttonPin, INPUT);
  pinMode(calPin, INPUT);

  pinMode(A0, INPUT);
  pinMode(D8, OUTPUT);

}

void loop() {
  if (currentState == firebase_observe) {
    Serial.println("Firebase Observe"); 

  } else if (currentState == refill) {
    Serial.println("REFILL STATE");

    if (currentCompartment < numberOfCompartmentsToFill) {
      if (!isRotated) {
        //Activate motor to go to specific compartment
        myservo.write(180);
        delay(rotate * 3.5);                // the servo will continue to rotate at this speed for 'rotate' seconds
        myservo.write(90);
        isRotated = true;
      }

      // turn on the light of the button
      digitalWrite(D8, HIGH);
    
      // read the state of the pushbutton value:
      buttonState = digitalRead(buttonPin);

      // if button is pressed
      if (buttonState == LOW) {
        Serial.println("Button is pressed");

        isRotated = false;
        currentCompartment++;
      }
      
    } else {
      // Calibrate the system by rotating until the calibration click switch is pressed
      buttonState = digitalRead(calPin);
      while (buttonState == HIGH) {
        Serial.println("calibrating");
        myservo.write(1);         // tell servo to rotate at speed 'pos'
        delay(5);                   // rotate for 5 ms and then read value again
        buttonState = digitalRead(calPin);
      }
      
      myservo.write(90);
      digitalWrite(D8, LOW);
      currentState = firebase_observe;
      currentCompartment = 0;
      isRotated = false;
    }

    
  } else if (currentState == fetch) {
    Serial.println("FETCH STATE");

    if (!isRotated) {
      //Activate motor to go to specific compartment
      myservo.write(180);
      //myservo.write(pos);           // tell servo to go to rotate at speed written in variable 'pos'
      delay(rotate * 3.5);                // the servo will continue to rotate at this speed for 'rotate' seconds
      myservo.write(90);
      isRotated = true;
    }

    // turn on the light of the button
    digitalWrite(D8, HIGH);
    
    // read the state of the pushbutton value:
    buttonState = digitalRead(buttonPin);

    // if button is  pressed
    if (buttonState == LOW) {
      Serial.println("Button is pressed");
      
      // Calibrate the system by rotating until the calibration click switch is pressed
      buttonState = digitalRead(calPin);
      while (buttonState == HIGH) {
        Serial.println("calibrating");
        myservo.write(1);         // tell servo to rotate at speed 'pos'
        delay(5);                   // rotate for 5 ms and then read value again
        buttonState = digitalRead(calPin);
      }
      
      myservo.write(90);
      digitalWrite(D8, LOW);
      currentState = firebase_observe;
      isRotated = false;
    }
  }
  delay(500);                //delay in between for stability
}
