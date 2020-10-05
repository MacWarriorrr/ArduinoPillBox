#include <Servo.h>
#include "pitches.h"

int melody[] = {
NOTE_D4, NOTE_G4, NOTE_FS4, NOTE_A4,
NOTE_G4, NOTE_C5, NOTE_AS4, NOTE_A4,                   
NOTE_FS4, NOTE_G4, NOTE_A4, NOTE_FS4, NOTE_DS4, NOTE_D4,
NOTE_C4, NOTE_D4,0,                                 

NOTE_D4, NOTE_G4, NOTE_FS4, NOTE_A4,
NOTE_G4, NOTE_C5, NOTE_D5, NOTE_C5, NOTE_AS4, NOTE_C5, NOTE_AS4, NOTE_A4,      //29               //8
NOTE_FS4, NOTE_G4, NOTE_A4, NOTE_FS4, NOTE_DS4, NOTE_D4,
NOTE_C4, NOTE_D4,0,                                       

NOTE_D4, NOTE_FS4, NOTE_G4, NOTE_A4, NOTE_DS5, NOTE_D5,
NOTE_C5, NOTE_AS4, NOTE_A4, NOTE_C5,
NOTE_C4, NOTE_D4, NOTE_DS4, NOTE_FS4, NOTE_D5, NOTE_C5,
NOTE_AS4, NOTE_A4, NOTE_C5, NOTE_AS4,             //58

NOTE_D4, NOTE_FS4, NOTE_G4, NOTE_A4, NOTE_DS5, NOTE_D5,
NOTE_C5, NOTE_D5, NOTE_C5, NOTE_AS4, NOTE_C5, NOTE_AS4, NOTE_A4, NOTE_C5, NOTE_G4,
NOTE_A4, 0, NOTE_AS4, NOTE_A4, 0, NOTE_G4,
NOTE_G4, NOTE_A4, NOTE_G4, NOTE_FS4, 0,

NOTE_C4, NOTE_D4, NOTE_G4, NOTE_FS4, NOTE_DS4,
NOTE_C4, NOTE_D4, 0,
NOTE_C4, NOTE_D4, NOTE_G4, NOTE_FS4, NOTE_DS4,
NOTE_C4, NOTE_D4, END

};

// note durations: 8 = quarter note, 4 = 8th note, etc.
int noteDurations[] = {       //duration of the notes
8,4,8,4,
4,4,4,12,
4,4,4,4,4,4,
4,16,4,

8,4,8,4,
4,2,1,1,2,1,1,12,
4,4,4,4,4,4,
4,16,4,

4,4,4,4,4,4,
4,4,4,12,
4,4,4,4,4,4,
4,4,4,12,

4,4,4,4,4,4,
2,1,1,2,1,1,4,8,4,
2,6,4,2,6,4,
2,1,1,16,4,

4,8,4,4,4,
4,16,4,
4,8,4,4,4,
4,20,
};



Servo myservo; // servo object to control a servo

int speed=90; // higher value, slower notes
int played=0; // variable to know if reminder has been played

int pos=90; // variable to store the servo position
int rotate=10 //variable to store the servo rotation time

const int buttonPin = 2; // the number of the pushbutton pin
int buttonState = 0; //variable for reading the pushbutton status

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600)
  // attaches the servo on pin 8 to the servo object
  myservo.attach(8);
  //initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
}

void loop() {
  // read value from app somehow to know if medicine needs to be taken.
  if (played==0) {
    for (int thisNote = 0; melody[thisNote]!=-1; thisNote++) {

    int noteDuration = speed*noteDurations[thisNote];
    tone(3, melody[thisNote],noteDuration*.95);
    Serial.println(melody[thisNote]);
    
    delay(noteDuration);
    
    noTone(3);
    }
    }

  
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);


  //Activate motor when button is pressed and app gives a signal to
  if (buttonState == HIGH) {
    myservo.write(pos);           // tell servo to go to rotate at speed written in variable 'pos'
    delay(rotate);                // the servo will continue to rotate at this speed for 'rotate' seconds
    
  }

  
  // Detect input of gas sensor:
  gasThreshold = 0;
  int gasValue = analogRead(A0);

  if gasValue > gasThreshold;
    Serial.println("teveel gas");




  delay(5);                //delay in between for stability
}
