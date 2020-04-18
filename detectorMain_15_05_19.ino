#include <Arduino.h>
#include "DRV8825.h"

//Using a 200-step motor (most common)
#define MOTOR_STEPS 200

//Define the serial port as something more descriptive
#define detectorUART Serial4

//Configure the pins used by the motor
#define DIR 29
#define STEP 30
#define MODE0 28
#define MODE1 27
#define MODE2 26

//Pin used by the esp to trigger the data pull from the teensy
#define detectorInteruptPin 24

//Initialise the stepper motor
DRV8825 stepper(MOTOR_STEPS, DIR, STEP, MODE0, MODE1, MODE2);

const byte numChars = 32;                 //Define the size of the char array
char receivedChars[numChars];             //Initialise an array to store the received data
volatile int differential = 0;                    //Where the recieved data is stored

///////////////////////////////////////////////////////////////////////
// Recieves data from the detector teensy. Called via a pin interupt //
///////////////////////////////////////////////////////////////////////

void detectorData() {

  static byte ndx = 0;                    //Recieved char index
  char endMarker = '\n';                  //endMarker = newline char
  char rc;                                //Recieved char
  
  if (detectorUART.available() > 0) {     //Is there data on the detector UART port?
    rc = detectorUART.read();             //Read the port and save it into rc

    if (rc != endMarker) {                //Have we reached the end of the sentance?
      receivedChars[ndx] = rc;            //If we have not then fill the current position in the char array with the rc
      ndx++;                              //Incriment ndx ready for the next char
      if (ndx >= numChars) {              //If the index is greater than or equal to the predefined size of the char array
        ndx = numChars - 1;               //Set the index to (size of array - 1)
      }
    } else {
      receivedChars[ndx] = '\0';          //If \n has been recieved then terminate the array
      ndx = 0;                            //Reset the counter
      differential = atoi(receivedChars);         //Convert the received char array into an intiger
      digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN) );
      Serial.println(differential);
    }
  }
}

///////////
// Setup //
///////////

void setup() {

  // Set target motor RPM to 1RPM and microstepping to 1 (full step mode) 4 has a good accuracy and torque
  Serial.begin(9600);
  detectorUART.begin(9600);

  //Initialise the stepper with RPM = 200 and full step mode
  stepper.begin(200, 1);

  //Write full step mode directly to the pins
  digitalWrite(MODE2, 1);
  digitalWrite(MODE1, 0);
  digitalWrite(MODE0, 0);

  //Setup the interupt pin
  pinMode(detectorInteruptPin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  //Create an interupt triggered by the above pin, set it to trigger 'detector data' on a pin = HIGH
  attachInterrupt(digitalPinToInterrupt(detectorInteruptPin), detectorData, HIGH);
}

/////////////////////////////////////////////////////////////////////
// Main loop - takes recieved data and moves the motor accordingly //
/////////////////////////////////////////////////////////////////////

void loop() {

  cli();

  //If at the centre do not move
  if ((differential > 0) && (differential < 30)) {

    digitalWrite(MODE2, 0);
    digitalWrite(MODE1, 0);
    digitalWrite(MODE0, 0);

    stepper.disable();
    azimuthRotate(20);
    delay(1000);
    azimuthRotate(-20);
    delay(1000);
  }

  //If at the far left
  else if (differential > 400) {

    //Set to full step mode
    digitalWrite(MODE2, 1);
    digitalWrite(MODE1, 0);
    digitalWrite(MODE0, 0);

    // Tell motor to rotate by 5 * the scaler contained in azimuthRotate
    stepper.enable();
    azimuthRotate(5);
  }
  //left
  else if ((differential > 300) && (differential < 400)) {
    
    // 1/4 step mode
    digitalWrite(MODE2, 1);
    digitalWrite(MODE1, 1);
    digitalWrite(MODE0, 0);

    stepper.enable();
    azimuthRotate(20);
  }
  //slight left
  else if ((differential > 30) && (differential < 300)) {
    // 1/8 step mode
    digitalWrite(MODE2, 1);
    digitalWrite(MODE1, 1);
    digitalWrite(MODE0, 1);

    stepper.enable();
    azimuthRotate(40);
  }

  //far right
  else if (differential < -400) {
    //full step mode
    digitalWrite(MODE2, 1);
    digitalWrite(MODE1, 0);
    digitalWrite(MODE0, 0);

    // Tell motor to rotate 1 degree. That's it.
    stepper.enable();
    azimuthRotate(-5);
  }
  //right
  else if ((differential < -300) && (differential > -400)) {
    // 1/4 step mode
    digitalWrite(MODE2, 1);
    digitalWrite(MODE1, 1);
    digitalWrite(MODE0, 0);

    stepper.enable();
    azimuthRotate(-20);
  }
  //slight right
  else if ((differential < -30) && (differential > -300)) {
    // 1/8 step mode
    digitalWrite(MODE2, 1);
    digitalWrite(MODE1, 1);
    digitalWrite(MODE0, 1);

    stepper.enable();
    azimuthRotate(-40);
  }
  
  sei();
  delay(1);
}

//////////////////////////////////////////////////////////////////////////////////////
// Takes in an number of steps, scales it, then makes the motor rotate by that many //
//////////////////////////////////////////////////////////////////////////////////////

void azimuthRotate(int oldBearing) {
  int newBearing = oldBearing * 7.5;
  stepper.move(newBearing);
}
