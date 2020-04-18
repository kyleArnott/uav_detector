//Set up required libraries
#include <TinyGPS++.h>                              //for interfacing with the base GPS module
#include <math.h>                                   //for using arctan()
#include <SparkFun_MAG3110.h>                       //for using the magnometer
#include "DRV8825.h"                                //Driver for the stepper motors

//Define the serial ports as something more descriptive
#define baseGPSPort Serial5
#define droneGPSPort Serial1

//Defining Stepper motor items
#define DIR 29
#define STEP 30
#define MODE0 28
#define MODE1 27
#define MODE2 26
#define MOTOR_STEPS 200

#define droneDataReady 25

static const uint32_t GPSBaud = 9600;                     //For the serial5 setup

double avgBASE_LAT, avgBASE_LON, avgBASE_ALT;             //Average GPS variables. Ditto
int avgCounter = 0;                                       //Counts the number of running averages
float verticalGPSBearing;
float horizontaGPSBearing;
volatile bool baseDataFlag = false;

volatile int j = 0;
//char string[6];
volatile int flag = 0;
char *rubbish = NULL;
volatile double DRONE_LAT;
volatile double DRONE_LON;
volatile double DRONE_ALT;
volatile double BASE_LAT;
volatile double BASE_LON;
volatile double BASE_ALT;
char BUFFER[11] =  {'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'};
volatile long data = 0;

int GPSperiod = 5000000;
int startTimerValue = 0;

TinyGPSPlus gpsBase;                                      //Setup the TinyGPS++ object for the base data
TinyGPSPlus gpsDrone;                                     //Setup the TinyGPS++ object for the drone data

MAG3110 mag = MAG3110();                                  //Setup the magnometer object

DRV8825 stepper(MOTOR_STEPS, DIR, STEP, MODE0, MODE1, MODE2);

IntervalTimer baseGPSDataPull;

/////////////////////////////////////////////////////////////////////
// Recieve GPS data from the base reciever. Interupts every second //
/////////////////////////////////////////////////////////////////////

void baseGPSData() {

  cli();

  unsigned long start = millis();

  do {
    while (baseGPSPort.available()) {             //If the serial port connected to the base GPS has data
      gpsBase.encode(baseGPSPort.read());              //If the data is complete and ready to be used
      BASE_LAT = gpsBase.location.lat();          //Find the latitude
      BASE_LON = gpsBase.location.lng();          //Find the longitude
      BASE_ALT = gpsBase.altitude.meters();       //Find the altitude

      baseDataFlag = true;                   //Set the data flag high to allow further processing
     
    }
  } while (millis() - start < 1000);


  baseDataFlag = false;
  sei();
  
}

/////////////////////////////////////////////////////////////////////
// Recieves GPS data from ESP8266. Called via a pin high interrupt //
/////////////////////////////////////////////////////////////////////

void droneGPSData() {
  if (droneGPSPort.available() > 0) {
    BUFFER[j] = droneGPSPort.read();
    if (BUFFER[j] == 't') {
      //If the string in the buffer terminates with a t it is the laTitude da
      //Convert the current contents of the buffer into an int, save it and reset the buffer
      long LAT = atoi(BUFFER);
      DRONE_LAT = (double)LAT / 1000000;
//      Serial.print("LAT ");
//      Serial.print(BASE_LAT);
//      Serial.print(" ");
//      Serial.println(DRONE_LAT);
      bufferReset(BUFFER);
      j = 0;
    } else if (BUFFER[j] == 'n') {
      //If the string in the buffer terminates with an n it is the loNgitude data
      //Convert the current contents of the buffer into an int, save it and reset the buffer
      long LON = atoi(BUFFER);
      DRONE_LON = (double)LON / 10000000;
//      Serial.print("LON ");
//      Serial.print(BASE_LON);
//      Serial.print(" ");
//      Serial.println(DRONE_LON);
      bufferReset(BUFFER);
      j = 0;
    } else if (BUFFER[j] == 'a') {
      //If the string in the buffer terminates with an a it is the Altiude data
      //Convert the current contents of the buffer into an int, save it and reset the buffer
      long ALT = atoi(BUFFER);
      DRONE_ALT = (double)ALT;
//      Serial.print("ALT ");
//      Serial.print(BASE_ALT);
//      Serial.print(" ");
//      Serial.println(DRONE_ALT);
      bufferReset(BUFFER);
      j = 0;
    } else if (BUFFER[j] == '\n') {
      j = 0;
      flag = 1;
    } else {
      j++;
    }
  }
}


///////////
// Setup //
///////////

void setup()
{
  Serial.begin(9600);                                     //Initialise the USB serial monitor
  baseGPSPort.begin(9600);                                //Initialise serial5 UART port for the GPS module data
  droneGPSPort.begin(9600);                               //Initialise serial4 UART port for the detecotr teensy data

  pinMode(droneDataReady, INPUT);

  stepper.begin(200, 1);                                  //Initialise the stepper motor
  digitalWrite(MODE2, 1);
  digitalWrite(MODE1, 0);
  digitalWrite(MODE0, 0);
  stepper.enable();

  mag.initialize();                                     //Initialise the magnometer

  attachInterrupt(digitalPinToInterrupt(droneDataReady), droneGPSData, HIGH);

  startTimerValue = baseGPSDataPull.begin(baseGPSData, GPSperiod);

}

///////////////
// Main loop //
///////////////

void loop() {

  stepper.setMicrostep(1);            // Set microstep mode to 1:1
  int16_t x, y, z;                    //x, y, z values for the magnometer
  //Calibrate the magnometer
  if (!mag.isCalibrated()) {          //If we're not calibrated

    if (!mag.isCalibrating()) {       //And we're not currently calibrating
      mag.enterCalMode();             //Enter calbiration
      stepper.rotate(360);            //Rotate platform 360 degrees
      delay(1000);

    } else {
      mag.calibrate();
    }
  }

   Serial.println("1");

  //These base_lat etc will populated by a serial connection, in the mean-time these are for testing
  DRONE_LAT = 55.871157;
  DRONE_LON = -4.2886805;
  DRONE_ALT = 100;
  
  Serial.println("2");
  
  //Create a running average of the stations location IF there is data form the base station
//  if (baseDataFlag = true) {
//    void gpsBearing();
//    avgCounter++;
//    void baseRunningAverage();
//    baseDataFlag = false;
//  }

  //if (flag == 1) {
  gpsBearing();
  //}
  Serial.println("3");

}

////////////////////////////////////////////////////
// Create a running average of the baseGPS coords //
////////////////////////////////////////////////////

void baseRunningAverage(double BASE_LAT, double BASE_LON, double DRONE_ALT, int avgCounter) {

  if (avgCounter <= 60) {

    double netBASE_LAT = + BASE_LAT;
    avgBASE_LAT = netBASE_LAT / avgCounter;

    double netBASE_LON = + BASE_LON;
    avgBASE_LON = netBASE_LON / avgCounter;

    double netBASE_ALT = + BASE_ALT;
    avgBASE_ALT = netBASE_ALT / avgCounter;

  } else {

    //Resets the averages every 60 seconds

    avgBASE_LAT = 0;
    avgBASE_LON = 0;
    avgBASE_ALT = 0;

    avgCounter = 0;
  }

  return;

}

///////////////////////////////////////////////////
// Calculates desired bearings based on GPS data //
///////////////////////////////////////////////////

void gpsBearing() {

  //Calculates the distance to the dronw station. In impelementation drone_lat/drone_lon will be recieved in an interrupt
  unsigned long distanceKmToBase =
    (unsigned long)TinyGPSPlus::distanceBetween(
      BASE_LAT,
      BASE_LON,
      DRONE_LAT,
      DRONE_LON);

  //Calculates the initial bearing that would lead to the drone
  double courseToBase =
    TinyGPSPlus::courseTo(
      BASE_LAT,
      BASE_LON,
      DRONE_LAT,
      DRONE_LON);

  //Calculates the vertical bearing
  double altitudeDifference = BASE_ALT - DRONE_ALT;
  float verticalBearing = (atan(altitudeDifference / distanceKmToBase)) * 180 / PI;

    int16_t x, y, z;                                              //x, y, z values for the magnometer
    float magHeading;
  
    //Calculates the horizontal bearing
    mag.readMag(&x, &y, &z);
  
    if (mag.readHeading() <= 0) {
      magHeading = mag.readHeading();
      magHeading += 360;
    } else {
      magHeading = mag.readHeading();
    }

  float horizontalError = (courseToBase - (360 - magHeading));
  
  Serial.print("Desired Bearing: ");
  Serial.println(courseToBase);
  Serial.print("Current Bearing: ");
  Serial.println(horizontalError);
  Serial.println("Distance Between: ");
  Serial.println(distanceKmToBase);
 
  
  //  Serial.print("Lat: ");
  //  Serial.println(BASE_LON - DRONE_LON);
  //  Serial.print("Lon: ");
  //  Serial.println(BASE_LAT - DRONE_LAT);

  flag = 0;

  return;
}

///////////////////////////////////////////////////////////////
// Converts the bearings found into degrees of the rig motor //
///////////////////////////////////////////////////////////////

void azimuthRotate(int oldBearing) {
  int newBearing = oldBearing * 0.1;
  stepper.rotate(newBearing);
}

void altitudeRotate(int oldBearing) {
  int newBearing = oldBearing * 0.1;
  stepper.rotate(newBearing);
}

void bufferReset(char *BUFFER) {
  for (int i = 0; i == 10; i++) {
    BUFFER[i] = 0;
  }
}
