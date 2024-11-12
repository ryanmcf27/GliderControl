/*
  Teensy_Descent

  Custom code for Iowa State University's 2024-2025 Design Build Fly (DBF) team's X-1 glider
  
*/

/* Libraries */
#include <TaskScheduler.h> //for more info refer to source repository: https://github.com/arkhipenko/TaskScheduler/tree/master
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

/* Method Prototypes */
void updateGPS();
void updateRot();
void updateAccel();
void updateProx();
void strobe();

// set up a global struct to store our sensor data
struct SensorDataStruct {
  float gpsLat = 0.0;     //X-1's latitude (degrees)
  float gpsLng = 0.0;     //X-1's longitude (degrees)
  float roll = 0.0;       //roll angle (degrees) - may need to add offset if CodeCell isn't mounted flat
  float pitch = 0.0;      //pitch angle (degrees) - may need to add offset if CodeCell isn't mounted flat
  float yaw = 0.0;        //yaw angle (degrees) - may need to add offset if CodeCell isn't mounted flat
  //uint16_t proxDist = 0;  //distance detected by proximity sensor in <UNITS>
  float accelX = 0.0;     //X-1's acceleration in the X direction
  float accelY = 0.0;     //X-1's acceleration in the Y direction
  float accelZ = 0.0;     //X-1's acceleration in the Z direction
};

// global variables to store the coordinates of our target location
float targetLat = 32.265617;
float targetLng = -111.273524;

// set up enum for the glider's flight state, initialize to PRERELEASE state
enum {PRERELEASE, DESCENT, ERROR_STATE} flightState = PRERELEASE;

/* Constants and Pins */
static const int RXPin = 0, TXPin = 1;
static const uint32_t MONITOR_BAUD = 115200, GPS_BAUD = 9600;
static const uint16_t RELEASE_PROX_THRES = 10;

/* Tasks */
#define GPS_TASK_RATE 250
#define ROT_TASK_RATE 250
#define ACCEL_TASK_RATE 250
//#define PROX_TASK_RATE 250
#define STROBE_TASK_RATE 250
Task gpsTask(GPS_TASK_RATE, TASK_FOREVER, &updateGPS);      //update GPS global variables every GPS_TASK_RATE ms, will be enabled from the start
Task rotTask(ROT_TASK_RATE, TASK_FOREVER, &updateRot);      //update rotation global variables every ROT_TASK_RATE ms, will be enabled from the start
Task accelTask(ACCEL_TASK_RATE, TASK_FOREVER, &updateAccel);      //update acceleration global variables every ACCEL_TASK_RATE ms, will be enabled from the start
//Task proxTask(PROX_TASK_RATE, TASK_FOREVER, &updateProx);   //update proximity global variables every PROX_TASK_RATE ms, will be enabled from the start
Task strobeTask(STROBE_TASK_RATE, TASK_FOREVER, &strobe);   //strobe lights every STROBE_TASK_RATE ms, won't be enabled until release

/* Declaring Components */
Scheduler runner;                     //declare our task scheduler
TinyGPSPlus gps;                      //declare TinyGPSPlus object to handle GPS data (convert raw data to be more user friendly)
SensorDataStruct sensorData;          //declare struct to hold our sensor data
SoftwareSerial ss_GPS(RXPin, TXPin);  //declare serial connection to the GPS module 

void setup() {
  Serial.begin(MONITOR_BAUD);
  ss_GPS.begin(GPS_BAUD);
  runner.init();
  Serial.println("Initialized scheduler");
  
  runner.addTask(gpsTask);
  Serial.println("added gpsTask");
  
  runner.addTask(rotTask);
  Serial.println("added rotTask");

  runner.addTask(accelTask);
  Serial.println("added accelTask");

  //runner.addTask(proxTask);
  //Serial.println("added proxTask");

  runner.addTask(strobeTask);
  Serial.println("added strobeTask");

  delay(2500);
  
  gpsTask.enable();
  Serial.println("Enabled gpsTask");
  rotTask.enable();
  Serial.println("Enabled rotTask");
  //proxTask.enable();
  //Serial.println("Enabled proxTask");

  delay(2000);
}

void loop() {
  runner.execute();
  switch(flightState){
    case PRERELEASE:
      //checks to see if X-1 has been released based on proximity sensor's data
      //if(sensorData.proxDist >= RELEASE_PROX_THRES){
        //update flightState
       // flightState = DESCENT;
      //}
      break;
    case DESCENT:
      //TODO - logic for steering descent into target zone
      break;
    case ERROR_STATE:
      Serial.println("In ERROR flight state!!!");
      break;
  }
}

void updateGPS() {
  // get GPS data from i2C
  if (ss_GPS.available()){
    gps.encode(ss_GPS.read());
    if(gps.location.isUpdated()){
      //if the received gps data is successfully encoded, update global variables
      sensorData.gpsLat = gps.location.lat();
      sensorData.gpsLng = gps.location.lng();
      
      //for testing
      char buffer [50];
      sprintf(buffer, "Latitude: %f, Longitude: %f", sensorData.gpsLat, sensorData.gpsLng);
      Serial.println(buffer);
    }
  }
}

void updateRot() {
  // Update rotation global variables
  //myCodeCell.Motion_RotationRead(sensorData.roll, sensorData.pitch, sensorData.yaw);
}

void updateAccel() {
  // Update acceleration global variables
  //myCodeCell.Motion_AccelerometerRead(sensorData.accelX, sensorData.accelY, sensorData.accelZ);
}

//void updateProx() {
  // Update proximity distance global variable
  //sensorData.proxDist = myCodeCell.Light_ProximityRead();
//}

void strobe() {
  // TODO - method to strobe LEDs
}