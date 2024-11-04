/*
  AutonomousDescent_ALPHA_1.0

  Custom code for Iowa State University's 2024-2025 Design Build Fly (DBF) team's X-1 glider
  
*/

/* Libraries */
#include <TaskScheduler.h> //for more info refer to source repository: https://github.com/arkhipenko/TaskScheduler/tree/master
#include <CodeCell.h>
#include <TinyGPSPlus.h>
#include <SparkFun_I2C_GPS_Arduino_Library.h> //Use Library Manager or download here: https://github.com/sparkfun/SparkFun_I2C_GPS_Arduino_Library
I2CGPS myI2CGPS; //Hook object to the library

/* Method Prototypes */
void updateGPS();
void updateRot();
void updateProx();
void strobe();

// set up a global struct to store our sensor data
struct SensorDataStruct {
  float gpsLat = 0.0;     //X-1's latitude (degrees)
  float gpsLng = 0.0;     //X-1's longitude (degrees)
  float roll = 0.0;       //roll angle (degrees) - may need to add offset if CodeCell isn't mounted flat
  float pitch = 0.0;      //pitch angle (degrees) - may need to add offset if CodeCell isn't mounted flat
  float yaw = 0.0;        //yaw angle (degrees) - may need to add offset if CodeCell isn't mounted flat
  uint16_t proxDist = 0;  //distance detected by proximity sensor in <UNITS>
};

// set up enum for the glider's flight state, initialize to STAGING state
enum {STAGING, FLIGHT, DESCENT} flightState = STAGING;

/* Constants and Pins */
static const uint32_t MonitorBaud = 115200;

/* Tasks */
#define GPS_TASK_RATE 250
#define ROT_TASK_RATE 250
#define PROX_TASK_RATE 250
#define STROBE_TASK_RATE 250
Task gpsTask(GPS_TASK_RATE, TASK_FOREVER, &updateGPS);      //update GPS global variables every GPS_TASK_RATE ms, will be enabled from the start
Task rotTask(ROT_TASK_RATE, TASK_FOREVER, &updateRot);      //update rotation global variables every ROT_TASK_RATE ms, will be enabled from the start
Task proxTask(PROX_TASK_RATE, TASK_FOREVER, &updateProx);   //update proximity global variables every PROX_TASK_RATE ms, will be enabled from the start
Task strobeTask(STROBE_TASK_RATE, TASK_FOREVER, &strobe);   //strobe lights every STROBE_TASK_RATE ms, won't be enabled until release

/* Declaring Components */
Scheduler runner;                 //declare our task scheduler
CodeCell myCodeCell;              //declare our CodeCell
TinyGPSPlus gps;                  //declare TinyGPSPlus object to handly GPS data
SensorDataStruct sensorData;      //declare struct to hold our sensor data

void setup() {
  Serial.begin(MonitorBaud);
  runner.init();
  Serial.println("Initialized scheduler");
  
  runner.addTask(gpsTask);
  Serial.println("added gpsTask");
  
  runner.addTask(rotTask);
  Serial.println("added rotTask");

  runner.addTask(proxTask);
  Serial.println("added proxTask");

  runner.addTask(strobeTask);
  Serial.println("added strobeTask");

  delay(2500);
  
  gpsTask.enable();
  Serial.println("Enabled gpsTask");
  rotTask.enable();
  Serial.println("Enabled rotTask");
  proxTask.enable();
  Serial.println("Enabled proxTask");

  // Initializes the CodeCell for the sensing features we will use (proximity and rotation angle)
  myCodeCell.Init(LIGHT + MOTION_ROTATION); 

  // Check that GPS is connected
  if (myI2CGPS.begin() == false)
  {
    Serial.println("Module failed to respond. Please check wiring.");
    while (1); //Freeze!
  }
  Serial.println("GPS module found!");

  delay(2000);
}

void loop() {
  runner.execute();
}

void updateGPS() {
  // get GPS data from i2C
  if (myI2CGPS.available()){
    if(gps.encode(myI2CGPS.read())){
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
  myCodeCell.Motion_RotationRead(sensorData.roll, sensorData.pitch, sensorData.yaw);
}

void updateProx() {
  // Update proximity distance global variable
  sensorData.proxDist = myCodeCell.Light_ProximityRead();
}

void strobe() {
  // TODO - method to strobe LEDs
}