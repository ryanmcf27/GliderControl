/*
  AutonomousDescent_ALPHA_1.0

  Custom code for Iowa State University's 2024-2025 Design Build Fly (DBF) team's X-1 glider
  
*/

/* Libraries */
#include <TaskScheduler.h> //for more info refer to source repository: https://github.com/arkhipenko/TaskScheduler/tree/master
#include <CodeCell.h>
CodeCell myCodeCell;  //declare our CodeCell

/* Method Prototypes */
void updateGPS();
void updateRot();
void updateProx();
void strobe();

// set up a global struct to store our sensor data
struct sensorData {
  float gpsLong = 0.0;
  float gpsLat = 0.0;
  float roll = 0.0;       //roll angle (degrees)
  float pitch = 0.0;      //pitch angle (degrees)
  float yaw = 0.0;        //yaw angle (degrees)
  uint16_t proxDist = 0;  //distance detected by proximity sensor in <UNITS>
}

// set up enum for the glider's flight state, initialize to STAGING state
enum {STAGING, FLIGHT, DESCENT} flightState = STAGING;

/* Tasks */
#define GPS_TASK_RATE 250
#define ROT_TASK_RATE 250
#define PROX_TASK_RATE 250
#define STROBE_TASK_RATE 250
Task gpsTask(GPS_TASK_RATE, TASK_FOREVER, &updateGPS);      //update GPS global variables every GPS_TASK_RATE ms, will be enabled from the start
Task rotTask(ROT_TASK_RATE, TASK_FOREVER, &updateRot);      //update rotation global variables every ROT_TASK_RATE ms, will be enabled from the start
Task proxTask(PROX_TASK_RATE, TASK_FOREVER, &updateProx);   //update proximity global variables every PROX_TASK_RATE ms, will be enabled from the start
Task strobeTask(STROBE_TASK_RATE, TASK_FOREVER, &strobe);   //strobe lights every STROBE_TASK_RATE ms, won't be enabled until release

Scheduler runner;

void setup() {
  Serial.begin(115200);

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

  myCodeCell.Init(LIGHT + MOTION_ROTATION); // Initializes the CodeCell for the sensing features we will use (proximity and rotation angle)
  
  delay(2000);
}

void loop() {
  runner.execute();
}

void updateGPS() {
  // TODO - method to update GPS global variables
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