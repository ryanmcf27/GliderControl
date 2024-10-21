/* Libraries */
#include <TaskScheduler.h>

/* Method Prototypes */
void updateGPS();
void updateIMU();
void updateProx();
void strobe();

/* Global Variables */
#define GPS_TASK_RATE 250
#define IMU_TASK_RATE 250
#define PROX_TASK_RATE 250
#define STROBE_TASK_RATE 250

float gpsLong = 0;
float gpsLat = 0;
float imuAccelX = 0;
float imuAccelY = 0;
float imuAccelZ = 0;
float proxDist = 0;

/* Tasks */
Task gpsTask(GPS_TASK_RATE, TASK_FOREVER, &updateGPS);      //update GPS global variables every GPS_TASK_RATE ms, will be enabled from the start
Task imuTask(IMU_TASK_RATE, TASK_FOREVER, &updateIMU);      //update IMU global variables every IMU_TASK_RATE ms, will be enabled from the start
Task proxTask(PROX_TASK_RATE, TASK_FOREVER, &updateProx);   //update proximity global variables every PROX_TASK_RATE ms, will be enabled from the start
Task strobeTask(STROBE_TASK_RATE, TASK_FOREVER, &strobe);   //strobe lights every STROBE_TASK_RATE ms, won't be enabled until release

Scheduler runner;

void setup() {
  Serial.begin(115200);

  runner.init();
  Serial.println("Initialized scheduler");
  
  runner.addTask(gpsTask);
  Serial.println("added gpsTask");
  
  runner.addTask(imuTask);
  Serial.println("added imuTask");

  runner.addTask(proxTask);
  Serial.println("added proxTask");

  runner.addTask(strobeTask);
  Serial.println("added strobeTask");

  delay(5000);
  
  gpsTask.enable();
  Serial.println("Enabled gpsTask");
  imuTask.enable();
  Serial.println("Enabled imuTask");
  proxTask.enable();
  Serial.println("Enabled proxTask");

}

void loop() {
  runner.execute();
}

void updateGPS() {
  // TODO - method to update GPS global variables
}

void updateIMU() {
  // TODO - method to update IMU global variables
}

void updateProx() {
  // TODO - method to update proximity sensor's global variables
}

void strobe() {
  // TODO - method to strobe LEDs
}