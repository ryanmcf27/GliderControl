/*
  GPS_Runner_Test

  Code to test GPS in a runner task
  
*/

/* Libraries */
#include <TaskScheduler.h> //for more info refer to source repository: https://github.com/arkhipenko/TaskScheduler/tree/master
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

/* Method Prototypes */
void updateGPS();
void printGPSData();

// set up a global struct to store our sensor data
struct GPSDataStruct {
  float gpsLat = 0.0;     //X-1's latitude (degrees)
  float gpsLng = 0.0;     //X-1's longitude (degrees)
};

/* Constants and Pins */
static const int RXPin = 0, TXPin = 1;
static const uint32_t MONITOR_BAUD = 115200, GPS_BAUD = 9600;

/* Tasks */
#define GPS_TASK_RATE 2500
#define PRINT_TASK_RATE 500

Task gpsTask(GPS_TASK_RATE, TASK_FOREVER, &updateGPS);      //update GPS global variables every GPS_TASK_RATE ms, will be enabled from the start
Task printTask(PRINT_TASK_RATE, TASK_FOREVER, &printGPSData);

/* Declaring Components */
Scheduler runner;                     //declare our task scheduler
TinyGPSPlus gps;                      //declare TinyGPSPlus object to handle GPS data (convert raw data to be more user friendly)
SoftwareSerial ss_GPS(RXPin, TXPin);  //declare serial connection to the GPS module
GPSDataStruct gpsData;          //declare struct to hold our sensor data

void setup() {
  //setup Serial communication
  Serial.begin(MONITOR_BAUD);
  ss_GPS.begin(GPS_BAUD);

  //setup pins
  //note: RX and TX pins are set as input and output by default, don't need to explicitly set them here


  //setup runners and tasks
  runner.init();
  Serial.println("Initialized scheduler");
  
  runner.addTask(printTask);
  Serial.println("added printTask");

  delay(1000);
  
  printTask.enable();
  Serial.println("Enabled printTask");

  delay(1000);
  //runner.execute();
}

void loop() {
  runner.execute();
  updateGPS(); 
}

void updateGPS() {
  // get GPS data from i2C
  while(ss_GPS.available() > 0){
    gps.encode(ss_GPS.read());
    if(gps.location.isUpdated()){
      //if the received gps data is successfully encoded, update global variables
      gpsData.gpsLat = gps.location.lat();
      gpsData.gpsLng = gps.location.lng();
    }
  }
}

void printGPSData() {
  char buffer [50];
  sprintf(buffer, "Latitude: %f, Longitude: %f", gpsData.gpsLat, gpsData.gpsLng);
  Serial.println(buffer);
}
