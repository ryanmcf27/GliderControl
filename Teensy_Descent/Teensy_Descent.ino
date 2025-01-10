/*
  Teensy_Descent

  Custom code for Iowa State University's 2024-2025 Design Build Fly (DBF) team's X-1 glider
  
*/

/* Libraries */
#include <TaskScheduler.h> //for more info refer to source repository: https://github.com/arkhipenko/TaskScheduler/tree/master
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "src/MPU6050/MPU6050.h"
#include <PWMServo.h> //must use this library because basic Servo.h Arduino library cannot run on Teensy 4.1's processor

/* Method Prototypes */
void updateGPS();
void updateIMU();
void strobe();
void moveRudder(int angle);
void calculate_IMU_error();

// set up a global struct to store our sensor data
struct SensorDataStruct {
  float gpsLat = 0.0;     //X-1's latitude (degrees)
  float gpsLng = 0.0;     //X-1's longitude (degrees)
  float roll = 0.0;       //roll angle (degrees) - may need to add offset if IMU isn't mounted flat
  float pitch = 0.0;      //pitch angle (degrees) - may need to add offset if IMU isn't mounted flat
  float yaw = 0.0;        //yaw angle (degrees) - may need to add offset if IMU isn't mounted flat
  float accelX = 0.0;     //X-1's acceleration in the X direction
  float accelY = 0.0;     //X-1's acceleration in the Y direction
  float accelZ = 0.0;     //X-1's acceleration in the Z direction
};

// global variables to store the coordinates of our target location
//float targetLat = 32.265617;
//float targetLng = -111.273524;

// global flag to tell whether (true) or not (false) the limit switch has been triggered 
bool limitTriggeredFlag = true;
unsigned long limitTriggeredTime = 0;

// global variable to store the yaw (direction our glider is facing) when released
float releasedYaw = 0.0;

// set up enum for the glider's flight state, initialize to PRERELEASE state
enum {PRERELEASE, DESCENT_OUT, DESCENT_TURNAROUND, DESCENT_RETURN, DESCENT_CIRCLE, ERROR_STATE} flightState = PRERELEASE;

/* Constants and Pins */
static const int RXPin = 0, TXPin = 1, ServoPin = 2, LEDPin = 6, LimitPin = 8;
static const uint32_t MONITOR_BAUD = 115200, GPS_BAUD = 9600;
static const int RUDDER_RANGE_DEGREES = 90; //Limits the servo's movement from (90 - RUDDER_RANGE_DEGREES) to (90 + RUDDER_RANGE_DEGREES), corrects if invalid input is given to moveRudder function
static const int LIMIT_TRIGGERED_BUFFER_MILLIS = 1000;
static const float ACCEL_SCALE_FACTOR = 1.0;    //full scale accelerometer range (16384.0 for 2 G's) - reference dRehmFlight, using 1 for now to see raw IMU output
static const float GYRO_SCALE_FACTOR = 1.0;     //full scale accelerometer range (131.0 for 250 degrees/sec) - reference dRehmFlight, using 1 for now to see raw IMU output
//conversion factor to help us find the equation of the lines that we'd like the X-1 to turn around at and to start circular descent 
static const float DEGREES_PER_FOOT_FROM_START_LINE = 1.109787E-5;     //NOTE this isn't how longitude and latitude work precisely, but will be fine on this small bit of the global scale
static const int TURNAROUND_FEET_FROM_START_LINE = 200;
//IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()
float AccErrorX = 0.0;
float AccErrorY = 0.0;
float AccErrorZ = 0.0;
float GyroErrorX = 0.0;
float GyroErrorY= 0.0;
float GyroErrorZ = 0.0;

/* Tasks */
#define GPS_TASK_RATE 250
#define IMU_TASK_RATE 250
#define STROBE_TASK_RATE 250
Task gpsTask(GPS_TASK_RATE, TASK_FOREVER, &updateGPS);      //update GPS global variables every GPS_TASK_RATE ms, will be enabled from the start
Task imuTask(IMU_TASK_RATE, TASK_FOREVER, &updateIMU);      //update IMU global variables every IMU_TASK_RATE ms, will be enabled from the start
Task strobeTask(STROBE_TASK_RATE, TASK_FOREVER, &strobe);   //strobe lights every STROBE_TASK_RATE ms, won't be enabled until release

/* Declaring Components */
Scheduler runner;                     //declare our task scheduler
MPU6050 mpu6050;                      //declare MPU6050 object for our IMU
TinyGPSPlus gps;                      //declare TinyGPSPlus object to handle GPS data (convert raw data to be more user friendly)
SoftwareSerial ss_GPS(RXPin, TXPin);  //declare serial connection to the GPS module
SensorDataStruct sensorData;          //declare struct to hold our sensor data
PWMServo rudderServo;                 //declare servo to control rudder

void setup() {
  //setup Serial communication
  Serial.begin(MONITOR_BAUD);
  ss_GPS.begin(GPS_BAUD);

  //setup pins
  //note: RX and TX pins are set as input and output by default, don't need to explicitly set them here
  pinMode(LEDPin, OUTPUT);
  pinMode(LimitPin, INPUT); //might need to switch to INPUT_PULLUP for actual limit switch implementation, this is for testing with simple button
  pinMode(ServoPin, OUTPUT);

  //setup servo
  rudderServo.attach(ServoPin); //attach pin to servo object
  moveRudder(90);               //set servo to 90 degrees

  //setup runners and tasks
  runner.init();
  Serial.println("Initialized scheduler");
  
  runner.addTask(gpsTask);
  Serial.println("added gpsTask");
  
  runner.addTask(imuTask);
  Serial.println("added imuTask");

  runner.addTask(strobeTask);
  Serial.println("added strobeTask");

  delay(1000);
  
  gpsTask.enable();
  Serial.println("Enabled gpsTask");
  imuTask.enable();
  Serial.println("Enabled imuTask");

  delay(1000);

  //calculate_IMU_error();
  //delay(1000);
}

void loop() {
  runner.execute();
  switch(flightState){
    case PRERELEASE:
      //checks to see if X-1 has been released based on limit switch
      if(digitalRead(LimitPin) == HIGH){
        if(!limitTriggeredFlag){
          limitTriggeredFlag = true;
          limitTriggeredTime = millis();
        } else {
          unsigned long currentMillis = millis();
          if((currentMillis - limitTriggeredTime) >= LIMIT_TRIGGERED_BUFFER_MILLIS){
            //start strobing LEDs by enabling strobe task
            strobeTask.enable();
            //set release yaw
            releasedYaw = sensorData.yaw;
            //update flightState
            flightState = DESCENT_OUT;
          }
        }
      } else {
        limitTriggeredFlag = false;
      }
      break;
    case DESCENT_OUT:
      //fly straight until reaching turn line - could add more control to straighten flight path if not on right path, for now just leave rudder at 90 deg
      moveRudder(90);
      if(sensorData.gpsLat >= (3.326530*sensorData.gpsLng + 402.420296 + DEGREES_PER_FOOT_FROM_START_LINE*TURNAROUND_FEET_FROM_START_LINE)){
        //turn line has been passed, go to DESCENT_TURNAROUND
        flightState = DESCENT_TURNAROUND;
      }
      break;
    case DESCENT_TURNAROUND:
      //turn around 180 degrees, using releaseYaw as reference
      moveRudder(50); //TODO: replace with constant rudder angle for this turn
      if(abs(sensorData.yaw - releasedYaw) >= 180){
        //glider has completed 180 degree turn, go to DESCENT_RETURN
        flightState = DESCENT_RETURN;
      }
      break;
    case DESCENT_RETURN:
      //fly straight until reaching finish line
      moveRudder(90);
      if(sensorData.gpsLat <= (3.326530*sensorData.gpsLng + 402.420296)){
        //start line has been passed, go to DESCENT_CIRCLE
        flightState = DESCENT_CIRCLE;
      }
      break;
    case DESCENT_CIRCLE:
      //set rudder to circle in scoring zone until landing
      moveRudder(50); //TODO replace with constant rudder angle for our circular descent
      break;
    case ERROR_STATE:
      Serial.println("In ERROR flight state!!!");
      break;
  }
}

void updateGPS() {
  Serial.println("updateGPS");
  //uncomment the lines below when GPS is connected
  // // get GPS data from i2C
  // if (ss_GPS.available()){
  //   gps.encode(ss_GPS.read());
  //   if(gps.location.isUpdated()){
  //     //if the received gps data is successfully encoded, update global variables
  //     sensorData.gpsLat = gps.location.lat();
  //     sensorData.gpsLng = gps.location.lng();
      
  //     //for testing
  //     char buffer [50];
  //     sprintf(buffer, "Latitude: %f, Longitude: %f", sensorData.gpsLat, sensorData.gpsLng);
  //     Serial.println(buffer);
  //   }
  // }
}

void updateIMU() {
  Serial.println("updateIMU");
  //uncomment the lines below once IMU is connected
  // // Get new raw IMU values
  // int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;
  // mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

  // // Update IMU global variables with corrected and scaled values
  // sensorData.accelX = (AcX / ACCEL_SCALE_FACTOR) - AccErrorX;
  // sensorData.accelY = (AcY / ACCEL_SCALE_FACTOR) - AccErrorY;
  // sensorData.accelZ = (AcZ / ACCEL_SCALE_FACTOR) - AccErrorZ;
  // sensorData.roll = (GyX / GYRO_SCALE_FACTOR) - GyroErrorX;
  // sensorData.pitch = (GyY / GYRO_SCALE_FACTOR) - GyroErrorY;
  // sensorData.yaw = (GyZ / GYRO_SCALE_FACTOR) - GyroErrorZ;
}

void strobe() {
  Serial.println("Strobe");

  // Stobe LEDs by setting them to HIGH if they were LOW, and LOW if they were HIGH
  if(digitalRead(LEDPin) == LOW){
    digitalWrite(LEDPin, HIGH);
  } else {
    digitalWrite(LEDPin, LOW);
  }
}

void moveRudder(int angle){
  Serial.print("Moving rudder to ");
  Serial.print(angle);
  Serial.println(" degrees.");

  //correct angle if it is an invalid value
  int rudderLowerBound = 90 - (RUDDER_RANGE_DEGREES / 2);
  int rudderUpperBound = 90 + (RUDDER_RANGE_DEGREES / 2);
  if(angle < rudderLowerBound) {
    angle = rudderLowerBound;
  } else if (angle > rudderUpperBound) {
    angle = rudderUpperBound;
  }

  //set servo position
  rudderServo.write(angle);
}

void calculate_IMU_error() {
  //DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
  //Credit: dRehmFlight - https://github.com/nickrehm/dRehmFlight/blob/master/Versions/dRehmFlight_Teensy_BETA_1.3/dRehmFlight_Teensy_BETA_1.3.ino
  /*
   * Don't worry too much about what this is doing. The error values it computes are applied to the raw gyro and 
   * accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata(). This eliminates drift in the
   * measurement. 
   */
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;  //temp vars for getting raw values from the IMU
  float AccX, AccY, AccZ, GyroX, GyroY, GyroZ; //temp vars for scaling IMU values
  
  AccErrorX = 0.0;
  AccErrorY = 0.0;
  AccErrorZ = 0.0;
  GyroErrorX = 0.0;
  GyroErrorY= 0.0;
  GyroErrorZ = 0.0;
  
  //Read IMU values 12000 times
  int c = 0;
  while (c < 12000) {
    mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
    
    AccX  = AcX / ACCEL_SCALE_FACTOR;
    AccY  = AcY / ACCEL_SCALE_FACTOR;
    AccZ  = AcZ / ACCEL_SCALE_FACTOR;
    GyroX = GyX / GYRO_SCALE_FACTOR;
    GyroY = GyY / GYRO_SCALE_FACTOR;
    GyroZ = GyZ / GYRO_SCALE_FACTOR;
    
    //Sum all readings
    AccErrorX  = AccErrorX + AccX;
    AccErrorY  = AccErrorY + AccY;
    AccErrorZ  = AccErrorZ + AccZ;
    GyroErrorX = GyroErrorX + GyroX;
    GyroErrorY = GyroErrorY + GyroY;
    GyroErrorZ = GyroErrorZ + GyroZ;
    c++;
  }
  //Divide the sum by 12000 to get the error value
  AccErrorX  = AccErrorX / c;
  AccErrorY  = AccErrorY / c;
  AccErrorZ  = AccErrorZ / c - 1.0;
  GyroErrorX = GyroErrorX / c;
  GyroErrorY = GyroErrorY / c;
  GyroErrorZ = GyroErrorZ / c;

  Serial.print("float AccErrorX = ");
  Serial.print(AccErrorX);
  Serial.println(";");
  Serial.print("float AccErrorY = ");
  Serial.print(AccErrorY);
  Serial.println(";");
  Serial.print("float AccErrorZ = ");
  Serial.print(AccErrorZ);
  Serial.println(";");
  
  Serial.print("float GyroErrorX = ");
  Serial.print(GyroErrorX);
  Serial.println(";");
  Serial.print("float GyroErrorY = ");
  Serial.print(GyroErrorY);
  Serial.println(";");
  Serial.print("float GyroErrorZ = ");
  Serial.print(GyroErrorZ);
  Serial.println(";");

  Serial.println("Paste these values in user specified variables section and comment out calculate_IMU_error() in void setup.");
}