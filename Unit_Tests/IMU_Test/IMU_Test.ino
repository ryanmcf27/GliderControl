#include "src/MPU6050/MPU6050.h"

/* Method Prototypes */
void updateIMU();
void calculate_IMU_error();

MPU6050 mpu6050;                      //declare MPU6050 object for our IMU

//IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()
float AccErrorX = 0.0;
float AccErrorY = 0.0;
float AccErrorZ = 0.0;
float GyroErrorX = 0.0;
float GyroErrorY= 0.0;
float GyroErrorZ = 0.0;

static const float ACCEL_SCALE_FACTOR = 1.0;    //full scale accelerometer range (16384.0 for 2 G's) - reference dRehmFlight, using 1 for now to see raw IMU output
static const float GYRO_SCALE_FACTOR = 1.0;     //full scale accelerometer range (131.0 for 250 degrees/sec) - reference dRehmFlight, using 1 for now to see raw IMU output

void setup() {
  Serial.begin(115200);

  //can comment out if irrelevant to test
  calculate_IMU_error();
  delay(1000);
}

void loop() {
  updateIMU();
  delay(1000);
}

void updateIMU() {
  Serial.println("Updating IMU...");
  // Get new raw IMU values
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;
  mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

  // print corrected and scaled values
  Serial.print("AccelX: ");
  Serial.println((AcX / ACCEL_SCALE_FACTOR) - AccErrorX);
  Serial.print("AccelY: ");
  Serial.println((AcY / ACCEL_SCALE_FACTOR) - AccErrorY);
  Serial.print("AccelZ: ");
  Serial.println((AcZ / ACCEL_SCALE_FACTOR) - AccErrorZ);
  Serial.print("GyroX/roll: ");
  Serial.println((GyX / GYRO_SCALE_FACTOR) - GyroErrorX);
  Serial.print("GyroY/pitch: ");
  Serial.println((GyY / GYRO_SCALE_FACTOR) - GyroErrorY);
  Serial.print("GyroX/yaw: ");
  Serial.println((GyZ / GYRO_SCALE_FACTOR) - GyroErrorZ);
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