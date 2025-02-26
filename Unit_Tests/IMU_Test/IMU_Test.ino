#include "src/MPU6050/MPU6050.h"
#include <Wire.h>

/* Method Prototypes */
void updateIMU();
void IMUinit();
void calculate_IMU_error();

MPU6050 mpu6050;                      //declare MPU6050 object for our IMU

//IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()
float AccErrorX = -0.08;
float AccErrorY = 0.01;
float AccErrorZ = -0.02;
float GyroErrorX = -1.52;
float GyroErrorY= 0.86;
float GyroErrorZ = 0.79;

static const float ACCEL_SCALE_FACTOR = 16384.0;    //full scale accelerometer range (16384.0 for 2 G's) - reference dRehmFlight, using 1 for now to see raw IMU output
static const float GYRO_SCALE_FACTOR = 131.0;     //full scale accelerometer range (131.0 for 250 degrees/sec) - reference dRehmFlight, using 1 for now to see raw IMU output

float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float B_accel = 0.14;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
float B_gyro = 0.1;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)

void setup() {
  Serial.begin(115200);
  Serial.println("Setup has begun.");

  //initialize IMU
  IMUinit();

  //uncomment to get calibration values
  //calculate_IMU_error();

  delay(1000);

  Serial.println("Setup Finished.");
}

void loop() {
  updateIMU();
  delay(1000);
}

void IMUinit(){
  //initialize the IMU
  Wire.begin();
  Wire.setClock(200000); //400 kHz max...

  mpu6050.initialize();

  if (mpu6050.testConnection() == false) {
    Serial.println("MPU6050 initialization unsuccessful");
    Serial.println("Check MPU6050 wiring or try cycling power");
    while(1) {}
  }

  //research the details of these ranges (taken from dRehmFlight)
  mpu6050.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu6050.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

}

void updateIMU() {
  Serial.println("Updating IMU...");
  // Get new raw IMU values
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;
  mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

   //Accelerometer
  AccX = AcX / ACCEL_SCALE_FACTOR; //G's
  AccY = AcY / ACCEL_SCALE_FACTOR;
  AccZ = AcZ / ACCEL_SCALE_FACTOR;
  //Correct the outputs with the calculated error values
  AccX = AccX - AccErrorX;
  AccY = AccY - AccErrorY;
  AccZ = AccZ - AccErrorZ;
  //LP filter accelerometer data
  // AccX = (1.0 - B_accel)*AccX_prev + B_accel*AccX;
  // AccY = (1.0 - B_accel)*AccY_prev + B_accel*AccY;
  // AccZ = (1.0 - B_accel)*AccZ_prev + B_accel*AccZ;
  AccX_prev = AccX;
  AccY_prev = AccY;
  AccZ_prev = AccZ;

  //Gyro
  GyroX = GyX / GYRO_SCALE_FACTOR; //deg/sec
  GyroY = GyY / GYRO_SCALE_FACTOR;
  GyroZ = GyZ / GYRO_SCALE_FACTOR;
  //Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX;
  GyroY = GyroY - GyroErrorY;
  GyroZ = GyroZ - GyroErrorZ;
  //LP filter gyro data
  // GyroX = (1.0 - B_gyro)*GyroX_prev + B_gyro*GyroX;
  // GyroY = (1.0 - B_gyro)*GyroY_prev + B_gyro*GyroY;
  // GyroZ = (1.0 - B_gyro)*GyroZ_prev + B_gyro*GyroZ;
  GyroX_prev = GyroX;
  GyroY_prev = GyroY;
  GyroZ_prev = GyroZ;

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
  Serial.print("GyroZ/yaw: ");
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