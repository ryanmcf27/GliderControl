#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

/* Method Prototypes */
void updateGPS();

static const int RXPin = 0, TXPin = 1;
int i = 0;

TinyGPSPlus gps;                      //declare TinyGPSPlus object to handle GPS data (convert raw data to be more user friendly)
SoftwareSerial ss_GPS(RXPin, TXPin);  //declare serial connection to the GPS module 

void setup() {
  Serial.begin(115200);
  ss_GPS.begin(9600);
  Serial.println("Setup complete.");
}

void loop() {
  while(ss_GPS.available() > 0){
    gps.encode(ss_GPS.read());
    if(gps.location.isUpdated()){
      /* uncomment one of the sections below to read from monitor or to graph in plotter */

      // //monitor version
      // Serial.print("Lat: ");
      // Serial.println(gps.location.lat(), 6);
      // Serial.print("Lng: ");
      // Serial.println(gps.location.lng(), 6);
      // Serial.print("Speed (kmph): ");
      // Serial.println(gps.speed.kmph(), 6);
      // Serial.print("Altitude (m): ");
      // Serial.println(gps.altitude.meters(), 6);
      // Serial.println();

      //plotter version
      Serial.print(gps.location.lat(), 6);
      Serial.print("  ");
      Serial.print(gps.location.lng(), 6);
      Serial.print("  ");
      Serial.print(gps.speed.kmph(), 6);
      Serial.print("  ");
      Serial.println(gps.altitude.meters(), 6);
      //Serial.println(gps.location.lat() + " " + gps.location.lat() + " " + gps.speed.kmph() + " " + gps.altitude.meters());

    }
  }
}
