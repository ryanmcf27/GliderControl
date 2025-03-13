#include <SD.h>
#include <SPI.h>
#include <string.h>

const int CS = BUILTIN_SDCARD;
int count = 0;
const char* FILENAME = "simpleSD_Test1.txt";


void setup() {
  Serial.begin(115200);
  if (SD.begin(CS)) {
    Serial.println("SD card initialised");
  }
  else {
    Serial.println("SD card initialisation failed");
  }
}

void loop() {
  // open the file named FILENAME on the SD card
  File dataFile = SD.open(FILENAME, FILE_WRITE);
  // if the file is available, write the contents of datastring to it
  if (dataFile) {
    dataFile.print("Count is: ");
    dataFile.println(count);
    Serial.println(count);
    count++;
    //closing the file often flushes the data to the SD often, makes sure it actually gets saved on power down
    dataFile.close();
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening file");
  }  
  delay(1000); 
}

