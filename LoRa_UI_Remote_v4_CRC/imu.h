#include <Wire.h>
//---BNO055---
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define BNO055_SAMPLE_DELAY  100
Adafruit_BNO055 bno2 = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 bno1 = Adafruit_BNO055(55, 0x29);
Adafruit_BNO055 bno3 = Adafruit_BNO055(55, 0x1);
// ---BNO080---
#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;

void setBNO080()
{
  if (!myIMU.begin())
  {
    Serial.println("Ooops, no BNO080 detected ... Wiring Check Kar!... SDA & SCL is reversed or not connected!");
    delay(1000);
    //ESP.restart();
    
    while (1);
  }
  myIMU.enableRotationVector(50); //Send data update every 
  Serial.println("BNO080 detected!!");
}

void setBNO0551()
{
  if (!bno1.begin())
  {
    Serial.print("Ooops, no BNO055-1 detected ... Wiring Check Kar!... SDA & SCL is reversed or not connected!");
    delay(1000);
    //ESP.restart();
    //while (1);
    
  }
  Serial.println("BNO055-1 detected!!");
}

void setBNO0552()
{
  if (!bno2.begin())
  {
    Serial.print("Ooops, no BNO055-2 detected ... Wiring Check Kar!... SDA & SCL is reversed or not connected!");
    delay(1000);
    //ESP.restart();
   // while (1);
  }
  Serial.println("BNO055-2 detected!!");
}
void setBNO0553()
{
  if (!bno3.begin())
  {
    Serial.print("Ooops, no BNO055-3 detected ... Wiring Check Kar!... SDA & SCL is reversed or not connected!");
    delay(1000);
    //ESP.restart();
    //while (1);
    
  }
  Serial.println("BNO055-3 detected!!");
}
float xBNO080()
{
  if (myIMU.dataAvailable())
  {
    float roll = degrees(myIMU.getRoll()) ; 
    return (roll);
  }  
}

float yBNO080()
{
  if (myIMU.dataAvailable())
  {
    float pitch = degrees(myIMU.getPitch()); 
    return (pitch);
  }  
}

float zBNO080()
{
  if (myIMU.dataAvailable())
  {
    float yaw = degrees(myIMU.getYaw()); 
    return (yaw);
  }  
}

float yBNO0551()
{
  sensors_event_t orientationData;
  bno1.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  float y = orientationData.orientation.z;

  return (y);
}
float yBNO0552()
{
  sensors_event_t orientationData;
  bno2.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  float y = orientationData.orientation.z;

  return (y);
}
float zBNO0552()
{
  sensors_event_t orientationData;
  bno2.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  float z = orientationData.orientation.x;
  if(z>180)
  {
    z=z-360;
  }
  return (z);
}
float zBNO0553() {
  String dataString = "";
  Wire.requestFrom(0x8, 7); // Request data from Nano at address 0x8
  
  while (Wire.available()) {
    char c = Wire.read();
    dataString += c;
  }
  
  float zed = dataString.toFloat();
  return zed;
}
