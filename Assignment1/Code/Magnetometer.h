/***************************************
 ,        .       .           .     ,--, 
 |        |       |           |       /  
 |    ,-: |-. ,-. |-. ,-. ,-. |-     `.  
 |    | | | | `-. | | |-' |-' |        ) 
 `--' `-` `-' `-' ' ' `-' `-' `-'   `-'  
***************************************/

// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _MAGNETOMETER_H
#define _MAGNETOMETER_H

#include <Wire.h>
#include <LIS3MDL.h>

#define MAX_AXIS 3

class Magnetometer_c {

  public:

    // Instance of the LIS3MDL class used to
    // interact with the magnetometer device.
    LIS3MDL mag;

    // A place to store the latest readings 
    // from the magnetometer
    float readings[ MAX_AXIS ];
    float maximum[ MAX_AXIS ];
    float minimum[ MAX_AXIS ];
    float range[ MAX_AXIS ];
    float offset[ MAX_AXIS ];
    float scaling[ MAX_AXIS ];
    float calibrated[ MAX_AXIS ];

    // Constructor, must exist.
    Magnetometer_c () {
      // Leave this empty.
      // If you put Wire.begin() into this function
      // it will crash your microcontroller.
    }

    void enableDefault(){
      mag.enableDefault();
    }
    
    // Call this function witin your setup() function
    // to initialise the I2C protocol and the
    // magnetometer sensor
    bool initialise() {

      // Start the I2C protocol
      Wire.begin();

      // Try to connect to the magnetometer
      if ( !mag.init() ) {
        return false;
      } else {
        return true;
      }
    } // End of initialise()

    void mysetup(){
      Wire.begin();
//      Serial.println("***RESET***");
      delay(1000);
      if (!mag.init() ) {  // no..? :(
        while(1) {
        Serial.println("Failed to detect and initialize magnetometer!");
        delay(1000);
        }
      }
      mag.enableDefault();
    }

    // Function to update readings array with
    // latest values from the sensor over i2c
    void getReadings() {
      mag.read();
      readings[0] = mag.m.x;
      readings[1] = mag.m.y;
      readings[2] = mag.m.z;
    } // End of getReadings()

    void CalibrationRoutine( int CollectTime ){
      unsigned long TurningTime = millis() + CollectTime;
      for ( int sensor = 0; sensor < MAX_AXIS; sensor ++){
        maximum[ sensor ] = -9999.9;
        minimum[ sensor ] = 9999.9;
      }
      while (millis() < TurningTime){
        getReadings();
        for ( int sensor = 0; sensor < MAX_AXIS; sensor ++){
          if( readings[sensor] > maximum[sensor]){
              maximum[sensor] = readings[sensor];
          }
          if( readings[ sensor ] < minimum[ sensor ]){
              minimum[ sensor ] = readings[ sensor ];
          }
        }
        delay(10);
      }
      for( int sensor = 0; sensor < MAX_AXIS; sensor++ ){
        range[sensor] = maximum[sensor] - minimum[sensor];
        offset[sensor] = minimum[sensor] + (range[sensor] / 2);
        scaling[sensor] = 1.0 / (range[sensor] / 2.0);
        }
      }
    

    void calcCalibratedMAG(){
      getReadings();
      for (int sensor = 0; sensor < MAX_AXIS; sensor ++){
        calibrated[sensor] = (readings[sensor] - offset[sensor]) * scaling[sensor];
      }
    }
  
}; // End of Magnetometer_c class definition

#endif
