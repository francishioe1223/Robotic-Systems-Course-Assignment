/***************************************
 ,        .       .           .     ,-.  
 |        |       |           |        ) 
 |    ,-: |-. ,-. |-. ,-. ,-. |-      /  
 |    | | | | `-. | | |-' |-' |      /   
 `--' `-` `-' `-' ' ' `-' `-' `-'   '--' 
****************************************/

// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _LINESENSORS_H
#define _LINESENSORS_H

// We will use all 5 line sensors (DN1 - 5)
// and so define a constant here, rather than
// type '5' in lots of places.
#define NUM_SENSORS 5

// Pin definitions
// This time, we will use an array to store the
// pin definitions.  This is a bit like a list.
// This way, we can either loop through the
// list automatically, or we can ask for a pin
// by indexing, e.g. sensor_pins[0] is A11,
// sensors_pins[1] is A0.
const int sensor_pins[ NUM_SENSORS ] = { A11, A0, A2, A3, A4 };

// This is the pin used to turn on the infra-
// red LEDs.
#define EMIT_PIN   11


// Class to operate the linesensors.
class LineSensors_c {
  
  public:

    // Store your readings into this array.
    // You can then access these readings elsewhere
    // by using the syntax line_sensors.readings[n];
    // Where n is a value [0:4]
    float readings[ NUM_SENSORS ];

    // Variables to store calibration constants. 
    // Make use of these as a part of the exercises 
    // in labsheet 2.
    float minimum[ NUM_SENSORS ];
    float maximum[ NUM_SENSORS ];
    float scaling[ NUM_SENSORS ];

    float minimumDigital[ NUM_SENSORS ];
    float maximumDigital[ NUM_SENSORS ];
    float scalingDigital[ NUM_SENSORS ];

    // Variable to store the calculated calibrated
    // (corrected) readings. Needs to be updated via
    // a function call, which is completed in 
    // labsheet 2.
    float calibrated[ NUM_SENSORS ];
    float calibratedDigital[ NUM_SENSORS ];

    bool SensorOnLine[ NUM_SENSORS ];

    unsigned long start_time[ NUM_SENSORS ];
    unsigned long end_time[ NUM_SENSORS ];
    unsigned long elapsed_time[ NUM_SENSORS ];

    // Constructor, must exist.
    LineSensor_c() {
      // leave this empty
    }

    void initialiseForADC() {
      pinMode( EMIT_PIN, OUTPUT );
      digitalWrite( EMIT_PIN, HIGH );
      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
           pinMode( sensor_pins[ sensor ], INPUT_PULLUP );
      }
    } // End of initialiseForADC()

    void readSensorsADC() {
      initialiseForADC();
      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
           readings[sensor] = analogRead( sensor_pins[ sensor ] );
      }
    } // End of readSensorsADC()

    void calcCalibratedADC() {
      readSensorsADC();
      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
           calibrated[ sensor ] = ( (readings[sensor] - minimum[ sensor ]) / scaling[sensor] );         
      }
    } // End of readSensorsADC()

    void calcCalibratedDigital() {
      readSensorsDigital();
      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
           calibratedDigital[ sensor ] = ( (elapsed_time [ sensor ] - minimumDigital[ sensor ]) / scalingDigital[sensor] );         
      }
    } // End of readSensorsADC()
  
    void initialiseForDigital() {
//      pinMode( EMIT_PIN, OUTPUT ); // teacher's example
      pinMode( EMIT_PIN, INPUT );
      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
      pinMode( sensor_pins[ sensor ], INPUT );
      delay(1500);
      }
    } // End of initialiseForDigital()

    void readSensorsDigital() {
      pinMode( EMIT_PIN, OUTPUT );
      digitalWrite( EMIT_PIN, HIGH );
      
      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
      pinMode( sensor_pins[ sensor ], OUTPUT );
      digitalWrite( sensor_pins[ sensor ], HIGH );
      }
      delayMicroseconds(10);
      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
      pinMode( sensor_pins[ sensor ], INPUT );
      }
      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
        start_time [ sensor ]= micros();
        while(digitalRead( sensor_pins[ sensor ] ) == HIGH ){
          if (elapsed_time [ sensor ] > 160){
          break;
          }
        }
      }
      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
      end_time [ sensor ] = micros();
      pinMode( sensor_pins[ sensor ], INPUT );
      elapsed_time [ sensor ] = end_time [ sensor ] - start_time [ sensor ];
      } 

//      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
//      pinMode( sensor_pins[ sensor ], OUTPUT );
//      digitalWrite( sensor_pins[ sensor ], HIGH );  
//      delayMicroseconds(10);
//      pinMode( sensor_pins[ sensor ], INPUT );
//      start_time [ sensor ]= micros();
//      while(digitalRead( sensor_pins[ sensor ] ) == HIGH ){
//        if (elapsed_time [ sensor ] > 160){
//        break;
//        }
//      }
//      end_time [ sensor ] = micros();
//      pinMode( sensor_pins[ sensor ], INPUT );
//      elapsed_time [ sensor ] = end_time [ sensor ] - start_time [ sensor ]; 
//      }

        
    } // End of readSensorsDigital()
    

    void CalibrationRoutine( int CollectTime ){
      unsigned long TurningTime = millis() + CollectTime;
      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ){
        minimum[ sensor ] = 1023;
        maximum[ sensor ] = 0;
      }
      
      while(millis() < TurningTime){
        readSensorsADC();
        for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ){   
          if( readings[sensor] > maximum[ sensor ]){
            maximum[ sensor ] = readings[sensor];
          }
          if( readings[sensor] < minimum[ sensor ]){
            minimum[ sensor ] = readings[sensor];
          }
        } 
        delay(10);       
      }  

      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ){
        scaling[sensor] = maximum[sensor] - minimum[sensor];
      }
    } // End of CalibrationRoutine( int CollectTime )

    void CalibrationRoutineDigital( int CollectTime ){
      unsigned long TurningTime = millis() + CollectTime;
      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ){
        minimumDigital[ sensor ] = 1023;
        maximumDigital[ sensor ] = 0;
      }
      
      while(millis() < TurningTime){
        readSensorsDigital();
        for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ){   
          if( elapsed_time [ sensor ] > maximumDigital[ sensor ]){
            maximumDigital[ sensor ] = elapsed_time [ sensor ];
          }
          if( elapsed_time [ sensor ] < minimumDigital[ sensor ]){
            minimumDigital[ sensor ] = elapsed_time [ sensor ];
          }
        } 
        delay(10);       
      }  

      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ){
        scalingDigital[sensor] = maximumDigital[sensor] - minimumDigital[sensor];
      }
    } // End of CalibrationRoutineDigital( int CollectTime )

     void OnLine(){
      for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
        if (calibrated[sensor] > 0.9) {
            SensorOnLine[sensor] = true;
            //Serial.println("1");
        } else {
            SensorOnLine[sensor] = false;
            //Serial.println("0");
        }
      }
     } // End of OnLine()

     
     
  
}; // End of LineSensor_c class defintion

#endif
