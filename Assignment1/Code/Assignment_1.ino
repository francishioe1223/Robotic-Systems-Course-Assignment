#include "Motors.h"
#include "PID.h"
#include "LineSensors.h"
#include "Kinematics.h"
#include "Encoders.h"
#include "Magnetometer.h"

#define EMIT_PIN 11
#define STATE_CALIBRATE   0  
#define STATE_LEAVE_START 1
#define STATE_SEARCH_AREA 2
#define STATE_START_AREA 3
#define STATE_MAGNET_AREA 4
#define STATE_DONE 5
#define TURN_THRESHOLD 0.1
#define MOVE_THRESHOLD 1
#define MAGNET_THRESHOLD 2

Motors_c motors;    
LineSensors_c line_sensors;
Magnetometer_c mag;
Kinematics_c pose;

unsigned long TurningTime;
unsigned long SearchingTime;
unsigned long Kinematics_previousTime = 0;
unsigned long stayAreaTime = 0;  
const unsigned long stayAreaTimeDuration = 500; 
const long Kinematics_interval = 20;    
int FirstCollectTime = 2000;
int state;
float target_angle;
float magposition_x;
float magposition_y;
bool isTurning = false; 
bool Magnet_status_A;
bool Magnet_status_B;
bool Magnet_status_C;
bool find_Magnet;
bool arrive_start;
bool arrive_magnet;
bool failed;

void setup() {
  pinMode(A2, INPUT_PULLUP);
  pinMode( EMIT_PIN, OUTPUT );
  digitalWrite( EMIT_PIN, HIGH );
  Serial.begin(9600);
  mag.mysetup();
  setupEncoder0();
  setupEncoder1(); // Initialise the pose of the robot to x=0, y=0, theta=0.  
  pose.initialise( 0, 0, 0 );
  state = STATE_CALIBRATE;
}

void loop() {
  unsigned long Kinematics_currentTime = millis();
  if( Kinematics_currentTime - Kinematics_previousTime >= Kinematics_interval){
    pose.update();
    Kinematics_previousTime = Kinematics_currentTime;
  }
  if( state == STATE_CALIBRATE ) {
    doCalibration();
    if (millis() >= TurningTime) {
      state = STATE_LEAVE_START;
      setForward(4000); // drive into search area
      SearchingTime = millis() + 120000; 
    }
  } else if( state == STATE_LEAVE_START ) {
      bool status = checkTurn();
      if( status == false ) {
        state = STATE_SEARCH_AREA;
        motors.setPWM(-20, -20); 
      }
  } else if( state == STATE_SEARCH_AREA ) {
      checkSearch();
      if(find_Magnet == true){
        motors.setPWM(0, 0);  
        delay(1500);
        state = STATE_START_AREA;
      } else if( millis() > SearchingTime ) {
          pose.update(); 
          positionMAGNET(pose.x, pose.y);
          failed = true;
          state = STATE_START_AREA;
          motors.setPWM(0,0);
      }    
   } else if( state == STATE_START_AREA ){
        setStartAngle();
        if (!checkTurnAngle()) {
          return;
        } 
        moveToStart();
    } else if( state == STATE_MAGNET_AREA ){
        setMagnetAngle();
        if (!checkTurnAngle()) {
          return;
        } 
        moveToMagnet();
    } else if( state == STATE_DONE ){
        // do nothing
    }
} // End of FSM update()

void setForward(unsigned long duration_ms){
  TurningTime = millis() + duration_ms; 
  motors.setPWM(-20, -20); 
  isTurning = true; 
} // End of setForward()

void setBackward(unsigned long duration_ms){
  TurningTime = millis() + duration_ms; 
  motors.setPWM(20, 20); 
  isTurning = true; 
} // End of setBackward()

void setTurn(float left_pwm, float right_pwm, unsigned long duration_ms) {
  TurningTime = millis() + duration_ms; 
  motors.setPWM(left_pwm, right_pwm); 
  isTurning = true; 
} // End of setTurn()

bool checkTurn() {
  if (isTurning && millis() >= TurningTime) {
    motors.setPWM(0, 0);  
    isTurning = false;  
    return false;   
  }
  return true;     
} // End of checkTurn()

void ifOnline(){
  if (line_sensors.SensorOnLine[0] == true) {
     setTurn(-25, 25, random(50, 200));  // Turn right
   }
   else if (line_sensors.SensorOnLine[1] == true) {
     setTurn(-25, 25, random(150, 300));  // Turn right
   }
   else if (line_sensors.SensorOnLine[0] == true && line_sensors.SensorOnLine[1] == true) {
     setTurn(-25, 25, random(200, 280));  // Turn right
   }
   else if (line_sensors.SensorOnLine[2] == true) {
     setTurn(-25, 15, random(10, 40));  // Slight adjustment
   }
   else if (line_sensors.SensorOnLine[3] == true) {
     setTurn(25, -25, random(100, 300));  // Turn left
   }
   else if (line_sensors.SensorOnLine[1] == true && line_sensors.SensorOnLine[2] == true && line_sensors.SensorOnLine[3] == true) {
     setTurn(-25, 25, random(100, 600));  // Turn face
   }
   else if (line_sensors.SensorOnLine[4] == true) {
     setTurn(25, -25, random(50, 200));  // Turn left
   }
   else if (line_sensors.SensorOnLine[3] == true && line_sensors.SensorOnLine[4] == true) {
     setTurn(25, -25, random(200, 280));  // Turn right
   }
} // End of ifOnline()

void doCalibration(){
  setTurn(25, -25, 2050);
  line_sensors.CalibrationRoutine(2050);
  mag.CalibrationRoutine(2050);
} // End of doCalibration()

void detectLine(){
  line_sensors.calcCalibratedADC();  
  line_sensors.OnLine();
  ifOnline();
  bool turn_status = checkTurn();
  if (turn_status == false) { // Not turning, drive forward
    motors.setPWM(-20, -20);  
    } else { 
    // Is turning, do sth
  }
} // End of detectLine()

void detectMagnet(){
  mag.calcCalibratedMAG();
  if(abs(mag.calibrated[0]) > 2){
    Magnet_status_A = true;
  } else {
    Magnet_status_A = false;
    }
  if(abs(mag.calibrated[1]) > 2){
    Magnet_status_B = true;
  } else {
    Magnet_status_B = false;
    }
  if(abs(mag.calibrated[2]) > 2){
    Magnet_status_C = true;
  } else {
    Magnet_status_C = false;
    }
  if(Magnet_status_A && Magnet_status_B && Magnet_status_C == true){
    motors.setPWM(-13, -12); 
    delay(1500);
    pose.update(); 
    positionMAGNET(pose.x, pose.y);
    find_Magnet = true; 
  } else{
    find_Magnet = false; 
  }
} // End of detectMagnet()

void checkSearch(){
  stayArea();
  detectLine();
  detectMagnet();
} // End of checkSearch()

bool checkTurnAngle() {
  float angle_I = pose.theta;
  float angle_S = target_angle - angle_I;
  angle_S = atan2( sin(angle_S), cos(angle_S) );
  if( abs(angle_S) < TURN_THRESHOLD ) {
    motors.setPWM( 0, 0 ); // Stop robot
    return true;
  }
  if (angle_S > 0) {
    motors.setPWM(20, -20); // Turn left 
  } else {
    motors.setPWM(-20, 20); // Turn right
    }
  return false;
} // End of checkTurnAngle()

void setTurnAngle(float angle) {
  target_angle = angle;
} // End of setTurnAngle()

void setStartAngle(){
  float angle_H = calculateStartAngle(pose.x, pose.y);
  setTurnAngle(angle_H);
} // End of setStartAngle()

float calculateStartAngle(float x, float y){
  
  float angle_H = atan2( -y,  -x );
  if (SearchingTime > 80000){
    angle_H = atan2(-10 -y, 30 -x);
  }
  return angle_H;
} // End of calculateStartAngle()

void moveToStart(){
  float start_distance = sqrt( sq( abs(pose.x) ) + sq( abs(pose.y) ) );
  if (SearchingTime > 80000){
    start_distance = sqrt( sq( abs(pose.x -30) ) + sq( abs(pose.y +10) ) );
  }
  motors.setPWM(-13, -11);
  //Serial.println(start_distance);
  if(start_distance < MOVE_THRESHOLD){
    motors.setPWM(0, 0);  
    if (failed == true){
      state = STATE_DONE;
    } else if (failed == false){
      state = STATE_MAGNET_AREA;
    }
    arrive_start = true;
    delay(500);
  } else{
    arrive_start = false;
  } 
} // End of moveToStart()

float positionMAGNET(float x, float y){
  magposition_x = x;
  magposition_y = y;
} // End of positionMAGNET()

void setMagnetAngle(){
  float angle_M = calculateMagnetAngle(magposition_x, magposition_y);
  setTurnAngle(angle_M);
} // End of setMagnetAngle()

float calculateMagnetAngle(float x, float y){
  float angle_M = atan2(y - pose.y, x - pose.x);
  return angle_M;
} // End of calculateMagnetAngle()

void moveToMagnet(){
  float magnet_distance = sqrt( sq( abs(magposition_x - pose.x) ) + sq( abs(magposition_y - pose.y)) )  ;
  motors.setPWM(-13, -11);
  detectMagnet();
  Serial.println(magposition_x);
  Serial.println(magposition_y);
  if(magnet_distance < MAGNET_THRESHOLD){
    motors.setPWM(0, 0);  
    state = STATE_DONE;
    arrive_magnet = true;
  } else{
    arrive_magnet = false;
  } 
} // End of moveToMagnet()

void stayArea(){
  unsigned long currentMillis = millis();
  float boundary = -260;
  if(pose.y > boundary && currentMillis >= stayAreaTime && !isTurning){
    if (pose.x > 150){
    setTurn(25,-25,random(200, 350));
    } else {
    setTurn(-25,25,random(200, 350));
    }
    stayAreaTime = currentMillis + stayAreaTimeDuration;
  } 
  bool turn_status = checkTurn();
  if (!turn_status) { // Not turning, drive forward
    motors.setPWM(-20, -20);  
    } 
} // End of stayArea()
