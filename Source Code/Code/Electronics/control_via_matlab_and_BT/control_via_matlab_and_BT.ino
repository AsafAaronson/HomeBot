#include <VarSpeedServo.h>
#include <NewPing.h>
#include <Wire.h>
//#include <SoftwareSerial.h>

//lib+ pins
VarSpeedServo servo1;
VarSpeedServo servo2;
VarSpeedServo servo3;
VarSpeedServo servo4;
VarSpeedServo servo5;
VarSpeedServo servo6;
VarSpeedServo servo7;

int servo1_pin=2;
int servo2_pin=3;
int servo3_pin=4;
int servo4_pin=5;
int servo5_pin=6;
int servo6_pin=7;
int servo7_pin=8;

//bluetooth pins
//int RX=10;
//int TX=11;
//
//SoftwareSerial BTSerial(TX, RX);   // RX and TX acorrding to the pins on the module! (RX pin to 11, TX pin to 12)
// initial position- important because of the lib VarSpeed
int servo1_initial_pos=90;
int servo2_initial_pos=90;
int servo3_initial_pos=90;
int servo4_initial_pos=90;
int servo5_initial_pos=90;
int servo6_initial_pos=90;
int servo7_initial_pos=90;

//smooth parameters 
float prev_smooth_servo1 = servo1_initial_pos;
float prev_smooth_servo2 = servo2_initial_pos;
float prev_smooth_servo3 = servo3_initial_pos;
float prev_smooth_servo4 = servo4_initial_pos;
float prev_smooth_servo5 = servo5_initial_pos;
float prev_smooth_servo6 = servo6_initial_pos;

float curr_smooth_servo1 = servo1_initial_pos;
float curr_smooth_servo2 = servo2_initial_pos;
float curr_smooth_servo3 = servo3_initial_pos;
float curr_smooth_servo4 = servo4_initial_pos;
float curr_smooth_servo5 = servo5_initial_pos;
float curr_smooth_servo6 = servo6_initial_pos;

float req_pos_smooth_servo1 = servo1_initial_pos;
float req_pos_smooth_servo2 = servo2_initial_pos;
float req_pos_smooth_servo3 = servo3_initial_pos;
float req_pos_smooth_servo4 = servo4_initial_pos;
float req_pos_smooth_servo5 = servo5_initial_pos;
float req_pos_smooth_servo6 = servo6_initial_pos;

//costants
int motor;
int input_pos=90;
String message;
char char_array[2];
int fast_var_speed=10;
int slow_var_speed=10;
  String outMsg;
  int incomingByte;
  byte byteCounter = 0;
  byte words[64];
  int motor2_correction=11;
  
//Smoothed Servos Paramters
float ratio = 0.04;
int loop_delay = 10;

//walk good parameter: fast_var_speed=40;slow_var_speed=10;ratio = 0.04;loop_delay = 1;

// Sonar Variables
#define TRIGGER_PIN  9
#define ECHO_PIN     10
#define MAX_DISTANCE 30 // [cm] Maximum distance to measure
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

// Gyro Variables
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;


void setup() {
  Serial.begin(9600); //communication rate
  Serial.setTimeout(10);  //collecting delay
 // BTSerial.begin(9600);  
 // BTSerial.setTimeout(10);  //collecting delay
  servo1.attach(servo1_pin); 
  servo2.attach(servo2_pin); 
  servo3.attach(servo3_pin); 
  servo4.attach(servo4_pin); 
  servo5.attach(servo5_pin); 
  servo6.attach(servo6_pin);
  servo7.attach(servo7_pin);
  
  // Gyro Setup
  Wire.begin();                 // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0);                     // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);
 // calculate_IMU_error();
  delay(20);
}

void loop() {
  if (Serial.available()>0){
    message= Serial.readStringUntil('\n');
    //message= Serial.read();
    message.toCharArray(char_array, 2);
    motor=(String(message[0])+String(message[1])).toInt();
    input_pos=(String(message[2])+String(message[3])+String(message[4])).toInt();
///////////////////////////////////////////Single motor command///////////////////////////////////////
    if (motor==1){
      req_pos_smooth_servo1=input_pos;
//      servo1.write(input_pos,slow_var_speed,false);
//      prev_smooth_servo1=input_pos;
//      req_pos_smooth_servo1=input_pos;
    }
    else if (motor==2){
      req_pos_smooth_servo2=input_pos+motor2_correction;
//      servo2.write(input_pos,slow_var_speed,false);
//      prev_smooth_servo2=input_pos;
//      req_pos_smooth_servo2=input_pos;
    }
    else if (motor==3){
      req_pos_smooth_servo3=input_pos;
//      servo3.write(input_pos,slow_var_speed,false);
//      prev_smooth_servo3=input_pos;
//      req_pos_smooth_servo3=input_pos;
    }
    else if (motor==4){
      req_pos_smooth_servo4=input_pos;
//      servo4.write(input_pos,slow_var_speed,false);
//      prev_smooth_servo4=input_pos;
//      req_pos_smooth_servo4=input_pos;
    }
    else if (motor==5){
      req_pos_smooth_servo5=input_pos;
//      servo5.write(input_pos,slow_var_speed,false);
//      prev_smooth_servo5=input_pos;
//      req_pos_smooth_servo5=input_pos;
    }
    else if (motor==6){
      req_pos_smooth_servo6=input_pos;
//      servo6.write(input_pos,slow_var_speed,false);
//      prev_smooth_servo6=input_pos;
//      req_pos_smooth_servo6=input_pos;
    }
    else if (motor==70){  //Gripper
      servo7.write(input_pos);
    }
    else if (motor==0){
      servo1.write(prev_smooth_servo1+input_pos,slow_var_speed,false);
      servo3.write(prev_smooth_servo3+input_pos,slow_var_speed,false);
      servo4.write(prev_smooth_servo4+input_pos,slow_var_speed,false);
      servo6.write(prev_smooth_servo6+input_pos,slow_var_speed,false);
    }

    // Read Distance from Sonar
    else if (motor==61){
      delay(50);  // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
      int distance = sonar.ping_cm(); // Send ping, get distance in cm and print result (0 = outside set distance range)
      //Serial.print("Distance: ");
      Serial.println(distance);
     // Serial.println("cm");
    }
    // Read from Gyro
    else if (motor==62){
      //Serial.print("Orienteation [Deg]: ");
      Serial.println(yaw);
    }
        else if (motor==51){
       ratio = 0.4;
       fast_var_speed=20;
       slow_var_speed=20;
      Serial.println("step speed");
      }
     else if (motor==52){
         ratio = 0.4;
         fast_var_speed=10;
         slow_var_speed=10;
      Serial.println("turn speed");
      }
     else if (motor==53){
        ratio = 0.4;
        fast_var_speed=5;
        slow_var_speed=5;
      Serial.println("climb speed");
      }
  }

  //smoothing the motor movement
  curr_smooth_servo1 =(ratio * req_pos_smooth_servo1)+((1-ratio)*prev_smooth_servo1);
  prev_smooth_servo1 = curr_smooth_servo1;
  servo1.write(curr_smooth_servo1,fast_var_speed,false);

  curr_smooth_servo2 =(ratio * req_pos_smooth_servo2)+((1-ratio)*prev_smooth_servo2);
  prev_smooth_servo2 = curr_smooth_servo2;
  servo2.write(curr_smooth_servo2,fast_var_speed,false);

  curr_smooth_servo3 =(ratio * req_pos_smooth_servo3)+((1-ratio)*prev_smooth_servo3);
  prev_smooth_servo3 = curr_smooth_servo3;
  servo3.write(curr_smooth_servo3,fast_var_speed,false);
  
  curr_smooth_servo4 =(ratio * req_pos_smooth_servo4)+((1-ratio)*prev_smooth_servo4);
  prev_smooth_servo4 = curr_smooth_servo4;
  servo4.write(curr_smooth_servo4,fast_var_speed,false);

  curr_smooth_servo5 =(ratio * req_pos_smooth_servo5)+((1-ratio)*prev_smooth_servo5);
  prev_smooth_servo5 = curr_smooth_servo5;
  servo5.write(curr_smooth_servo5,fast_var_speed,false);
  
  curr_smooth_servo6 =(ratio * req_pos_smooth_servo6)+((1-ratio)*prev_smooth_servo6);
  prev_smooth_servo6 = curr_smooth_servo6;
  servo6.write(curr_smooth_servo6,fast_var_speed,false);
  
  delay(loop_delay);

  //Gyro calculations
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) -0.31; // AccErrorX ~(-0.31) Using calculate_IMU_error()custom function
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) -3.99; // AccErrorY ~(-3.99)
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX -0.56;// GyroErrorX ~(-0.56)
  GyroY = GyroY + 2; // GyroErrorY ~(2)
  GyroZ = GyroZ - 0.5; // GyroErrorZ ~ (-0.8)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
}
          
void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
//  Serial.print("AccErrorX: ");
//  Serial.println(AccErrorX);
//  Serial.print("AccErrorY: ");
//  Serial.println(AccErrorY);
//  Serial.print("GyroErrorX: ");
//  Serial.println(GyroErrorX);
//  Serial.print("GyroErrorY: ");
//  Serial.println(GyroErrorY);
 // Serial.print("GyroErrorZ: ");
//  Serial.println(GyroErrorZ);
}
