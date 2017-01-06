/* Arduino nano v3 fligthcontroller
  
 Based on Joop Brokking's Arduino Uno flight controller project : http://www.brokking.net/ymfc-3d_main.html
 So many thanks to him.
  
 RC Inputs on Ditigal pins 8,9,10,11,12,13 (Six channels from RC)
 Channel 1 : Roll
 Channel 2 : Pitch (Nose up or down)
 Channel 3 : Thrust
 Channel 4 : Yaw
 Channel 5 : Switch A
 Channel 6 : Switch B
 
 ESC Outputs on digital pins 4,5,6,7
 Motor/ESC 1 : Pin 4 (Should rotate CCW)
 Motor/ESC 2 : Pin 5 (Should rotate CW)
 Motor/ESC 3 : Pin 6 (Should rotate CCW)
 Motor/ESC 4 : Pin 7 (Should rotate CW)

Next test:
----------
- ESC - Thrust / Speedcontroll : Basic test completed use one prop and it wanted to take off. ;)

Missing parts:
--------------
- PID Calculations
- Voltage drop compensation

Changes to implement:
---------------------
- Use only one micros() in RC Interupt > use "current_time = micros();" and compare with that instead of micros() for each RC channel to make code more efficient.
- Use register to set interupt instead of userfriendly programming to make code more efficient.
- Reconfigure to get only Gyro readings from MPU for quicker reading
*/

#include <Wire.h>

///////////////////////
//  PID Gain values  //
///////////////////////

float pid_p_gain_x = 1.3;     //Gain setting for the roll P-controller (1.3)
float pid_i_gain_x = 0.05;    //Gain setting for the roll I-controller (0.3)
float pid_d_gain_x = 15;      //Gain setting for the roll D-controller (15)
float pid_p_gain_y = 1.3;     //Gain setting for the roll P-controller (1.3)
float pid_i_gain_y = 0.05;    //Gain setting for the roll I-controller (0.3)
float pid_d_gain_y = 15;      //Gain setting for the roll D-controller (15)
float pid_p_gain_z = 4.0;     //Gain setting for the pitch P-controller. //4.0  
float pid_i_gain_z = 0.04;    //Gain setting for the pitch I-controller. //4.0
float pid_d_gain_z = 0.0;     //Gain setting for the pitch D-controller.

// PID Variables
float pid_i_mem_x;
float pid_i_mem_y;
float pid_i_mem_z;
int pid_max_x = 400;
int pid_max_y = 400;
int pid_max_z = 400;
int pid_diff;
int pid_last_diff_x;
int pid_last_diff_y;
int pid_last_diff_z;
float GyroXInput,GyroYInput,GyroZInput;                         // Gyro input to PID calculation
float rc_channel_1_input,rc_channel_2_input,rc_channel_4_input; // RC input to PID calculation
float pid_output_x,pid_last_x_error;                            // PID Roll output
float pid_output_y,pid_last_y_error;                            // PID Pitch output  
float pid_output_z,pid_last_z_error;                            // PID Yaw output

/////////////////////////////////////////////////////////////////////////////////////////
// Define RC input pins
#define CH1_PIN 8
#define CH2_PIN 9
#define CH3_PIN 10
#define CH4_PIN 11
#define CH5_PIN 12
#define CH6_PIN 13
#define MPU_address 0x68

// Define global virables
int i; // Counter
int last_channel_1 = 0; // Virable for status of RC channel 1
int last_channel_2 = 0; // Virable for status of RC channel 2
int last_channel_3 = 0; // Virable for status of RC channel 3
int last_channel_4 = 0; // Virable for status of RC channel 4
int last_channel_5 = 0; // Virable for status of RC channel 5
int last_channel_6 = 0; // Virable for status of RC channel 6
// unsigned long loopTimer = 0; // Timer to check loop cycle time
// unsigned long rcInteruptTimer = 0; // Timer to check rcInterupt cycle time
unsigned long engineTimer_1 = 1000; // Timer for runtime of engine 1
unsigned long engineTimer_2 = 1000; // Timer for runtime of engine 2
unsigned long engineTimer_3 = 1000; // Timer for runtime of engine 3
unsigned long engineTimer_4 = 1000; // Timer for runtime of engine 4
unsigned long timer_channel_1 = 0; // Timer for checkin impulse time for RC channel 1
unsigned long timer_channel_2 = 0; // Timer for checkin impulse time for RC channel 2
unsigned long timer_channel_3 = 0; // Timer for checkin impulse time for RC channel 3
unsigned long timer_channel_4 = 0; // Timer for checkin impulse time for RC channel 4
unsigned long timer_channel_5 = 0; // Timer for checkin impulse time for RC channel 5
unsigned long timer_channel_6 = 0; // Timer for checkin impulse time for RC channel 6
unsigned long timer_channel_1_lastValue = 1500; // Virable for last impulse timer RC channel 1
unsigned long timer_channel_2_lastValue = 1500; // Virable for last impulse timer RC channel 2
unsigned long timer_channel_3_lastValue = 1500; // Virable for last impulse timer RC channel 3
unsigned long timer_channel_4_lastValue = 1500; // Virable for last impulse timer RC channel 4
unsigned long timer_channel_5_lastValue = 1500; // Virable for last impulse timer RC channel 5
unsigned long timer_channel_6_lastValue = 1500; // Virable for last impulse timer RC channel 6
unsigned long zero_timer = 0; // Timer for ESC loop control
unsigned long esc_loop_timer = 0; // Timer for ESC loop control
unsigned long current_time = micros(); // current time in RC interupt function
bool enginesArmed = false; // Safty to set motor runtimes to 0 if not armed
int AcX,AcY,AcZ,Tmp; // Gyro output params
double GyX,GyY,GyZ,GyX_offset,GyY_offset,GyZ_offset; // Gyro offset callibrators

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  RC Pins                                                                                                                 //
//  interrupt setup                                                                                                         //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Interupt routine                                                                                                        //
//  for RC controlls                                                                                                        //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read register instead of highlevel digitalRead which is much slower
// Reding register is more than 5 times faster than high level commands like digital read.
ISR (PCINT0_vect){ // handle pin change interrupt for D8 to D13 here
//     rcInteruptTimer = micros();
     current_time = micros(); // Use current_time below to avoid multiple micros() reading

     // RC-channel 1
//     if(last_channel_1 == 0 && (digitalRead(CH1_PIN)) == HIGH){ // Digital read to slow
     if(last_channel_1 == 0&& PINB & B00000001){ // Register read pin 8
       last_channel_1 = 1;
       timer_channel_1 = micros();
     }
//     if(last_channel_1 == 1 && (digitalRead(CH1_PIN)) == LOW){ // Digital read to slow
     if(last_channel_1 == 1 && !(PINB & B00000001)){ // Register read pin 8
       last_channel_1 = 0;
       timer_channel_1_lastValue = micros() - timer_channel_1;
     }

      // RC-channel 2
//     if(last_channel_2 == 0 && (digitalRead(CH2_PIN)) == HIGH){ // Digital read to slow
     if(last_channel_2 == 0&& PINB & B00000010){ // Register read pin 9
       last_channel_2 = 1;
       timer_channel_2 = micros();
     }
//     if(last_channel_2 == 1 && (digitalRead(CH2_PIN)) == LOW){ // Digital read to slow
     if(last_channel_2 == 1&& !(PINB & B00000010)){ // Register read pin 9
      last_channel_2 = 0;
       timer_channel_2_lastValue = micros() - timer_channel_2;
     }

      // RC-channel 3
//     if(last_channel_3 == 0 && (digitalRead(CH3_PIN)) == HIGH){ // Digital read to slow
     if(last_channel_3 == 0&& PINB & B00000100){ // Register read pin 10
       last_channel_3 = 1;
       timer_channel_3 = micros();
     }
//     if(last_channel_3 == 1 && (digitalRead(CH3_PIN)) == LOW){ // Digital read to slow
     if(last_channel_3 == 1 && !(PINB & B00000100)){ // Register read pin 10
       last_channel_3 = 0;
       timer_channel_3_lastValue = micros() - timer_channel_3;
     }

      // RC-channel 4
//     if(last_channel_4 == 0 && (digitalRead(CH4_PIN)) == HIGH){ // Digital read to slow
     if(last_channel_4 == 0&& PINB & B00001000){ // Register read pin 11
       last_channel_4 = 1;
       timer_channel_4 = micros();
     }
//     if(last_channel_4 == 1 && (digitalRead(CH4_PIN)) == LOW){ // Digital read to slow
     if(last_channel_4 == 1&& !(PINB & B00001000)){ // Register read pin 11
       last_channel_4 = 0;
       timer_channel_4_lastValue = micros() - timer_channel_4;
     }

      // RC-channel 5
//     if(last_channel_5 == 0 && (digitalRead(CH5_PIN)) == HIGH){ // Digital read to slow
     if(last_channel_5 == 0&& PINB & B00010000){ // Register read pin 12
       last_channel_5 = 1;
       timer_channel_5 = micros();
     }
//     if(last_channel_5 == 1 && (digitalRead(CH5_PIN)) == LOW){ // Digital read to slow
     if(last_channel_5 == 1&& !(PINB & B00010000)){ // Register read pin 12
       last_channel_5 = 0;
       timer_channel_5_lastValue = micros() - timer_channel_5;
     }

      // RC-channel 6
//     if(last_channel_6 == 0 && (digitalRead(CH6_PIN)) == HIGH){ // Digital read to slow
     if(last_channel_6 == 0&& PINB & B00100000){ // Register read pin 13
       last_channel_6 = 1;
       timer_channel_6 = micros();
     }
//     if(last_channel_6 == 1 && (digitalRead(CH6_PIN)) == LOW){ // Digital read to slow
     if(last_channel_6 == 1&& !(PINB & B00100000)){ // Register read pin 13
       last_channel_6 = 0;
       timer_channel_6_lastValue = micros() - timer_channel_6;
     }

//     Serial.println(micros()-rcInteruptTimer); // Calculate runtime for RC interupt
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Read gyro                                                                                                             //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void readMPU(){
  Wire.beginTransmission(MPU_address);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_address,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  GyY = GyY * -1;
  GyZ = GyZ * -1;
  
//  Serial.print("AcX = "); Serial.print(AcX);
//  Serial.print(" | AcY = "); Serial.print(AcY);
//  Serial.print(" | AcZ = "); Serial.print(AcZ);
//  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
//  Serial.print(" | GyX = "); Serial.print(GyX);
//  Serial.print(" | GyY = "); Serial.print(GyY);
//  Serial.print(" | GyZ = "); Serial.println(GyZ);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  PID Calculations                                                                                                      //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculatePID(){
  //Roll calculations 
  pid_diff = GyroXInput - rc_channel_1_input;                                                               // Difference between gyro and rc
  //          +=                                                                                            // from original code
  pid_i_mem_x = pid_i_gain_x * pid_diff;                                                                    // Integral mem  =  integral mem + ( integral gain factor * (diff gyro -rc) )  
  if (pid_i_mem_x > pid_max_x) pid_i_mem_x = pid_max_x;                                                     // If integral roll compentsation > max then set max
  else if (pid_i_mem_x < (pid_max_x * -1)) pid_i_mem_x = pid_max_x * -1;                                     // If integral roll comp.  < - max then set -max
  pid_output_x = (pid_p_gain_x * pid_diff)+ pid_i_mem_x +  (pid_d_gain_x * ( pid_diff - pid_last_diff_x )); // Calculate roll compensation output 
  if(pid_output_x > pid_max_x)pid_output_x = pid_max_x;                                                     // Check if exceeding max values
     else if(pid_output_x < pid_max_x * -1)pid_output_x = pid_max_x * -1;
  pid_last_diff_x = pid_diff;                                                                               // Remember last diff                                                                    

  //Pitch calculations 
  pid_diff = GyroYInput - rc_channel_2_input;                                                               // Difference between gyro and rc
  //          +=                                                                                            // from original code
  pid_i_mem_y  =   pid_i_gain_y * pid_diff;                                                                 // Integral mem  =  integral mem + ( integral gain factor * (diff gyro -rc) )  
  if (pid_i_mem_y > pid_max_y) pid_i_mem_y = pid_max_y;                                                     // If integral roll compentsation > max then set max
  else if (pid_i_mem_y < (pid_max_y * -1)) pid_i_mem_y = pid_max_y * -1;                                     // If integral roll comp.  < - max then set -max
  pid_output_y = (pid_p_gain_y * pid_diff)+ pid_i_mem_y +  (pid_d_gain_y * ( pid_diff - pid_last_diff_y )); // Calculate roll compensation output 
  if(pid_output_y > pid_max_y)pid_output_y = pid_max_y;                                                     // Check if exceeding max values
     else if(pid_output_y < pid_max_y * -1)pid_output_y = pid_max_y * -1;
  pid_last_diff_y = pid_diff;                                                                               // Remember last diff                                                                    

  //Yaw calculations 
  pid_diff = GyroZInput - rc_channel_4_input;                                                                // Difference between gyro and rc
  //          +=                                                                                             // from original code
  pid_i_mem_z  =   pid_i_gain_z * pid_diff;                                                                  // Integral mem  =  integral mem + ( integral gain factor * (diff gyro -rc) )  
  if (pid_i_mem_z > pid_max_z) pid_i_mem_z = pid_max_z;                                                      // If integral roll compentsation > max then set max
  else if (pid_i_mem_z < (pid_max_z * -1)) pid_i_mem_z = pid_max_z * -1;                                      // If integral roll comp.  < - max then set -max
  pid_output_z = (pid_p_gain_z * pid_diff)+ pid_i_mem_z +  (pid_d_gain_z * ( pid_diff - pid_last_diff_z ));  // Calculate roll compensation output 
  if(pid_output_z > pid_max_z)pid_output_z = pid_max_z;                                                      // Check if exceeding max values
     else if(pid_output_z < pid_max_z * -1)pid_output_z = pid_max_z * -1;
  pid_last_diff_z = pid_diff;                                                                                // Remember last diff                                                                    

/*
  Serial.print("Roll output: ");
  Serial.print(pid_output_x);
  Serial.print(" Pitch output: ");
  Serial.print(pid_output_y);
  Serial.print(" Yaw output: ");
  Serial.println(pid_output_z);
*/
}

/////////////////////
//  Setup process  //
/////////////////////
void setup() { // Run Once
// Serial output enabling
  Serial.begin(230400);
  Serial.println("");
  Serial.println("Void Setup Begins");

// Define pins as input withOUT Internal PullUp on
  pinMode(CH1_PIN, INPUT);
  digitalWrite(CH1_PIN, LOW);
  pinMode(CH2_PIN, INPUT);
  digitalWrite(CH2_PIN, LOW);
  pinMode(CH3_PIN, INPUT);
  digitalWrite(CH3_PIN, LOW);
  pinMode(CH4_PIN, INPUT);
  digitalWrite(CH4_PIN, LOW);
  pinMode(CH5_PIN, INPUT);
  digitalWrite(CH5_PIN, LOW);
  pinMode(CH6_PIN, INPUT);
  digitalWrite(CH6_PIN, LOW);

// enable interrupt for RC-pins...
  Serial.println("Enable Interups for RC-pins");
  pciSetup(CH1_PIN);
  pciSetup(CH2_PIN);
  pciSetup(CH3_PIN);
  pciSetup(CH4_PIN);
  pciSetup(CH5_PIN);
  pciSetup(CH6_PIN);

  Serial.println("Set Digital pins 4-7 as output for ESC");
  DDRD = B11110000;  // sets Arduino pins 4 to 7 as outputs, pin 0 as input (PIN Register D are digital pins 0-7)

// Init MPU(Gyro/Accm.)
  Serial.println("Init MCU");
  Wire.begin();
  Wire.beginTransmission(MPU_address);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

// Calculate Gyro offset
  Serial.println("Calculate Gyro Offset");
  GyX_offset = 0;
  GyY_offset = 0;
  GyZ_offset = 0;
  i = 0;
  while (i < 2000) {
    readMPU();
    GyX_offset = GyX_offset + GyX;
    GyY_offset = GyY_offset + GyY;
    GyZ_offset = GyZ_offset + GyZ;
/*
  Serial.print("GyX: ");
  Serial.print(GyX);
  Serial.print( "GyY: ");
  Serial.print(GyY);
  Serial.print("GyZ: ");
  Serial.println(GyZ);
*/
    i++;
  }
  GyX_offset = GyX_offset / 2000;
  GyY_offset = GyY_offset / 2000;
  GyZ_offset = GyZ_offset / 2000;
  Serial.print("GyX_offset: ");
  Serial.print(GyX_offset);
  Serial.print(" GyY_offset: ");
  Serial.print(GyY_offset);
  Serial.print(" GyZ_offset: ");
  Serial.println(GyZ_offset);
  Serial.println("Gyro offset calculated");
  
  Serial.println("Void Setup Ends");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Main Loop                                                                                                            //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() { // Loop
// loopTimer = micros(); // Calculate loop runtime

// Arm Engines by putting the two sticks to bottom left corner
    if (!enginesArmed & 
       (timer_channel_3_lastValue < 1200 &&
        timer_channel_4_lastValue < 1250 &&
        timer_channel_1_lastValue < 1200 &&
        timer_channel_2_lastValue < 1200)) {
          enginesArmed = true;
          Serial.println("Engines Armed");
        }

// Disarm Engines by putting the two sticks to bottom right corner
    if (enginesArmed &
       (timer_channel_3_lastValue < 1200 &&
        timer_channel_4_lastValue > 1800 &&
        timer_channel_1_lastValue > 1800 &&
        timer_channel_2_lastValue < 1200)) {
          enginesArmed = false;
          Serial.println("Engines Disarmed");
        }

// Get Gyro readings
  readMPU();
  GyY = GyY - GyY_offset;
  GyroXInput = (GyroXInput * 0.8) + ((GyX / 57.14286) * 0.2);
  GyX = GyX - GyX_offset;
  GyroYInput = (GyroYInput * 0.8) + ((GyY / 57.14286) * 0.2);
  GyZ = GyZ - GyZ_offset;
  GyroZInput = (GyroZInput * 0.8) + ((GyZ / 57.14286) * 0.2);
/*
  Serial.print("GyX: ");
  Serial.print(GyX);
  Serial.print(" GyY: ");
  Serial.print(GyY);
  Serial.print(" GyZ: ");
  Serial.println(GyZ);
*/

/*
  Serial.print("GyroXInput: ");
  Serial.print(GyroXInput);
  Serial.print(" GyroYInput: ");
  Serial.print(GyroYInput);
  Serial.print(" GyroZInput: ");
  Serial.println(GyroZInput);
*/  

/*
    Serial.print("CH1: ");
    Serial.print(timer_channel_1_lastValue);
    Serial.print(" CH2: ");
    Serial.print(timer_channel_2_lastValue);
    Serial.print(" CH3: ");
    Serial.print(timer_channel_3_lastValue);
    Serial.print(" CH4: ");
    Serial.print(timer_channel_4_lastValue);
    Serial.print(" CH5: ");
    Serial.print(timer_channel_5_lastValue);
    Serial.print(" CH6: ");
    Serial.println(timer_channel_6_lastValue);
*/

// RC Channel input to PID calculations
  rc_channel_1_input = 0.0;
  if(timer_channel_1_lastValue > 1508)rc_channel_1_input = (timer_channel_1_lastValue - 1508)/3.0;
//  else if(timer_channel_1_lastValue < 1492)rc_channel_1_input = (timer_channel_1_lastValue - 1492)/3.0;
  else if(timer_channel_1_lastValue < 1492)rc_channel_1_input = ((1492 - timer_channel_1_lastValue)/3.0) * -1;
  
  rc_channel_2_input = 0.0;
  if(timer_channel_2_lastValue > 1508)rc_channel_2_input = (timer_channel_2_lastValue - 1508)/3.0;
//  else if(timer_channel_2_lastValue < 1492)rc_channel_2_input = (timer_channel_2_lastValue - 1492)/3.0;
  else if(timer_channel_2_lastValue < 1492)rc_channel_2_input = ((1492 - timer_channel_2_lastValue)/3.0) * -1;
  
  rc_channel_4_input = 0.0;
  if(timer_channel_4_lastValue > 1508)rc_channel_4_input = (timer_channel_4_lastValue - 1508)/3.0;
//  else if(timer_channel_4_lastValue < 1492)rc_channel_4_input = (timer_channel_4_lastValue - 1492)/2.0;
  else if(timer_channel_4_lastValue < 1492)rc_channel_4_input = ((1492 - timer_channel_4_lastValue)/2.0) * -1;

/*
  Serial.print("Roll input to PID: ");
  Serial.print(rc_channel_1_input);
  Serial.print("Pitch input to PID: ");
  Serial.print(rc_channel_2_input);
  Serial.print("Yaw input to PID: ");
  Serial.println(rc_channel_4_input);
*/

// Pid calclations
  calculatePID();

// Controll ESC connected to digital pins 4,5,6,7
   while(zero_timer + 4000 > micros()); // wait until 4000 microseconds since last loop for a 250 Hz ESC update

// If engines are armed set controllpins to high
    if (enginesArmed){
      PORTD |= B11110000; // Sets all ports 4-7 high ((OR) |= shift with 0 keeps current |= shift with 1 sets 1)
    } else {
      PORTD &= B00001111; // Sets all ports 4-7 low ((AND) &= shift with 0 set 0, &= shift with 1 keeps current value)
    }

    zero_timer = micros(); // reset timer for next loop delay
    
 // Set engine shut down timers

//    engineTimer_1 = timer_channel_3_lastValue - pid_output_y + pid_output_x - pid_output_z; //Calculate the pulse for esc 1 (front-right - CCW)
//    engineTimer_2 = timer_channel_3_lastValue + pid_output_y + pid_output_x + pid_output_z; //Calculate the pulse for esc 2 (rear-right - CW)
//    engineTimer_3 = timer_channel_3_lastValue + pid_output_y - pid_output_x - pid_output_z; //Calculate the pulse for esc 3 (rear-left - CCW)
//    engineTimer_4 = timer_channel_3_lastValue - pid_output_y - pid_output_x + pid_output_z; //Calculate the pulse for esc 4 (front-left - CW)
    engineTimer_1 = timer_channel_3_lastValue + pid_output_y + pid_output_x - pid_output_z; //Calculate the pulse for esc 1 (front-right - CCW)
    engineTimer_2 = timer_channel_3_lastValue - pid_output_y + pid_output_x + pid_output_z; //Calculate the pulse for esc 2 (rear-right - CW)
    engineTimer_3 = timer_channel_3_lastValue - pid_output_y - pid_output_x - pid_output_z; //Calculate the pulse for esc 3 (rear-left - CCW)
    engineTimer_4 = timer_channel_3_lastValue + pid_output_y - pid_output_x + pid_output_z; //Calculate the pulse for esc 4 (front-left - CW)


// No PID gain calc
//    engineTimer_1 = engineTimer_3 * 1.03;
//    engineTimer_2 = engineTimer_3 * 1.02;
//    engineTimer_3 = engineTimer_3 * 1.03;
//    engineTimer_4 = engineTimer_4 * 1.09;

    if(engineTimer_1 > 2000)engineTimer_1 = 2000; //Limit the esc-1 pulse to 2000us.
    if(engineTimer_2 > 2000)engineTimer_2 = 2000; //Limit the esc-2 pulse to 2000us.
    if(engineTimer_3 > 2000)engineTimer_3 = 2000; //Limit the esc-3 pulse to 2000us.
    if(engineTimer_4 > 2000)engineTimer_4 = 2000; //Limit the esc-4 pulse to 2000us.  

    if(engineTimer_1 < 1100)engineTimer_1 = 1100; //Limit the esc-1 pulse to 2000us.
    if(engineTimer_2 < 1100)engineTimer_2 = 1100; //Limit the esc-2 pulse to 2000us.
    if(engineTimer_3 < 1100)engineTimer_3 = 1100; //Limit the esc-3 pulse to 2000us.
    if(engineTimer_4 < 1100)engineTimer_4 = 1100; //Limit the esc-4 pulse to 2000us.  

/*
    Serial.print("ecs 1: ");
    Serial.print(engineTimer_1);
    Serial.print(" ecs 2: ");
    Serial.print(engineTimer_2);
    Serial.print(" ecs 3: ");
    Serial.print(engineTimer_3);
    Serial.print(" ecs 4: ");
    Serial.println(engineTimer_4);
*/
   
    engineTimer_1 = engineTimer_1 + zero_timer; // Old values before PID.
    engineTimer_2 = engineTimer_2 + zero_timer;
    engineTimer_3 = engineTimer_3 + zero_timer;
    engineTimer_4 = engineTimer_4 + zero_timer;
    
    while(PORTD >= 16) {
      esc_loop_timer = micros();
      if (engineTimer_1 <= esc_loop_timer) PORTD &= B11101111; // Shift in 0 on esc 1 (Digital pin 4)
      if (engineTimer_2 <= esc_loop_timer) PORTD &= B11011111; // Shift in 0 on esc 2 (Digital pin 5)
      if (engineTimer_3 <= esc_loop_timer) PORTD &= B10111111; // Shift in 0 on esc 3 (Digital pin 6)
      if (engineTimer_4 <= esc_loop_timer) PORTD &= B01111111; // Shift in 0 on esc 4 (Digital pin 7)
    }

//    Serial.print(" LoopTimer: ");
//    Serial.println(micros() - loopTimer);

}
