//This program is for the orinial "lorica opimus" flight controller, the one with arduino nano, mpu6050 and XM reciever inside transparent shrinkwrap.
//Green led is "radioled", red on nano is "boardled" and SMD led on PCB is "gyroled"

#define RADIOTIMEOUT 50           //After this many ms consider radio dead, and do whatever is possible to recover

#define TODEG 57.29577951          //Conversion factors for degrees & radians
#define TORAD 0.017453293         //The AVR float cannot reach this level of precision but w/e

const int NORMALIZEDSERVO = 10000;  //Normalized servo commands, 0 is center, -+10000 limits

const long GYROMAXRANGE = 20000;  //In tenths of deg/s, THIS MUST BE SAME AS GYRO SENSITIVITY SETTING
const int ACCELMAXG = 4;        //In G. THIS MUST BE SAME AS ACCEL SENSITIVITY SETTING

const long RATENORM = 1000;  //Steering max rate deg/s in tenths (1 decimals). This is where joystick input is scaled tob Default switch pos

long ratescale = (32767*RATENORM)/GYROMAXRANGE; //Value at which gyro output corresponds to selected maximum rate in degrees
const float MAXANGLE = 60.0;  //Maximum tilt angle in anglemode, one decimal.

boolean acroMode = false; //If true, hovering is in acro (rate) mode. Else, it's in angle (horizon) mode

#define UNINSECTIFYING  //IF this is defined, serial will not use the easytransfer packets but normal printing.
//#define RAWOUTPUT       //This causes debug print to print raw output suitable for serial monitor, else human readable

#define radioLed 5     //Active low
#define gyroLed 8     //Active low
#define boardLed 13  //Active high

#include "floatPID.h"
#include <Servo.h>
#include <avr/wdt.h>  //Watchdog
#include "I2Cdev.h" //
#include "MPU6050_6Axis_MotionApps20.h"
#include <SBUS.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;
SBUS sbus(Serial);
Servo ch1;
Servo ch2;
Servo ch3;
Servo ch4;
Servo ch5;
Servo ch6;
Servo ch7;

//--Steering inputs--  
float pitch;  //These are fly-by-wire targets to the inner loop pid sclaled to so that maximum angular velocity = max rate.
float roll;   //They are updated either from radio (acro mode) or from angle pid (angle mode)
float yaw;

int pitchCommand = 1024;  //These are Commands from the radio, in acro mode they go directly to inner loop, else to the outer loop
int rollCommand = 1024;   //Unit is "frsky sbus value" so 1024 is center
int yawCommand = 1025;
int throttleCommand = 256;

float pitchTarget = 0;  //These are pitch and roll angle targets for horizon mode pid
float rollTarget = 0;   //They are updated from pitch and roll commands

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful (It can still return garbage data)
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
bool freshOrientData = false;  //This flag is set when new DMP data arrives, causing orientation PIDs to be run

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int gx, gy, gz;
float gxf, gyf, gzf;
//VectorInt16 aa;         // [x, y, z]            accel sensor measurements

//========MPU INTERRUPT ROUTINE USING PIN CHANGE INTERRUPTS=========
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
ISR(PCINT2_vect) {
  if (digitalRead(7)) mpuInterrupt = true;
  //digitalWrite(8,HIGH);
}

//LORICA STUFF
boolean radioActive = false;
int gyroMode = 2000; //Disabled by default
byte debugByte = 0;
boolean mpuOverflowDetect = false;

//MISCELLANEOUS
unsigned int loopRate = 0;
int testvar = 0;
int testvar2 = 0;
int testvar3 = 0;


//--------INNER LOOP PID COEFFICIENTS-------------
const float p = 4.0;  //Good for  ms looptime 0.041
const float i = 0.00;//018; //0.00018
const float d = 0.0;//3;  //0.01

//----------OUTERLOOP PID COEFFICIENTS-------------
const float kp = 2000.0;  //Good for  ms looptime
const float ki = 0.0;
const float kd = 0.0; 

floatPID pitchPID(&gxf, &pitch, 13.0, 0.0, 0.0, -(float)NORMALIZEDSERVO,(float)NORMALIZEDSERVO);  //SETPOINT is angular rotatio rate, either from radio (acromode) or angle pid (horizon mode)
floatPID rollPID(&gyf, &roll, 2.2, 0.0, 0.0, -(float)NORMALIZEDSERVO,(float)NORMALIZEDSERVO);   //INPUT is gyro
floatPID yawPID(&gzf, &yaw, 10.0, 0.0, 0.0, -(float)NORMALIZEDSERVO,(float)NORMALIZEDSERVO);   //OUTPUT is stabliziation term for motors and servos in pitch, roll and yaw

floatPID pitchAnglePID(&ypr[2], &pitchTarget, 0.0, 0.0, 0.0, -ratescale,ratescale);  //SETPOINT is wanted angle, from radio (stick input interpreted as this)
floatPID rollAnglePID(&ypr[1], &rollTarget, 2000.0, 0.0, 0.0, -ratescale,ratescale);   //INPUT is angular orientation data from MPU sensor fusion
                                                                                      // OUTPUT is rate command for inner loop PID

void setup() {
  wdt_disable();
  wdt_enable(WDTO_1S); //This is here since setup can fail

  Serial.begin(100000); //Sbus begin does this too

  //Serial.println("Hi!");
  pinMode(boardLed, OUTPUT);
  pinMode(radioLed, OUTPUT);
  pinMode(gyroLed, OUTPUT);

  //MPU6050
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  //Serial.println("Oi!!");
  delay(50);
  wdt_reset();
  mpu.initialize();
  delay(50);
  digitalWrite(gyroLed, !mpu.testConnection());  //Led goes on if connection is fine
  //Serial.println("Ello!");
  delay(50);
  devStatus = mpu.dmpInitialize();
  wdt_reset();
  //Serial.println("Hola!");
  //  Serial.println(F("Initializing DMP..."));
  //  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(67);  //These are hallowed for loricagyro
  mpu.setYGyroOffset(10);
  mpu.setZGyroOffset(40);
  mpu.setXAccelOffset(-3781);
  mpu.setYAccelOffset(-252);
  mpu.setZAccelOffset(2187);

  //mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  //mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);               //Digital low pass filter
  //  #define MPU6050_DLPF_BW_256         0x00
  //  #define MPU6050_DLPF_BW_188         0x01
  //  #define MPU6050_DLPF_BW_98          0x02
  //  #define MPU6050_DLPF_BW_42          0x03
  //  #define MPU6050_DLPF_BW_20          0x04
  //  #define MPU6050_DLPF_BW_10          0x05
  //  #define MPU6050_DLPF_BW_5           0x06
  //  mpu.setRate(4);  //Gyro output rate with low pass filter is this 1 kHz / (1+this), so 1 kHz here


  if (devStatus == 0) { //DMP ready, proceed
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
    while (1) { //Get stuck and blink, should get watchdoge'd soon
      digitalWrite(boardLed, HIGH);
      digitalWrite(gyroLed, LOW);
      digitalWrite(radioLed, LOW);
      delay(500);
      digitalWrite(boardLed, LOW);
      digitalWrite(gyroLed, HIGH);
      digitalWrite(radioLed, HIGH);
      delay(500);
    }
  }
  ch1.attach(10);
  ch2.attach(9);
  ch3.attach(15);
  ch4.attach(14);
  ch5.attach(2);
  ch6.attach(3);
  ch7.attach(4);
  sbus.begin(false);

  // The pin (PB0), setting up pin change interrupts
  PCMSK2 = (1 << PCINT23);
  // The port (PORTB)
  PCICR = (1 << PCIE2);
  
  wdt_reset();
  wdt_enable(WDTO_120MS); //Tighter watchdog for actual loop
  //Serial.println("Oorah!");
}

void loop() {

  //Serial.println("asdasd");
  while (!mpuInterrupt && fifoCount < packetSize) { //This is the actual "main" loop, rest is mpu
    //==========MAIN=========

    wdt_reset(); //Feed the watchdog, reset whole thing if the control functions are not being reached since that would mean a crash landing
  #ifdef UNINSECTIFYING  //If in debug mode, print out stuff
    DebugPrinter(30); //Prints out mpustuff for debugging, does not interfere with radio since it and this are one way only, and the opposite.
  #endif
    RadioHandler();
    LedHandler();
    
    //==========MAIN=========
  }
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) { //FIFO overflow
    mpu.resetFIFO();
    mpuOverflowDetect = true;
    //Serial.println("OVERFLOW");
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    //Actual DMP updates, should be 100 hz (gyro updated elsewhere faster
    mpu.getRotation(&gx, &gy, &gz);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //ypr[2] *=-1.0; //Flip the sign on this because it and roll gyro increase in opposite directions, and there are pointers to both. 
    freshOrientData = true;
    gxf = (float)gx; gyf = (float)gy; gzf = (float)gz;
    //mpu.dmpGetAccel(&aa, fifoBuffer);
  }

}

void LoopRateCounter(){
  static unsigned long loopRateCheckTime = 0;
  static unsigned int loopCounter = 0;
  unsigned long curTime = millis();
  if(curTime - loopRateCheckTime >= 1000){
    loopRate = loopCounter;
    loopCounter = 0;
    loopRateCheckTime = curTime;
  }
  else loopCounter ++;
}

void RadioHandler() {
  
  static bool outputOn = true;

  static int pitchStab;  //The different stabilization terms
  static int rollStab;   //They are different for vertical and horizontal flight
  static int yawStab;    //Static because PIDs are not run always
  int out;
  int leftMotorWant;
  int rightMotorWant;

  int leftMot = -10000;
  int rightMot = -10000;
  int leftAileron = 0;
  int rightAileron = 0;
  int rudder = 0;
  int elevator = 0;

  unsigned long curTime = millis();
  static unsigned long gyroPidTime = 0;

  sbus.process(curTime);  //This feeds the timestamp for the SBUS library, this is a hack to make it work without "millis"
  radioActive = (curTime - sbus.getLastTime()) < RADIOTIMEOUT;
  
  if (radioActive) {
    if (!outputOn) { //If servos are not on, reattach them
      ch1.attach(10); //Motor
      ch2.attach(9);  //Motor
      ch3.attach(15); //Aileron
      ch4.attach(14); //Aileron
      ch5.attach(2);  //Elevator
      ch6.attach(3);  //Rudder
      ch7.attach(4);  //Vectorservo
      outputOn = true;
    }

    ///===============CALCULATING STABILIZATION=====================

    boolean verticalMode = sbus.getChannel(7) < 1160;  //Flight mode, vertical hovering or conventional flight. Comes from wing position
    
    if(freshOrientData){  //Looprate 100 hz in lockstep with DMP updates
      freshOrientData = false;
      LoopRateCounter();  //Checks program looprate

      //==Get input from radio==
      throttleCommand = constrain(map(sbus.getChannel(1), 256, 1792, -10000, 10000),-10000,10000);
      rollCommand = -constrain(map(sbus.getChannel(2), 256, 1792, -10000, 10000),-10000,10000);  //Roll command flipped
      pitchCommand = constrain(map(sbus.getChannel(3), 256, 1792, -10000, 10000),-10000,10000);
      yawCommand = -constrain(map(sbus.getChannel(4), 256, 1792, -10000, 10000),-10000,10000);

      //===Create inner loop pitch roll yaw inputs (gyro rate) depending on mode==
      if(acroMode){
        pitch = (float)map(pitchCommand, -10000, 10000, -ratescale, ratescale); //Map stick data from radio to angular rotaion rates
        roll = (float)map(rollCommand, -10000, 10000, -ratescale, ratescale);  //This is for "acro mode"
        yaw = (float)map(yawCommand, -10000, 10000, -ratescale, ratescale);
      }
      else{
        pitchTarget = (((float)map(pitchCommand, -10000, 10000, -MAXANGLE*100, MAXANGLE*100))/100.0)*TORAD; //Map stick data from radio to angular position
        rollTarget = -(((float)map(rollCommand, -10000, 10000, -MAXANGLE*100, MAXANGLE*100))/100.0)*TORAD;  //multiply/divide by 100 to keep numerical accuracy since map works on ints
        pitch = pitchAnglePID.Compute();  //This is inner loop PID
        roll = rollAnglePID.Compute();
        yaw = (float)map(yawCommand, -10000, 10000, -ratescale, ratescale);  //Yaw is simple rate always
        testvar = pitchAnglePID._error*TODEG;
        testvar2 = pitchTarget*TODEG;
        testvar3 = ypr[2]*TODEG;
      }

      //==Run outer loop stabilization if hovering==
      if(verticalMode){
        pitchStab = (int)pitchPID.Compute();  //These give normalized servo commands (+-10000)
        rollStab = (int)rollPID.Compute();    //These are the "inner loop pids", taking in gyro rates and driving actuators
        yawStab = (int)yawPID.Compute();
        if(throttleCommand < - 9000) {pitchPID.Reset(); rollPID.Reset(); yawPID.Reset(); pitchAnglePID.Reset(); rollAnglePID.Reset();} //Reset I term if throttle low, no spoolup on ground.
      }
      else{
        pitchPID.Reset(); pitchStab = 0; pitch= 0; //Reset all PID's when they are not being used to prevent windup.
        rollPID.Reset(); rollStab = 0; roll = 0;    //Also reset all of their outputs just in case..
        yawPID.Reset(); yawStab = 0; yaw = 0;
        pitchAnglePID.Reset(); rollAnglePID.Reset();
      }
      //Serial.println(pitchStab);
      
    }
    //OPERATING WITH +-10000 RANGE HERE
    if (verticalMode) { //We are in "VTOL MODE", use stabilization and mix channels
      leftAileron = -pitchStab - yawStab;  //Mixing does not go over servo max pulses (servos used can take 300 - 2400 pulses apparently
      rightAileron = pitchStab - yawStab;
      leftMot = throttleCommand - rollStab;  //Motors do sinse escs are configured to 1000-2000
      rightMot = throttleCommand + rollStab;
      rudder = 0;
      elevator = 0;
    }
    else { //Plane mode fly by wire rules
      leftAileron = rollCommand;
      rightAileron = rollCommand;
      leftMot = throttleCommand - (abs(yawCommand)>2000 ? (yawCommand>0 ? yawCommand-2000 : yawCommand+2000)/3 : 0);  //Only use differential thrust if enough rudder applied
      rightMot = throttleCommand + (abs(yawCommand)>2000 ? (yawCommand>0 ? yawCommand-2000 : yawCommand+2000)/3 : 0); //Start applying diff thrust smoothly from zero
      rudder = yawCommand;
      elevator = pitchCommand;
    }
    //Trying to deal with motor saturation:
    int rangeTestVal = max(leftMot, rightMot); //Test if either motor is above max, reduce both to match highest if so
    if(rangeTestVal > 10000){
      leftMot -= rangeTestVal-10000;
      rightMot -= rangeTestVal-10000;
    }
    rangeTestVal = min(leftMot, rightMot);  //Same if they go under
     if(rangeTestVal < -10000){
      leftMot += -10000-rangeTestVal;
      rightMot += -10000-rangeTestVal;
    }
    
    //=================MOTOR CONTROL===============
    if(sbus.getChannel(11) > 1024){ //Motors are armed
      ch1.writeMicroseconds(ServoDenormalizer(leftMot,1000,1500,2000));
      ch2.writeMicroseconds(ServoDenormalizer(rightMot,1000,1500,2000)); 
      //ch2.writeMicroseconds(1000); 
    }
    else{  //Motors are disarmed
      ch1.writeMicroseconds(1000);
      ch2.writeMicroseconds(1000); 
    }
    ch3.writeMicroseconds(ServoDenormalizer(leftAileron,800,1300,1800));    // Left ail These contain the endpoints and centers
    ch4.writeMicroseconds(ServoDenormalizer(rightAileron,800,1300,1800));  //Right ail
    ch5.writeMicroseconds(ServoDenormalizer(elevator,1000,1400,1800));   //Elev
    ch6.writeMicroseconds(ServoDenormalizer(rudder,900,1350,2000));    //Rudd
    ch7.writeMicroseconds(map(sbus.getChannel(7), 0, 2047, 500, 2500));  //Wide limits to use maximum range of servo. This is how all of these should really be.

  }
  else {
    ch1.detach();  //Unattach servo signals to time out motor controllers ect
    ch2.detach();
    ch3.detach();
    ch4.detach();
    ch5.detach();
    ch6.detach();
    ch7.detach();
    outputOn = false;
  }


}

int ServoDenormalizer(int normalized, int minimum, int center, int maximum){  //translates normalized +-10000 to a given servo signal (in us)
  int returnval = 0;
  if(normalized > 0) returnval = map(normalized, 1,10000, center, maximum);
  else returnval = map(normalized, -10000,0, minimum, center);
  return returnval;
}

void LedHandler() {
  static unsigned long timer = 0;
  static unsigned long stamp = 0;
  if (millis() - timer > 100) { //Blink this to show program is running
    digitalWrite(boardLed, !digitalRead(boardLed));
    timer = millis();
  }

  digitalWrite(radioLed, !radioActive);

  if (!mpuOverflowDetect) digitalWrite(gyroLed, HIGH); //No overflows
  else {                                                   //OVerflows!!
    if (millis() - stamp > 50) {
      digitalWrite(gyroLed, !digitalRead(gyroLed));
      stamp = millis();
    }
  }
}

void DebugPrinter(unsigned long spd) {
  static unsigned long stamp = 0;
  if (millis() - stamp > spd) {
  #ifdef RAWOUTPUT  //Non human readable stuff for serial monitor
    Serial.print(ypr[0] * TODEG); Serial.print(',');
    Serial.print(ypr[1] * TODEG); Serial.print(',');
    Serial.println(ypr[2] * TODEG);
    //Serial.print(aa.x);Serial.print(',');
    //Serial.print(aa.y);Serial.print(',');
    //Serial.println(aa.z);
  #else
    Serial.print("ypr\t");
    Serial.print(ypr[0] * TODEG);
    Serial.print("\t");
    Serial.print(ypr[1] * TODEG);
    Serial.print("\t");
    Serial.print(ypr[2] * TODEG);
    Serial.print("\t");
    Serial.print(gx);
    Serial.print("\t");
    Serial.print(testvar);
    Serial.print("\t");
    Serial.print(testvar2);
    Serial.print("\t");
    Serial.print(testvar3);
    Serial.print("\t");
    Serial.println(loopRate);
  #endif
    stamp = millis();
  }
}

