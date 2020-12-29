#include <SimpleFOC.h>

#include "src/I2Cdev/I2Cdev.h"
#include "src/MPU6050/MPU6050_6Axis_MotionApps_V6_12.h"
#include "Wire.h"

BLDCMotor motor1 = BLDCMotor(11);
BLDCMotor motor2 = BLDCMotor(11);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(10, 6, 5, 7);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(3, 13, 9, 7);

Encoder encoder1 = Encoder(2, 4, 500);
Encoder encoder2 = Encoder(A4, 12, 500);
// interrupt ruotine intialisation
void doA1(){encoder1.handleA();}
void doB1(){encoder1.handleB();}
void doA2(){encoder2.handleA();}
void doB2(){encoder2.handleB();}


MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];         // [psi, theta, phi]    Euler angle container


float steering = 0;
float throttle = 0;
float target_pitch = 0;
PIDController pid_stb{30,300,1,100000,7};
PIDController pid_vel{0.01,0.05,0,10000,_PI/10};
LowPassFilter lpf_pitch{0.02};
LowPassFilter lpf_throttle{0.5};
LowPassFilter lpf_steering{0.1};

HardwareSerial Serial(PA12, PA11);

void setup() {
  // use monitoring with Serial for motor init
  // monitoring port
  Serial.begin(115200);
  
  // initialise encoder hardware
  encoder1.init();
  encoder1.enableInterrupts(doA1, doB1); 
  encoder2.init();
  encoder2.enableInterrupts(doA2, doB2); 
  
  // link the motor to the sensor
  motor1.linkSensor(&encoder1);
  motor2.linkSensor(&encoder2);

  // power supply voltage [V]
  driver1.voltage_power_supply = 12;
  driver1.init();
  motor1.linkDriver(&driver1);
  driver2.voltage_power_supply = 12;
  driver2.init();
  motor2.linkDriver(&driver2);

  // set control loop type to be used
  motor1.controller = ControlType::voltage;
  motor2.controller = ControlType::voltage;

  // enable monitoring
  motor1.useMonitoring(Serial);
  motor2.useMonitoring(Serial);

  // initialise motor
  motor1.init();
  motor2.init();
  // align encoder and start FOC
  motor1.initFOC();
  motor2.initFOC();

  // set the inital target value
  motor1.target = 0;
  motor2.target = 0;

  _delay(1000);
  // imu 
  initIMU();
  _delay(1000);
  Serial.println("Balancing robot ready!");
}
long t1 = 0;
long t2 = 0;
void loop() {
  // iterative setting FOC phase voltage
  motor1.loopFOC();
  motor2.loopFOC();

  // iterative function setting the outter loop target
  // velocity, position or voltage
  // if tatget not set in parameter uses motor.target variable
  motor1.move();
  motor2.move();

  if(t1++>0 && dmpReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)){
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    float pitch = ypr[2];
    if(t2++> 5){
      target_pitch = (pid_vel((motor1.shaft_velocity+ motor2.shaft_velocity)/2 - lpf_throttle(throttle)));
      t2=0;
    }
    float voltage_control = pid_stb(lpf_pitch(target_pitch) - pitch);
    float steering_adj = lpf_steering(steering);
    motor1.target = voltage_control + steering_adj;
    motor2.target = voltage_control - steering_adj;
    t1=0;
  }
  tuneController();
  readUserCommands();
}

void initIMU(){
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
   // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
//  mpu.setDLPFMode(MPU6050_DLPF_BW_20);  //10,20,42,98,188
  //mpu.setRate(4);   // 0=1khz 1=500hz, 2=333hz, 3=250hz 4=200hz
  mpu.setSleepEnabled(false);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    //mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  delay(2000);
  Serial.println("Adjusting DMP sensor fusion gain...");
  dmpSetSensorFusionAccelGain(0x20);

}

// DMP FUNCTIONS
// This function defines the weight of the accel on the sensor fusion
// default value is 0x80
// The official invense name is inv_key_0_96 (??)
void dmpSetSensorFusionAccelGain(uint8_t gain)
{
  // INV_KEY_0_96
  mpu.setMemoryBank(0);
  mpu.setMemoryStartAddress(0x60);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(gain);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(0);
}


/**
*   Function receiving throttle and steering reference
*   over Bluetooth Serial port
*
*  Receives byte via Serial/Bluetooth port
*   and decodes it to throttle and steering reference
*
*/
void readUserCommands() {
  int inByte;
  if (Serial.available() > 0) {
    while (Serial.available()) {
      inByte = Serial.read();
    }
    inByte = inByte - 100;
    if (inByte == 155) {
      // ON Byte
      //state = ON;
      Serial.println("Gibalo ON");
    } else if (inByte == 154) {
      // OFF Byte
      //state = OFF;
      Serial.println("Gibalo OFF");
    } else if (inByte >= -100 && inByte <= 100) {
      // throttle set-point Byte
      throttle = 20.0 *  ((float)inByte) / 100.0;
    } else if (inByte >= 110 && inByte <= 150) {
      // steering set-point Byte
      steering = 1.0 * ((float)(inByte - 130.0)) / 20.0; 
    } else {
      // Error Byte
      //steering = 0;
      throttle = 0;
      Serial.println("Error number!");
    }
    Serial.flush();
  }

}

// Serial communication callback function
// gets the target value from the user
void tuneController() {
  // a string to hold incoming data
  static String inputString; 
  while (Serial.available()) {
     // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline
    // end of input
    if (inChar == '\n') {
      if(inputString.charAt(0) == 'A'){
        pid_stb.P = inputString.substring(1).toFloat();
      }else if(inputString.charAt(0) == 'B'){
        pid_stb.D = inputString.substring(1).toFloat();
      }else if(inputString.charAt(0) == 'C'){
        pid_stb.I= inputString.substring(1).toFloat();
      }else if(inputString.charAt(0) == 'D'){
        pid_vel.P = inputString.substring(1).toFloat();
      }else if(inputString.charAt(0) == 'E'){
        pid_vel.D=inputString.substring(1).toFloat(); 
      }else if(inputString.charAt(0) == 'F'){
        pid_vel.I =inputString.substring(1).toFloat(); 
      }else if(inputString.charAt(0) == 'G'){
        lpf_pitch.Tf =inputString.substring(1).toFloat(); 
      }else{
        vel=inputString.toFloat();
      }
      inputString = "";
    }
  }
}
