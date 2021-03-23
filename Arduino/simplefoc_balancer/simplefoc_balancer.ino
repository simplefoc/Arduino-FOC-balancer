#include <SimpleFOC.h>
#include "src/I2Cdev/I2Cdev.h"
#include "src/MPU6050/MPU6050_6Axis_MotionApps_V6_12.h"
#include "Wire.h"

// pinout es32 - nucleo/arduino in comment
// driver pionouts 
#define M1A 5     // 10
#define M1B 27    // 5
#define M1C 16    // 6
#define M1En 12   // 8
#define M2A 25    // 9
#define M2B 18    // 3
#define M2C 13    // 11
#define M2En 14   // 7
// encoder pinouts
#define E1A 26    // 2
#define E1B 17    // 4
#define E2A 36    // A4
#define E2B 19    // 12

// create motor and driver instances
BLDCMotor motor1 = BLDCMotor(11);
BLDCMotor motor2 = BLDCMotor(11);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(M1A, M1B, M1C, M1En);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(M2A, M2B, M2C, M2En);

// create the encoder instances
Encoder encoder1 = Encoder(E1A, E1B, 500);
// interrupt ruotine intialisation
void doA1(){encoder1.handleA();}
void doB1(){encoder1.handleB();}
Encoder encoder2 = Encoder(E2A, E2B, 500);
void doA2(){encoder2.handleA();}
void doB2(){encoder2.handleB();}

// IMU instance
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

// control algorithm parameters
PIDController pid_stb{.P=30,.I=100,.D=1,.ramp=100000,.limit=7};
PIDController pid_vel{.P=0.01,.I=0.03,.D=0,.ramp=10000,.limit=_PI/10};
LowPassFilter lpf_pitch{.Tf=0.07};
// low pass filters for user commands - throttle and steering
LowPassFilter lpf_throttle{.Tf=0.5};
LowPassFilter lpf_steering{.Tf=0.1};

// serial instance for bluetooth communication
// ESP32 bluetooth serial 
#include "BluetoothSerial.h"
BluetoothSerial bluetooth;
// nucleo stm32
//HardwareSerial bluetooth(PA12, PA11);

// Gibalo app variables
float steering = 0;
float throttle = 0;
float max_throttle = 20; // 20 rad/s
float max_steering = 1; // 1 V
int state = 1; // 1 on / 0 off

// motion control tunning using commander
Commander commander = Commander(Serial);
void cntStab(char* cmd) { commander.pid(&pid_stb, cmd); }
void cntMove(char* cmd) { commander.pid(&pid_vel, cmd); }
void lpfPicth(char* cmd) { commander.lpf(&lpf_pitch, cmd); }
void lpfThrottle(char* cmd) { commander.lpf(&lpf_throttle, cmd); }
void lpfSteering(char* cmd) { commander.lpf(&lpf_steering, cmd); }

void setup() {
  // use monitoring with Serial for motor init
  // monitoring port
  Serial.begin(250000);
  // bluetooth esp32
  bluetooth.begin("FOCBalancer_esp");

  _delay(1000);
  // imu init and configure
  initIMU();
  if ( !dmpReady ){
    Serial.println(F("IMU connection problem... Disabling!"));
    bluetooth.println(F("IMU connection problem... Disabling!"));
    return;
  }
  _delay(1000);
  
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
  // using voltage torque mode
  motor1.controller = MotionControlType::torque;
  motor2.controller = MotionControlType::torque;

  // enable monitoring
  motor1.useMonitoring(Serial);
  motor2.useMonitoring(Serial);

  // initialise motor
  motor1.init();
  motor2.init();
  // align encoder and start FOC
  motor1.initFOC();
  motor2.initFOC();

  // add the configuration commands
  commander.add('A',cntStab,"pid stab");
  commander.add('B',cntMove,"pid vel");
  commander.add('C',lpfThrottle,"lpf vel command");
  commander.add('D',lpfPicth,"lpf throttle");
  commander.add('E',lpfSteering,"lpf steering");
  
  Serial.println(F("Balancing robot ready!"));
  bluetooth.println(F("Balancing robot ready!"));
}

void loop() {
  // iterative setting FOC phase voltage
  motor1.loopFOC();
  motor2.loopFOC();

  // iterative function setting the outter loop target
  motor1.move();
  motor2.move();

  if(!state){ // if balancer disabled
    motor1.target = 0;
    motor2.target = 0;
  }else if( dmpReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)){  // when IMU has received the package
    // read the package
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    // calculate the angle of the robot
    float pitch = _PI_2 - atan2(q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z, 2*(q.y * q.z + q.w * q.x));
    
    // calculate the target angle for throttle control
    float target_pitch = lpf_pitch(pid_vel((motor1.shaft_velocity+ motor2.shaft_velocity)/2 - lpf_throttle(throttle)));
    // calculate the target voltage
    float voltage_control = pid_stb(target_pitch - pitch);
    // filter steering
    float steering_adj = lpf_steering(steering);

    // set the tergat voltage value
    motor1.target = voltage_control + steering_adj;
    motor2.target = voltage_control - steering_adj;
  }

  // read the tuning commands from Serial
  commander.run();
  // read the user command from bluetooth
  readUserCommands(bluetooth);
}

// initialise and configure the IMU with DMP
void initIMU(){
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
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
    // mpu.CalibrateAccel(6);
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
  Serial.println(F("Adjusting DMP sensor fusion gain..."));
  mpu.setMemoryBank(0);
  mpu.setMemoryStartAddress(0x60);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(0x20);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(0);
}


/**
* Function intended for connecting to the Gibalo application
*/
void readBluetooth(Stream& bt_port) {
  int inByte;
  if (bt_port.available() > 0) {
    while (bt_port.available()) {
      inByte = bt_port.read();
    }
    inByte = inByte - 100;
    if (inByte == 155) {
      // ON Byte
      steering = 0;
      throttle = 0;
      state = 1;
      bt_port.println("State ON");
    } else if (inByte == 154) {
      // OFF Byte
      steering = 0;
      throttle = 0;
      state = 0;
      bt_port.println("State OFF");
    } else if (inByte >= -100 && inByte <= 100) {
      // throttle set-point Byte
      throttle = max_throttle *  ((float)inByte) / 100.0;
    } else if (inByte >= 110 && inByte <= 150) {
      // steering set-point Byte
      steering = max_steering * ((float)(inByte - 130.0)) / 20.0; 
    } else {
      // Error Byte
      steering = 0;
      throttle = 0;
      bt_port.println("Error number!");
    }
    bt_port.flush();
  }
}
