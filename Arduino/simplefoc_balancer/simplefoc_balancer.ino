#include <SimpleFOC.h>

#include "balancer_pinouts.h"
#include "imu_helpers.h"

// serial instance for bluetooth communication
// ESP32 bluetooth serial
#include "BluetoothSerial.h"
BluetoothSerial bluetooth;
// bluetooth nucleo stm32
// HardwareSerial bluetooth(PA12, PA11);

// create motor and driver instances
BLDCMotor motor1 = BLDCMotor(11);
BLDCMotor motor2 = BLDCMotor(11);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(MOT1_A, MOT1_B, MOT1_C, MOT1_EN);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(MOT2_A, MOT2_B, MOT2_C, MOT2_EN);

// create the encoder instances
Encoder encoder1 = Encoder(ENC1_A, ENC1_B, 500);
// interrupt ruotine intialisation
void doA1() {  encoder1.handleA(); }
void doB1() {  encoder1.handleB(); }
Encoder encoder2 = Encoder(ENC2_A, ENC2_B, 500);
void doA2() {  encoder2.handleA(); }
void doB2() {  encoder2.handleB();}

// control algorithm parameters
// stabilisation pid
PIDController pid_stb{.P = 30, .I = 100, .D = 1, .ramp = 100000, .limit = 7};
// velocity pid
PIDController pid_vel{.P = 0.01, .I = 0.03, .D = 0, .ramp = 10000, .limit = _PI / 10};
// velocity control filtering
LowPassFilter lpf_pitch_cmd{.Tf = 0.07};
// low pass filters for user commands - throttle and steering
LowPassFilter lpf_throttle{.Tf = 0.5};
LowPassFilter lpf_steering{.Tf = 0.1};

// Bluetooth app variables
float steering = 0;
float throttle = 0;
float max_throttle = 20; // 20 rad/s
float max_steering = 1; // 1 V
int state = 1; // 1 on / 0 off

// motion control tunning using commander
Commander commander = Commander(Serial);
void cntStab(char* cmd) {  commander.pid(&pid_stb, cmd);}
void cntMove(char* cmd) {  commander.pid(&pid_vel, cmd);}
void lpfPitch(char* cmd) {  commander.lpf(&lpf_pitch_cmd, cmd);}
void lpfSteering(char* cmd) {  commander.lpf(&lpf_steering, cmd);}
void lpfThrottle(char* cmd) {  commander.lpf(&lpf_throttle, cmd);}

void setup() {
  // use monitoring with Serial for motor init
  // monitoring port
  Serial.begin(250000);
  // bluetooth esp32
  bluetooth.begin("FOCBalancer_esp");
  // bluetooth nucleo
  //  bluetooth.begin(115200);

  _delay(1000);
  // imu init and configure
  if ( !initIMU() ) {
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
  driver1.voltage_power_supply = 8;
  driver1.init();
  motor1.linkDriver(&driver1);
  driver2.voltage_power_supply = 8;
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
  commander.add('A', cntStab, "pid stab");
  commander.add('B', cntMove, "pid vel");
  commander.add('C', lpfThrottle, "lpf vel command");
  commander.add('D', lpfPitch, "lpf throttle");
  commander.add('E', lpfSteering, "lpf steering");

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

  if (!state) { // if balancer disabled
    motor1.target = 0;
    motor2.target = 0;
  } else if ( hasDataIMU() ) { // when IMU has received the package
    // read pitch from the IMU
    float pitch = getPitchIMU();
    // calculate the target angle for throttle control
    float target_pitch = lpf_pitch_cmd(pid_vel((motor1.shaft_velocity + motor2.shaft_velocity) / 2 - lpf_throttle(throttle)));
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
  handleBluetooth(bluetooth);
}

/**
  Function intended for connecting to the Gibalo application
  In one byte it receives either throttel or steering command
  - if received byte  in range [0,200] then throttle command in between [-100, 100]
  - if received byte  in range [210,250] then steering command in between [-20, 20]
*/
void handleBluetooth(Stream& bt_port) {
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
