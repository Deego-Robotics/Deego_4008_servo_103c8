# 速度开环控制实验

烧录后，发送**'T' + 速度 + 换行符** 即可让电机以指定速度运行

如, `T1.0\n` 即可让电机以1.0弧度每秒的速度运行

```c++
// 速度开环控制实验

#include <Arduino.h>
#include <SimpleFOC.h>
#include <TLE5012b.h>
#include <config.h>

TLE5012B sensor = TLE5012B();

HardwareSerial Serial1(RS485_RX_PIN, RS485_TX_PIN);

// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(11);

// BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
BLDCDriver3PWM driver = BLDCDriver3PWM(PWM_A_PIN, PWM_B_PIN, PWM_C_PIN, DRIVER_EN_PIN);

InlineCurrentSense current_sense  = InlineCurrentSense(0.01, 50, CURRENT_SENSE_A_PIN, CURRENT_SENSE_B_PIN);

//target variable
float target_velocity = 1.0;

// instantiate the commander
Commander command = Commander(Serial1);

void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

void setup() {

  initPorts();

  sensor.init();

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // limiting motor movements
  motor.voltage_limit = 3;   // [V]
  motor.velocity_limit = 5; // [rad/s] cca 50rpm
 
  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;

  // init motor hardware
  motor.init();

  current_sense.init();

  // add target command T
  command.add('T', doTarget, "target velocity");

  RS485_ON;

  Serial1.begin(115200);
  Serial1.println("Motor ready!");
  Serial1.println("Set target velocity [rad/s]");

  _delay(1000);

  LED_ON;

}

void loop() {

  // open loop velocity movement
  // using motor.voltage_limit and motor.velocity_limit
  motor.move(target_velocity);

  // user communication
  command.run();

}
```

