

# 速度电压闭环控制实验

## 介绍

速度电流闭环模式下，系统的速度环PID输出量为Uq电压值

![速度闭环](pics/velocity_loop_v.png)

烧录后，发送**'T' + 速度 + 换行符** 即可让电机以指定速度运行

如, `T1.0\n` 即可让电机以1.0弧度每秒的速度运行

注意：

- 由于利用磁编的速度估计在低速下精度不高，因此速度控制更适合带负载的较快速运动
- 系统的供电电压越高，能达到的最大速度越大
- 不需要查看波形时，注释掉主循环的monitor部分，性能会更好（得到更高的稳定最大速度）

## 代码

```c++
// 速度电压闭环控制实验

#include <Arduino.h>
#include <SimpleFOC.h>
#include <TLE5012b.h>
#include <config.h>

TLE5012B sensor = TLE5012B();

HardwareSerial Serial1(RS485_RX_PIN, RS485_TX_PIN);

BLDCMotor motor = BLDCMotor(11);

BLDCDriver3PWM driver = BLDCDriver3PWM(PWM_A_PIN, PWM_B_PIN, PWM_C_PIN, DRIVER_EN_PIN);

InlineCurrentSense current_sense  = InlineCurrentSense(0.01, 50, CURRENT_SENSE_A_PIN, CURRENT_SENSE_B_PIN);

//target variable
float target_velocity = 0.0;

// instantiate the commander
Commander command = Commander(Serial1);

void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

void setup() {

  initPorts();

  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  // limiting motor movements
  motor.voltage_limit = 6;   // [V]
  motor.velocity_limit = 50; // [rad/s] cca 50rpm
 
  // set motion control loop to be used
  motor.controller = MotionControlType::velocity;

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.3;
  motor.PID_velocity.I = 10;
  motor.PID_velocity.D = 0;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 500;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best. 
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.02;

  current_sense.init();

  RS485_ON;

  Serial1.begin(115200);
  Serial1.println("Motor ready!");
  Serial1.println("Set target velocity [rad/s]");

  motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_VOLT_Q ;
  // downsampling
  motor.monitor_downsample = 100; 

  // comment out if not needed
  motor.useMonitoring(Serial1);
  // init motor hardware
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target velocity");

  _delay(1000);

  LED_ON;

}

void loop() {

  motor.loopFOC();

  motor.move(target_velocity);

  command.run();

  motor.monitor();
}
```

## 效果

![速度跟随效果](pics/4_1.png)

- 蓝色：目标速度
- 绿色：实际速度
- 红色：输出量（当前模式下为Uq）