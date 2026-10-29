#include <iostream>
#include <math.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <BluetoothSerial.h>

#define esc_pin 16  //电调引脚
#define servo_pin 17  //舵机引脚

BluetoothSerial SerialBluetooth;
Servo esc_motor;  //电机
Servo servo;  //舵机

long int speed = 0; //电机初始速度
int angle = 90;  //舵机初始位置

//*10^-6
const int esc_minpulse = 1000;  //电调脉宽 >1.5ms前进 <1.5ms后退
const int esc_maxpulse = 2000;
const int servo_minpulse = 500;  //舵机脉宽 0-180度
const int servo_maxpulse = 2500;

void setup(){
  Serial.begin(115200);
  SerialBluetooth.begin("rc");  //蓝牙设备名称
  esc_motor.attach(esc_pin, esc_minpulse, esc_maxpulse);
  esc_motor.writeMicroseconds(esc_minpulse);
  servo.attach(servo_pin, servo_minpulse, servo_maxpulse);
  servo.write(angle);
}

void process(String a){
  //指令格式 type + value eg."F160"; "R30"; "S"
  //F:前 B:后 L:左 R:右 S:停
  char type = a.charAt(0);
  long int value = a.substring(1).toInt();

  switch (type) {
    case 'F': { // 前进
        speed = map(value, 0, 180, 91, 180);
        esc_motor.write(speed);
      }
      break;
      
    case 'B': { // 后退
        speed = map(value, 0, 180, 89, 0);
        esc_motor.write(speed);
      }
      break;
      
    case 'S': // 停车
      speed = 0;
      esc_motor.write(0);
      break;
      
    case 'L': // 左转
      angle = map(value, 0, 90, 89, 0);
      servo.write(angle);
      break;
      
    case 'R': // 右转
      angle = map(value, 0, 90, 91, 180);
      servo.write(angle);
      break;
      
    default:
      break;
  }
}

void loop(){
  if (SerialBluetooth.available()) {
    String msg = SerialBluetooth.readStringUntil('\n'); // 读取指令
    msg.trim();
    process(msg);
}