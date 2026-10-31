#include <Arduino.h>
#include <ESP32Servo.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

constexpr int ESC_PIN = 25;
constexpr int STEERING_SERVO_PIN = 26;

constexpr int ESC_REVERSE_MAX_US = 1000;
constexpr int ESC_NEUTRAL_US = 1500;
constexpr int ESC_FORWARD_MAX_US = 2000;
constexpr unsigned long ESC_ARM_DELAY_MS = 3000;

constexpr int STEERING_SERVO_MIN_US = 500;
constexpr int STEERING_SERVO_MAX_US = 2500;
constexpr int STEERING_CENTER_DEG = 90;
constexpr int STEERING_RANGE_DEG = 45;

constexpr int TRIGGER_DEADZONE = 20;
constexpr int JOYSTICK_DEADZONE = 2500;
constexpr int THROTTLE_LIMIT_PERCENT = 70;
constexpr bool INVERT_THROTTLE = false;
constexpr bool INVERT_STEERING = false;

Servo escMotor;
Servo steeringServo;

// Required to replace with your xbox address
// 需要在此替换成自己的手柄蓝牙MAC地址
XboxSeriesXControllerESP32_asukiaaa::Core
    xboxController("f4:6a:d7:a0:5c:59");

int readThrottleCommand()
{
  int throttle = (int)xboxController.xboxNotif.trigRT -
                 (int)xboxController.xboxNotif.trigLT;

  if (INVERT_THROTTLE)
  {
    throttle = -throttle;
  }

  if (abs(throttle) < TRIGGER_DEADZONE)
  {
    return 0;
  }

  return constrain(throttle, -(int)XboxControllerNotificationParser::maxTrig,
                   (int)XboxControllerNotificationParser::maxTrig);
}

int throttleToEscPulse(int throttle)
{
  const int maxTrig = XboxControllerNotificationParser::maxTrig;
  const int forwardLimitUs = ESC_NEUTRAL_US +
                             (ESC_FORWARD_MAX_US - ESC_NEUTRAL_US) *
                                 THROTTLE_LIMIT_PERCENT / 100;
  const int reverseLimitUs = ESC_NEUTRAL_US -
                             (ESC_NEUTRAL_US - ESC_REVERSE_MAX_US) *
                                 THROTTLE_LIMIT_PERCENT / 100;

  if (throttle > 0)
  {
    return map(throttle, 0, maxTrig, ESC_NEUTRAL_US, forwardLimitUs);
  }
  if (throttle < 0)
  {
    return map(-throttle, 0, maxTrig, ESC_NEUTRAL_US, reverseLimitUs);
  }
  return ESC_NEUTRAL_US;
}

int readSteeringAngle()
{
  const int joyCenter = XboxControllerNotificationParser::maxJoy / 2;
  int offset = (int)xboxController.xboxNotif.joyLHori - joyCenter;

  if (abs(offset) < JOYSTICK_DEADZONE)
  {
    offset = 0;
  }
  if (INVERT_STEERING)
  {
    offset = -offset;
  }

  const int angle = map(offset, -joyCenter, joyCenter,
                        STEERING_CENTER_DEG - STEERING_RANGE_DEG,
                        STEERING_CENTER_DEG + STEERING_RANGE_DEG);
  return constrain(angle, 0, 180);
}

void stopVehicle()
{
  escMotor.writeMicroseconds(ESC_NEUTRAL_US);
  steeringServo.write(STEERING_CENTER_DEG);
}

void setupVehicleControl()
{
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  escMotor.setPeriodHertz(50);
  escMotor.attach(ESC_PIN, ESC_REVERSE_MAX_US, ESC_FORWARD_MAX_US);
  steeringServo.setPeriodHertz(50);
  steeringServo.attach(STEERING_SERVO_PIN, STEERING_SERVO_MIN_US,
                       STEERING_SERVO_MAX_US);

  stopVehicle();
  delay(ESC_ARM_DELAY_MS);
}

void applyXboxToVehicleControl()
{
  if (xboxController.xboxNotif.btnA)
  {
    stopVehicle();
    return;
  }

  const int throttle = readThrottleCommand();
  escMotor.writeMicroseconds(throttleToEscPulse(throttle));
  steeringServo.write(readSteeringAngle());
}

String xbox_string()
{
  String str = String(xboxController.xboxNotif.btnY) + "," +
               String(xboxController.xboxNotif.btnX) + "," +
               String(xboxController.xboxNotif.btnB) + "," +
               String(xboxController.xboxNotif.btnA) + "," +
               String(xboxController.xboxNotif.btnLB) + "," +
               String(xboxController.xboxNotif.btnRB) + "," +
               String(xboxController.xboxNotif.btnSelect) + "," +
               String(xboxController.xboxNotif.btnStart) + "," +
               String(xboxController.xboxNotif.btnXbox) + "," +
               String(xboxController.xboxNotif.btnShare) + "," +
               String(xboxController.xboxNotif.btnLS) + "," +
               String(xboxController.xboxNotif.btnRS) + "," +
               String(xboxController.xboxNotif.btnDirUp) + "," +
               String(xboxController.xboxNotif.btnDirRight) + "," +
               String(xboxController.xboxNotif.btnDirDown) + "," +
               String(xboxController.xboxNotif.btnDirLeft) + "," +
               String(xboxController.xboxNotif.joyLHori) + "," +
               String(xboxController.xboxNotif.joyLVert) + "," +
               String(xboxController.xboxNotif.joyRHori) + "," +
               String(xboxController.xboxNotif.joyRVert) + "," +
               String(xboxController.xboxNotif.trigLT) + "," +
               String(xboxController.xboxNotif.trigRT) + "\n";
  return str;
};

/*
支持四种振动模式
left：上左电机动
right：上右电机动
center：下左电机和下右电机一起动，频率高力量小
shake：下左电机和下右电机一起动，频率低力量大

测试结果：
四种模式都可以调振动力度
下左电机和下右电机是绑定的，只能一起动，但是提供了两种振动模式，个人猜测是两种模式的原理是给电机不同的电压
可以随意搭配使用，但center和shake一起用的话执行的应该是shake
*/

// 配置参考
// repo.v.select.center = 0;
// repo.v.select.left = 0;
// repo.v.select.right = 0;
// repo.v.select.shake = 0;
// repo.v.power.center = 0; // x% power
// repo.v.power.left = 0;
// repo.v.power.right = 30;
// repo.v.power.shake = 0;
// repo.v.timeActive = 0; // 振动 x/100 秒，最大2.56秒(uint8_t)
// repo.v.timeSilent = 0;   // 静止 x/100 秒
// repo.v.countRepeat = 0;  // 循环次数 x+1

// 官方例程
void demoVibration()
{
  XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase repo;
  Serial.println("full power for 1 sec");
  xboxController.writeHIDReport(repo);
  delay(2000);

  repo.v.select.center = true;
  repo.v.select.left = false;
  repo.v.select.right = false;
  repo.v.select.shake = false;
  repo.v.power.center = 30; // 30% power
  repo.v.timeActive = 50;   // 0.5 second
  Serial.println("run center 30\% power in half second");
  xboxController.writeHIDReport(repo);
  delay(2000);

  repo.v.select.center = false;
  repo.v.select.left = true;
  repo.v.power.left = 30;
  Serial.println("run left 30\% power in half second");
  xboxController.writeHIDReport(repo);
  delay(2000);

  repo.v.select.left = false;
  repo.v.select.right = true;
  repo.v.power.right = 30;
  Serial.println("run right 30\% power in half second");
  xboxController.writeHIDReport(repo);
  delay(2000);

  repo.v.select.right = false;
  repo.v.select.shake = true;
  repo.v.power.shake = 30;
  Serial.println("run shake 30\% power in half second");
  xboxController.writeHIDReport(repo);
  delay(2000);

  repo.v.select.shake = false;
  repo.v.select.center = true;
  repo.v.power.center = 50;
  repo.v.timeActive = 20;
  repo.v.timeSilent = 20;
  repo.v.countRepeat = 2;
  Serial.println("run center 50\% power in 0.2 sec 3 times");
  xboxController.writeHIDReport(repo);
  delay(2000);
}

// 振动反馈，根据扳机按压力度调整振动力度
void demoVibration_2()
{
  XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase repo;
  static uint16_t TrigMax = XboxControllerNotificationParser::maxTrig;
  String str_1;
  repo.setAllOff();
  repo.v.select.left = true;
  repo.v.select.right = true;
  repo.v.power.left = (uint8_t)((float)xboxController.xboxNotif.trigLT / (float)TrigMax * 100);
  repo.v.power.right = (uint8_t)((float)xboxController.xboxNotif.trigRT / (float)TrigMax * 100);
  repo.v.timeActive = 50;
  xboxController.writeHIDReport(repo);
  str_1 = String(repo.v.power.left) + "," + String(repo.v.power.right) + "\n";

  Serial.print(str_1);
  delay(50);
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting NimBLE Client");
  setupVehicleControl();
  xboxController.begin();
}

void loop()
{
  xboxController.onLoop();
  if (xboxController.isConnected())
  {
    if (xboxController.isWaitingForFirstNotification())
    {
      stopVehicle();
      Serial.println("waiting for first notification");
    }
    else
    {
      applyXboxToVehicleControl();
      Serial.print(xbox_string());
      // demoVibration();
      // demoVibration_2();
    }
  }
  else
  {
    stopVehicle();
    Serial.println("not connected");
    if (xboxController.getCountFailedConnection() > 2)
    {
      ESP.restart();
    }
  }
}
