#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>
#include <esp_system.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

constexpr int ESC_PIN = 25;
constexpr int STEERING_SERVO_PIN = 26;

constexpr int ESC_REVERSE_MAX_US = 1000;
constexpr int ESC_NEUTRAL_US = 1500;
constexpr int ESC_FORWARD_MAX_US = 2000;
// Adjust according to the ESC manual. This is a braking command, not reverse.
constexpr int ESC_BRAKE_MAX_US = 1400;
constexpr unsigned long ESC_ARM_DELAY_MS = 3000;
constexpr int LIMITED_THROTTLE_PERCENT = 60;

// The vehicle steering range is limited to 100 degrees around the center.
constexpr int STEERING_SERVO_MIN_US = 500;
constexpr int STEERING_SERVO_MAX_US = 2500;
constexpr int STEERING_CENTER_DEG = 90;
constexpr int STEERING_RANGE_DEG = 50;
constexpr int STEERING_MIN_DEG = STEERING_CENTER_DEG - STEERING_RANGE_DEG;
constexpr int STEERING_MAX_DEG = STEERING_CENTER_DEG + STEERING_RANGE_DEG;
// Reduce the effective Xbox steering output while retaining the 100-degree limit.
constexpr int STEERING_INPUT_SCALE_PERCENT = 60;
constexpr float STEERING_TRIM_STEP_DEG = 0.5f;
constexpr float STEERING_TRIM_LIMIT_DEG = (float)STEERING_RANGE_DEG;
constexpr unsigned long STEERING_CHECK_STEP_MS = 500;

constexpr int TRIGGER_DEADZONE = 10;
constexpr int JOYSTICK_DEADZONE = 2500;
constexpr bool INVERT_THROTTLE = false;
constexpr bool INVERT_STEERING = false;
constexpr bool RESTART_AFTER_FAILED_CONNECTIONS = false;

Servo escMotor;
Servo steeringServo;
unsigned long escArmStartedAt = 0;
unsigned long lastLoopHeartbeatAt = 0;
float steeringTrimDeg = 0.0f;
bool throttleLimited = true;
bool previousBtnA = false;
bool previousBtnY = false;
bool previousBtnX = false;
bool previousBtnB = false;
bool previousBtnLB = false;
bool previousBtnRB = false;
bool steeringCheckActive = false;
uint8_t steeringCheckStep = 0;
unsigned long steeringCheckStepStartedAt = 0;

// Required to replace with your xbox address
// 需要在此替换成自己的手柄蓝牙MAC地址
XboxSeriesXControllerESP32_asukiaaa::Core xboxController(
    "a8:8c:3e:42:8e:95");

const char *resetReasonName(esp_reset_reason_t reason)
{
  switch (reason)
  {
  case ESP_RST_POWERON:
    return "POWERON";
  case ESP_RST_EXT:
    return "EXTERNAL_RESET";
  case ESP_RST_SW:
    return "SOFTWARE_RESTART";
  case ESP_RST_PANIC:
    return "PANIC_CRASH";
  case ESP_RST_INT_WDT:
    return "INTERRUPT_WATCHDOG";
  case ESP_RST_TASK_WDT:
    return "TASK_WATCHDOG";
  case ESP_RST_WDT:
    return "OTHER_WATCHDOG";
  case ESP_RST_DEEPSLEEP:
    return "DEEPSLEEP";
  case ESP_RST_BROWNOUT:
    return "BROWNOUT_POWER_DROP";
  case ESP_RST_SDIO:
    return "SDIO";
  default:
    return "UNKNOWN";
  }
}

void printResetReason()
{
  const esp_reset_reason_t reason = esp_reset_reason();
  Serial.print("Reset reason: ");
  Serial.print((int)reason);
  Serial.print(" ");
  Serial.println(resetReasonName(reason));
}

void printLoopHeartbeat()
{
  if (millis() - lastLoopHeartbeatAt < 1000)
  {
    return;
  }

  lastLoopHeartbeatAt = millis();
  Serial.print("loop heartbeat, ms=");
  Serial.print(millis());
  Serial.print(", connected=");
  Serial.print(xboxController.isConnected());
  Serial.print(", waitingFirstNotification=");
  Serial.print(xboxController.isWaitingForFirstNotification());
  Serial.print(", failedConnections=");
  Serial.println(xboxController.getCountFailedConnection());
}

int readThrottleCommand()
{
  int throttle = (int)xboxController.xboxNotif.trigRT;

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

int readBrakeCommand()
{
  const int brake = (int)xboxController.xboxNotif.trigLT;
  return brake < TRIGGER_DEADZONE ? 0 : brake;
}

int throttleToEscPulse(int throttle)
{
  const int maxTrig = XboxControllerNotificationParser::maxTrig;
  const int throttleLimitPercent =
      throttleLimited ? LIMITED_THROTTLE_PERCENT : 100;
  const int forwardLimitUs = ESC_NEUTRAL_US +
                             (ESC_FORWARD_MAX_US - ESC_NEUTRAL_US) *
                                 throttleLimitPercent / 100;
  const int reverseLimitUs = ESC_NEUTRAL_US -
                             (ESC_NEUTRAL_US - ESC_REVERSE_MAX_US) *
                                 throttleLimitPercent / 100;

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

int brakeToEscPulse(int brake)
{
  const int maxTrig = XboxControllerNotificationParser::maxTrig;
  return map(brake, 0, maxTrig, ESC_NEUTRAL_US, ESC_BRAKE_MAX_US);
}

float readSteeringOffsetDegrees()
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

  return (float)offset / (float)joyCenter *
         (float)STEERING_RANGE_DEG *
         (float)STEERING_INPUT_SCALE_PERCENT / 100.0f;
}

float readSteeringAngle()
{
  const float angle = (float)STEERING_CENTER_DEG + steeringTrimDeg +
                      readSteeringOffsetDegrees();
  return constrain(angle, (float)STEERING_MIN_DEG,
                   (float)STEERING_MAX_DEG);
}

void writeSteeringAngle(float angle)
{
  angle = constrain(angle, (float)STEERING_MIN_DEG,
                    (float)STEERING_MAX_DEG);
  const int pulse = (int)lroundf(
      (float)STEERING_SERVO_MIN_US +
      angle / 180.0f *
          (float)(STEERING_SERVO_MAX_US - STEERING_SERVO_MIN_US));
  steeringServo.writeMicroseconds(pulse);
}

void startSteeringCheck()
{
  steeringCheckActive = true;
  steeringCheckStep = 0;
  steeringCheckStepStartedAt = 0;
  Serial.println("Steering check started: 40 -> 90 -> 140 -> 90 deg");
}

void updateThrottleLimit()
{
  const bool btnA = xboxController.xboxNotif.btnA;
  const bool btnY = xboxController.xboxNotif.btnY;

  if (btnA && !previousBtnA)
  {
    throttleLimited = true;
    Serial.println("Throttle limit enabled: 60 percent");
  }
  else if (btnY && !previousBtnY && !btnA)
  {
    throttleLimited = false;
    Serial.println("Throttle limit disabled: 100 percent");
  }

  previousBtnA = btnA;
  previousBtnY = btnY;
}

void serviceSteeringCheck()
{
  static const int checkAngles[] = {STEERING_MIN_DEG, STEERING_CENTER_DEG,
                                    STEERING_MAX_DEG, STEERING_CENTER_DEG};
  const unsigned long now = millis();

  if (steeringCheckStepStartedAt != 0 &&
      now - steeringCheckStepStartedAt < STEERING_CHECK_STEP_MS)
  {
    return;
  }

  if (steeringCheckStep >= sizeof(checkAngles) / sizeof(checkAngles[0]))
  {
    steeringCheckActive = false;
    Serial.println("Steering check finished");
    return;
  }

  writeSteeringAngle((float)checkAngles[steeringCheckStep]);
  Serial.print("Steering check angle: ");
  Serial.print(checkAngles[steeringCheckStep]);
  Serial.println(" deg");
  steeringCheckStep++;
  steeringCheckStepStartedAt = now;
}

void updateSteeringTrim()
{
  const bool btnX = xboxController.xboxNotif.btnX;
  const bool btnB = xboxController.xboxNotif.btnB;
  const bool btnLB = xboxController.xboxNotif.btnLB;
  const bool btnRB = xboxController.xboxNotif.btnRB;

  // LB+RB is an edge-triggered steering self-check command.
  if (btnLB && btnRB && !(previousBtnLB && previousBtnRB))
  {
    startSteeringCheck();
  }
  else if (btnX && !previousBtnX)
  {
    steeringTrimDeg = constrain(steeringTrimDeg - STEERING_TRIM_STEP_DEG,
                                -STEERING_TRIM_LIMIT_DEG,
                                STEERING_TRIM_LIMIT_DEG);
  }
  else if (btnB && !previousBtnB)
  {
    steeringTrimDeg = constrain(steeringTrimDeg + STEERING_TRIM_STEP_DEG,
                                -STEERING_TRIM_LIMIT_DEG,
                                STEERING_TRIM_LIMIT_DEG);
  }

  previousBtnX = btnX;
  previousBtnB = btnB;
  previousBtnLB = btnLB;
  previousBtnRB = btnRB;
}

void stopVehicle()
{
  escMotor.writeMicroseconds(ESC_NEUTRAL_US);
  writeSteeringAngle((float)STEERING_CENTER_DEG + steeringTrimDeg);
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
  escArmStartedAt = millis();
}

bool isEscArmed()
{
  return millis() - escArmStartedAt >= ESC_ARM_DELAY_MS;
}

void applyXboxToVehicleControl()
{
  if (!isEscArmed())
  {
    stopVehicle();
    return;
  }

  updateThrottleLimit();
  updateSteeringTrim();
  if (steeringCheckActive)
  {
    escMotor.writeMicroseconds(ESC_NEUTRAL_US);
    return;
  }

  const int brake = readBrakeCommand();
  if (brake > 0)
  {
    escMotor.writeMicroseconds(brakeToEscPulse(brake));
  }
  else
  {
    escMotor.writeMicroseconds(throttleToEscPulse(readThrottleCommand()));
  }
  writeSteeringAngle(readSteeringAngle());
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
  delay(200);
  Serial.println("Starting NimBLE Client");
  printResetReason();
  Serial.println("Setting up vehicle control");
  setupVehicleControl();
  Serial.println("Vehicle control ready");
  Serial.println("Starting Xbox controller BLE");
  xboxController.begin();
  Serial.println("Xbox controller BLE ready");
}

void loop()
{
  printLoopHeartbeat();
  xboxController.onLoop();

  if (steeringCheckActive)
  {
    // Keep the ESC stopped while the steering mechanism is being checked.
    escMotor.writeMicroseconds(ESC_NEUTRAL_US);
    serviceSteeringCheck();
    return;
  }

  if (xboxController.isConnected())
  {
    if (xboxController.isWaitingForFirstNotification())
    {
      stopVehicle();
      Serial.println("time to shine");
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
    Serial.println("hirari do");
    if (RESTART_AFTER_FAILED_CONNECTIONS &&
        xboxController.getCountFailedConnection() > 2)
    {
      ESP.restart();
    }
  }
}
