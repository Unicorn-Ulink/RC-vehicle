#include <ESP32Servo.h>
#define esc_pin 16
#define servo_pin 17
Servo esc_motor;
Servo servo;

long int speed = 0;
int angle = 90;

const int esc_minpulse = 1000;
const int esc_maxpulse = 2000;
const int servo_minpulse = 500;
const int servo_maxpulse = 2500;

void setup() {
  Serial.begin(115200);

  esc_motor.attach(esc_pin, esc_minpulse, esc_maxpulse);
  esc_motor.writeMicroseconds(1500);

  servo.attach(servo_pin, servo_minpulse, servo_maxpulse);
  servo.write(angle);

  delay(5000);

  Serial.println("System ready! Send commands via Serial Monitor.");
  Serial.println("Format: F<value>  (e.g. F150)");
  Serial.println("        B<value>  (e.g. B80)");
  Serial.println("        L<value>  (e.g. L45)");
  Serial.println("        R<value>  (e.g. R60)");
  Serial.println("        S         (stop)");
}

void process(String a) {
  if (a.length() == 0) return;
  char type = a.charAt(0);
  long int value = a.substring(1).toInt();

  switch (type) {
    case 'F': {
        speed = map(value, 0, 180, 91, 180);
        esc_motor.write(speed);
        Serial.print("Forward: ");
        Serial.println(speed);
      }
      break;
    case 'B': {
        speed = map(value, 0, 180, 89, 0);
        esc_motor.write(speed);
        Serial.print("Backward: ");
        Serial.println(speed);
      }
      break;
    case 'S':
      speed = 0;
      esc_motor.write(0);
      esc_motor.write(90);
      Serial.println("Stop");
      break;
    case 'L':
      angle = map(value, 0, 90, 89, 0);
      servo.write(angle);
      Serial.print("Turn Left: ");
      Serial.println(angle);
      break;
    case 'R':
      angle = map(value, 0, 90, 91, 180);
      servo.write(angle);
      Serial.print("Turn Right: ");
      Serial.println(angle);
      break;
    default:
      Serial.println("Unknown command");
      break;
  }
}

void loop() {
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();
    process(msg);
  }

}