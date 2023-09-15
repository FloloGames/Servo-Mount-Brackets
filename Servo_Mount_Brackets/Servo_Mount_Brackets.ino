#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm;

// Servo details
const uint16_t SERVO_MAX = 550;  // Maximum servo pulse width (adjust according to your servo)
const uint16_t SERVO_MIN = 110;  // Minimum servo pulse width (adjust according to your servo)

const uint8_t X_SERVO_PIN = 0;
const uint8_t Y_SERVO_PIN = 4;

int targetValueY = 90;

void setup() {
  Serial.begin(9600);
  // Initialize the PCA9685
  pwm.begin();
  pwm.setPWMFreq(50);  // Set the PWM frequency (adjust according to your servo)

  // Set initial position of the servo
  setServoPosition(Y_SERVO_PIN, 90);  // Set to the desired angle (0-180 degrees)
  setServoPosition(X_SERVO_PIN, 90);  // Set to the desired angle (0-180 degrees)

  Serial.println("Write only int!\n IN DEGREES");
}
void loop() {
  if (Serial.available()) {
    String teststr = Serial.readString();
    teststr.trim();
    if (teststr.length() == 0) {
      Serial.println("Empty Input");
      return;
    }
    Serial.println("Input: '" + teststr + "'");
    uint16_t value = teststr.toInt();  //in degrees
    setServoPosition(Y_SERVO_PIN, value);
    setServoPosition(X_SERVO_PIN, value);
    // if (value < SERVO_MIN || value > SERVO_MAX) {
    //   Serial.println("value not in range");
    //   return;
    // }
    // pwm.setPWM(Y_SERVO_PIN, 0, value);
    // pwm.setPWM(X_SERVO_PIN, 0, value);
  }
}
void setServoPosition(uint8_t servoChannel, int angle) {
  angle = constrain(angle, 0, 180);
  // Convert the angle to pulse width
  uint16_t pulseWidth = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);

  // Set the pulse width on the servo channel
  pwm.setPWM(servoChannel, 0, pulseWidth);

  Serial.println("Set angle: " + (String)angle);
  Serial.println("Pulse Width: " + (String)pulseWidth);
}