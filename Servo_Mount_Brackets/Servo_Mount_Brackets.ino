#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm;

/* für unteren Servo (theta) (y)
const uint16_t SERVO_MAX = 540;//550;  // Maximum servo pulse width (adjust according to your servo)
const uint16_t SERVO_MIN = 110;  // Minimum servo pulse width (adjust according to your servo)*/
/* für oberen Servo (phi) (x)
const uint16_t SERVO_MAX = 560;//550;  // Maximum servo pulse width (adjust according to your servo)
const uint16_t SERVO_MIN = 110;  // Minimum servo pulse width (adjust according to your servo)
*/
// Servo details
const uint16_t SERVO_MAX = 560;  //550;  // Maximum servo pulse width (adjust according to your servo)
const uint16_t SERVO_MIN = 110;  // Minimum servo pulse width (adjust according to your servo)

const uint8_t X_SERVO_PIN = 0;
const uint8_t Y_SERVO_PIN = 4;

/*
0 = connected pos
1 = ready pos
2 = home  pos
*/
const short H_0s[3] = { 170, 150, 110 };

// setServoPosition(Y_SERVO_PIN, 70);  // up in the sky
// delay(1000);
// setServoPosition(Y_SERVO_PIN, 110);  //Set y horizontal
void setup() {
  Serial.begin(9600);
  delay(100);

  // Initialize the PCA9685
  pwm.begin();
  pwm.setPWMFreq(50);  // Set the PWM frequency (adjust according to your servo)

  // Set initial position of the servo

  // setServoPosition(X_SERVO_PIN, 90);      // Set to the desired angle (0-180 degrees)
  // setServoPosition(Y_SERVO_PIN, 90 - 6);  //Set y horizontal

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
    // float values[170];
    // long startMillis = millis();
    // for (int i = 70; i < 170; i++) {
    //   values[i] = calcThetaFromPhi(i, 80);
    // }

    // long endMillis = millis();
    // long neededTime = endMillis - startMillis;

    // Serial.println("Needed time: " + (String)neededTime);
    // for (int i = 70; i < 170; i++) {
    //   Serial.println(values[i]);
    // }
    // Serial.println("Input: '" + teststr + "'");

    uint16_t value = teststr.toInt();  //in degrees

    Serial.println("Calc Stuff");


    const double range = 50;
    // setServoPosition(Y_SERVO_PIN, value);
    value = constrain(value, 90 - range, 90 + range);
    setServoPosition(X_SERVO_PIN, value - 6);

    double theta = calcThetaFromPhi(value, H_0s[1]);
    Serial.println((String)theta);

    theta = constrain(theta, 70, 170);
    setServoPosition(Y_SERVO_PIN, theta);

    // if (value < SERVO_MIN || value > SERVO_MAX) {
    //   Serial.println("value not in range");
    //   return;
    // }
    // pwm.setPWM(Y_SERVO_PIN, 0, value);
    // pwm.setPWM(X_SERVO_PIN, 0, value);
  }
}
double calcThetaFromPhi(uint16_t phi, uint16_t H_0) {

  const auto H_00 = 170;  //175;
  const auto s = 150.0;   //158.2;
  const auto r_k = 100;

  const auto delta_h = (1 - sin(radians(phi))) * r_k;
  Serial.println("delta_h: " + (String)delta_h);
  const auto l_0 = sqrt(H_00 * H_00 + s * s);
  Serial.println("l_0: " + (String)l_0);
  const auto l = sqrt(pow(H_00 - delta_h, 2) + s * s);
  Serial.println("l: " + (String)l);
  const auto theta_0 = asin(H_0 / l_0);
  const auto theta_00 = atan(H_00 / s);
  const auto delta_theta = asin(H_0 / l) - asin((H_0 - delta_h * cos(theta_00 - theta_0)) / l);
  Serial.println("delta_theta: " + (String)degrees(delta_theta));
  Serial.println("H_0/l: " + (String)(H_0 / l));
  Serial.println("H_0-delta_h*cos...: " + (String)((H_0 - delta_h * cos(theta_00 - theta_0)) / l));
  auto theta = PI / 2 + theta_00 - theta_0 - delta_theta;
  Serial.println("theta_0: " + (String)degrees(theta_0));
  Serial.println("theta_00: " + (String)degrees(theta_00));
  Serial.println("theta: " + (String)degrees(theta));
  return degrees(theta);  // - delta_theta;

  // cons
  // asin()
  // return -1;
}
void setServoPosition(uint8_t servoChannel, double angle) {
  angle = constrain(angle, 0, 180);
  // Convert the angle to pulse width
  uint16_t pulseWidth = (int)(map(angle, 0, 180, SERVO_MIN, SERVO_MAX) + 0.5);

  // Set the pulse width on the servo channel
  pwm.setPWM(servoChannel, 0, pulseWidth);

  Serial.println("Set angle: " + (String)angle);
  Serial.println("Pulse Width: " + (String)pulseWidth);
}