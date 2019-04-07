#include <Servo.h>
#include <NewPing.h>

const int DELAY_AFTER_SEVRVO_MOVEMENT = 250; // After the servo motor to the stability of the time
const int LEFT_ANGLE = 180;
const int RIGHT_ANGLE = 8;
const int FOWARD_ANGLE = 96;

// Left motor pins
#define MotorLeftBackPin     14
#define MotorLeftFrontPin    15
#define MotorLeftSpeedPin    5  //ENA

// Right motor pins
#define MotorRightBackPin    16
#define MotorRightFrontPin   17
#define MotorRightSpeedPin   6  //ENB

//Ultrasonic sensor pins
#define UltrasonicOutputPin  8
#define UltrasonicInputPin   9
#define UltrasonicMotorPin   11

NewPing sonar(UltrasonicOutputPin, UltrasonicInputPin);
Servo ultrasonicServo;

void setup() {
  Serial.begin(115200);

  pinMode(MotorLeftBackPin, OUTPUT);
  pinMode(MotorLeftFrontPin, OUTPUT);
  pinMode(MotorLeftSpeedPin, OUTPUT);

  pinMode(MotorRightBackPin, OUTPUT);
  pinMode(MotorRightFrontPin, OUTPUT);
  pinMode(MotorRightSpeedPin, OUTPUT);

  pinMode(UltrasonicInputPin, INPUT);
  pinMode(UltrasonicOutputPin, OUTPUT);

  ultrasonicServo.attach(UltrasonicMotorPin);
}

#pragma region MotorLeftBackPin     Cotrols
void setMotor(int rb, int rf, int lb, int lf, int rSpeed, int lSpeed, int d, int delayScale = 15) {
  digitalWrite(MotorRightBackPin, rb);
  digitalWrite(MotorRightFrontPin, rf);
  analogWrite(MotorRightSpeedPin, rSpeed);

  digitalWrite(MotorLeftBackPin, lb);
  digitalWrite(MotorLeftFrontPin, lf);
  analogWrite(MotorLeftSpeedPin, lSpeed);

  delay(d * delayScale);
}

void turnRight(int d) {
  setMotor(HIGH, LOW, LOW, HIGH, 120, 120, d);
}

void turnLeft(int d) {
  setMotor(LOW, HIGH, HIGH, LOW, 120, 120, d);
}

void forward(int d) {
  setMotor(HIGH, LOW, HIGH, LOW, 120, 120, d);
}

void stop(int d) {
  setMotor(LOW, LOW, LOW, LOW, 0, 0, d);
}

void back(int d) {
  setMotor(LOW, HIGH, LOW, HIGH, 120, 120, d);
}
#pragma endregion

#pragma region Ultra Sonic Sensor Controls
float getDistance(int direction, int delayAfterServoMovement = DELAY_AFTER_SEVRVO_MOVEMENT) {
  ultrasonicServo.write(direction);
  delay(delayAfterServoMovement);

  return sonar.ping_cm();
}

float getDistanceLeft() {
  return logAndReturnValue(getDistance(LEFT_ANGLE), "Left Distance");
}

float getDistanceRight() {
  return logAndReturnValue(getDistance(RIGHT_ANGLE), "Right Distance");
}

float getDistanceForward() {
  return logAndReturnValue(getDistance(FOWARD_ANGLE), "Forward Distance");
}
#pragma endregion

template <typename T>
inline T const& logAndReturnValue(T const& val, String const& label) {
  Serial.print(label + ": ");
  Serial.println(val);

  return val;
}

void goBackwards() {
  stop(1);
  back(10);
  Serial.println("Reverse");
}

void goLeft() {
  turnLeft(6);
  Serial.println("Turn Left");
}

void goRight() {
  turnRight(6);
  Serial.println("Turn Right");
}

void goForward() {
  forward(1);
  Serial.println("Forward");
}

void loop() {
  float forwardDistance = getDistanceForward();

  if(forwardDistance < 10) {
    goBackwards();
  }
  else if (forwardDistance < 25)
  {
    stop(1);
    float leftDistance = getDistanceLeft();
    float rightDistance = getDistanceRight();

    if (leftDistance < 15 && rightDistance < 15) {
      goBackwards();
    } else {
      leftDistance >= rightDistance ? goLeft() : goRight();
    }
  } else {
    goForward();
  }
}