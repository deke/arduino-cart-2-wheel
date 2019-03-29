#include <Servo.h>

const int DELAY_AFTER_SEVRVO_MOVEMENT = 250; // After the servo motor to the stability of the time
const int LEFT_ANGLE = 180;
const int RIGHT_ANGLE = 8;
const int FOWARD_ANGLE = 96;

// Left motor pins
#define MotorLeftBackPin     14
#define MotorLeftFrontPin    15

// Right motor pins
#define MotorRightBackPin    16
#define MotorRightFrontPin   17

//Ultrasonic sensor pins
#define UltrasonicOutputPin  8
#define UltrasonicInputPin   9
#define UltrasonicMotorPin   11

Servo ultrasonicServo;

void setup() {
  Serial.begin(9600);

  pinMode(MotorLeftBackPin, OUTPUT);
  pinMode(MotorLeftFrontPin, OUTPUT);

  pinMode(MotorRightBackPin, OUTPUT);
  pinMode(MotorRightFrontPin, OUTPUT);

  pinMode(UltrasonicInputPin, INPUT);
  pinMode(UltrasonicOutputPin, OUTPUT);

  ultrasonicServo.attach(UltrasonicMotorPin);
}

#pragma region MotorLeftBackPin     Cotrols
void setMotor(int rb, int r, int lb, int lf, int d, int delayScale = 50) {
  digitalWrite(MotorRightBackPin, rb);
  digitalWrite(MotorRightFrontPin, rf);
  digitalWrite(MotorLeftBackPin, lb);
  digitalWrite(MotorLeftFrontPin, lf);

  delay(d * delayScale);
}

void turnRight(int d) {
  setMotor(HIGH, LOW, LOW, HIGH, d);
}

void turnLeft(int d) {
  setMotor(LOW, HIGH, HIGH, LOW, d);
}

void forward(int d) {
  setMotor(HIGH, LOW, HIGH, LOW, d);
}

void stop(int d) {
  setMotor(LOW, LOW, LOW, LOW, d);
}

void back(int d) {
  setMotor(LOW, HIGH, LOW, HIGH, d);
}
#pragma endregion

#pragma region Ultra Sonic Sensor Controls
float getDistance(int direction, int delayAfterServoMovement = DELAY_AFTER_SEVRVO_MOVEMENT) {
  ultrasonicServo.write(direction);
  delay(delayAfterServoMovement);

  digitalWrite(UltrasonicOutputPin, LOW);   // For low voltage 2 μs ultrasonic launch
  delayMicroseconds(2);
  digitalWrite(UltrasonicOutputPin, HIGH);  // Let ultrasonic launch 10 μs high voltage, there is at least 10 μs
  delayMicroseconds(10);
  digitalWrite(UltrasonicOutputPin, LOW);    // Maintaining low voltage ultrasonic launch

  float rawValue = pulseIn(UltrasonicInputPin, HIGH);  // Read the time difference

  return rawValue / 5.8 / 10; // Will be time to distance measurement (calc taken from factory code. Not verified) (unit: cm)
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