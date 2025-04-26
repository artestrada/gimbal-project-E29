#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_PWMServoDriver.h>
#include <PID_v1.h>
#include <Adafruit_Sensor.h>

// I2C Buses
#define I2C1_SDA_PIN 23
#define I2C1_SCL_PIN 22
#define I2C2_SDA_PIN 32
#define I2C2_SCL_PIN 33
TwoWire I2C_MPU = TwoWire(0); // SDA 22, SCL 23
TwoWire I2C_PCA = TwoWire(1); // SDA 32, SCL 33

// Devices
Adafruit_MPU6050 mpu;
Adafruit_PWMServoDriver pwm(0x40, I2C_PCA);

// Servo channels
#define YAW_SERVO  0
#define PITCH_SERVO 1
#define ROLL_SERVO 2

// CENTER the gimbal before loop function runs
bool gimbalReady = false;

// Servo PWM center and range
const int pulseCenter = 307;   // 1.5ms pulse ~90 degrees
const int pulseRange  = 102 ;  // ~0.5ms = 102 ticks (full swing)

// Target orientations (neutral)
double targetRoll = 0.0;
double targetPitch = 0.0;
double targetYaw = 0.0;

// Sensor zero offsets
double offsetRoll = 0.0;
double offsetPitch = 0.0;
double offsetYaw = 0.0;

// Sensor inputs
double inputRoll, inputPitch, inputYaw;

// PID outputs
double outputRoll, outputPitch, outputYaw;

// PID constants
double Kp = 2.5, Ki = 0.05, Kd = 0.8;

// PID controllers
PID rollPID(&inputRoll, &outputRoll, &targetRoll, Kp, Ki, Kd, DIRECT);
PID pitchPID(&inputPitch, &outputPitch, &targetPitch, Kp, Ki, Kd, DIRECT);
PID yawPID(&inputYaw, &outputYaw, &targetYaw, Kp, Ki, Kd, DIRECT);

// Time tracking for yaw integration
unsigned long lastTime = 0;
float yawAngle = 0.0;

// Set servo angle from -90 to 90 using center and range
void setServo(int channel, double angle) {
  int pulse = map((int)angle, -90, 90, pulseCenter - pulseRange, pulseCenter + pulseRange);
  pulse = constrain(pulse, pulseCenter - pulseRange, pulseCenter + pulseRange);
  pwm.setPWM(channel, 0, pulse);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Init MPU6050
  I2C_MPU.begin(23,22);
  if (!mpu.begin(0x68, &I2C_MPU)) {
    while (1); // Hang if MPU6050 not found
  }

  // Init PCA9685
  I2C_PCA.begin(32, 33);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  // PID setup
  rollPID.SetMode(AUTOMATIC);
  pitchPID.SetMode(AUTOMATIC);
  yawPID.SetMode(AUTOMATIC);

  rollPID.SetOutputLimits(-90, 90);
  pitchPID.SetOutputLimits(-90, 90);
  yawPID.SetOutputLimits(-90, 90);

  // Center all servos
  setServo(YAW_SERVO, 0);
  setServo(PITCH_SERVO, 0);
  setServo(ROLL_SERVO, 0);

  // Wait for you to assemble the gimbal
  while (!Serial.available());

  // Capture initial orientation
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  offsetRoll = atan2(accel.acceleration.y, accel.acceleration.z) * 180.0 / PI;
  offsetPitch = atan2(-accel.acceleration.x, sqrt(accel.acceleration.y * accel.acceleration.y + accel.acceleration.z * accel.acceleration.z)) * 180.0 / PI;
  offsetYaw = 0;
  yawAngle = 0.0;

  lastTime = millis();
  gimbalReady = true;
}

void loop() {
  if (!gimbalReady) {
    setServo(YAW_SERVO, 0);
    setServo(PITCH_SERVO, 0);
    setServo(ROLL_SERVO, 0);
    return;
  }

  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Calculate roll and pitch from accelerometer
  inputRoll = atan2(accel.acceleration.y, accel.acceleration.z) * 180 / PI;
  inputPitch = atan2(-accel.acceleration.x, sqrt(accel.acceleration.y * accel.acceleration.y + accel.acceleration.z * accel.acceleration.z)) * 180 / PI;

  // Integrate gyro.z to get yaw angle
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  yawAngle += gyro.gyro.z * dt;
  inputYaw = yawAngle - offsetYaw;
  lastTime = currentTime;

  // Compute PID outputs
  rollPID.Compute();
  pitchPID.Compute();
  yawPID.Compute();

  // Apply outputs to servos
  setServo(ROLL_SERVO, outputRoll);
  setServo(PITCH_SERVO, outputPitch);
  setServo(YAW_SERVO, outputYaw);

  delay(15); // ~65Hz control loop
}
