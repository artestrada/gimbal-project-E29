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

// CENTER the gimbal before loop function runs (runnign this will go to default 90 degere position)
bool gimbalReady = false;

// Servo PWM center and range
const int pulseCenter = 307;   // 1.5ms pulse ~90 degrees
const int pulseRange  = 102 ;  // ~0.5ms = 102 ticks (full swing)

// Target orientations (neutral)
double targetRoll = 0.0;
double targetPitch = 0.0;
double targetYaw = 0.0;

//Sensor zero offsets
double offsetRoll = 0.0;
double offsetPitch = 0.0;
double offsetYaw = 0.0;

// Sensor inputs
double inputRoll, inputPitch, inputYaw;

// PID outputs
double outputRoll, outputPitch, outputYaw;

// PID constants (adjust if needed)
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

  // Debug pulse output
  Serial.print("Servo "); Serial.print(channel);
  Serial.print(" → pulse: "); Serial.println(pulse);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Booting...");

  // Init MPU6050 on I2C (GPIO22/23)
  I2C_MPU.begin(23,22);
  if (!mpu.begin(0x68, &I2C_MPU)) {
    Serial.println("MPU6050 not found");
    while (1);
  }
  Serial.println("MPU6050 initialized");

  // Init PCA9685 on I2C (GPIO32/33)
  I2C_PCA.begin(32, 33);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);
  Serial.println("PCA9685 initialized");

  // PID setup
  pitchPID.SetMode(AUTOMATIC);
  yawPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);

  pitchPID.SetOutputLimits(-90, 90);
  yawPID.SetOutputLimits(-90, 90);
  rollPID.SetOutputLimits(-90, 90);

  // Center all servos
  setServo(YAW_SERVO, 0);
  setServo(PITCH_SERVO, 0);
  setServo(ROLL_SERVO,0);
  Serial.println("Servos gyatted (centered). Please rizz and edge arms.");

  // Step 2: Wait for you to manually assemble and tighten
  Serial.println("Press any key in to continue... Gada gadee gada-gada doe");
  while (!Serial.available());

  // Step 3: After arms are tightened, capture sensor offsets
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  offsetRoll = atan2(accel.acceleration.y, accel.acceleration.z) * 180.0 / PI;
  offsetPitch = atan2(-accel.acceleration.x, sqrt(accel.acceleration.y * accel.acceleration.y + accel.acceleration.z * accel.acceleration.z)) * 180.0 / PI;
  offsetYaw = 0;
  yawAngle = 0.0;

  lastTime = millis();

  // Now stabilization can start
  gimbalReady = true;
  Serial.println("Neutral position captured. Starting PID stabilization!");
}
// ---------------------------------------------------------------------------------- //
void loop() {
  if (!gimbalReady) {
    // Hold motors at center (0 degrees)
    setServo(YAW_SERVO, 0);
    setServo(PITCH_SERVO, 0);
    setServo(ROLL_SERVO, 0);
    return; // Don't run PID yet
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

  // Debug orientation and PID output
  Serial.print("Roll: "); Serial.print(inputRoll);
  Serial.print(" | PID: "); Serial.print(outputRoll);
  Serial.print(" | Pitch: "); Serial.print(inputPitch);
  Serial.print(" | PID: "); Serial.print(outputPitch);
  Serial.print(" | Yaw: "); Serial.print(inputYaw);
  Serial.print(" | PID: "); Serial.println(outputYaw);
  }
  delay(15); // ~65 Hz control loop
}