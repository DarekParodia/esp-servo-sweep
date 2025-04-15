#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h> // I2C library
#include "MPU6050_6Axis_MotionApps20.h"

// Create MPU6050 instance
MPU6050 mpu;

// DMP and FIFO variables
bool dmpReady = false;  // set true if DMP init was successful
uint16_t packetSize;    // expected DMP packet size
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Variables to hold orientation data
Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];        // [yaw, pitch, roll]   roll/pitch/yaw container

Servo myservo1;
Servo myservo2;
Servo myservo3;

int servoPin1 = 13;
int servoPin2 = 12;
int servoPin3 = 14;

int pos1 = 0;
int pos2 = 0;
int pos3 = 0;

const int servo_frequency = 200; // Frequency in Hz

int sweep_mode = 3; // 0 = sweep, 1 = out of phase, 2 = out of phase (back and forth), 3 = MPU6050
// sweep
int sweep_delay = 10; // Delay in milliseconds

// phase
int phase_delay = 10;        // Delay in milliseconds
int phase_displacement = 60; // Phase displacement in degrees
int phase_addition = 1;      // Phase addition in degrees
long current_phase = 0;      // Current phase in degrees

// MPU6050
const int numSamples = 10;
float pitchSamples[numSamples] = {0};
int sampleIndex = 0;

float getFilteredPitch(float newPitch)
{
  pitchSamples[sampleIndex] = newPitch;
  sampleIndex = (sampleIndex + 1) % numSamples;
  float sum = 0;
  for (int i = 0; i < numSamples; i++)
  {
    sum += pitchSamples[i];
  }
  return sum / numSamples;
}

const int numSamples2 = 10;
float yawSamples[numSamples2] = {0};
int sampleIndex2 = 0;
float getFilteredYaw(float newYaw)
{
  yawSamples[sampleIndex2] = newYaw;
  sampleIndex2 = (sampleIndex2 + 1) % numSamples2;
  float sum = 0;
  for (int i = 0; i < numSamples2; i++)
  {
    sum += yawSamples[i];
  }
  return sum / numSamples2;
}

void setAllServoPositions(int pos1, int pos2, int pos3)
{
  myservo1.write(pos1);
  myservo2.write(pos2);
  myservo3.write(pos3);
}

void setAllServoPositions(int pos)
{
  myservo1.write(pos);
  myservo2.write(pos);
  myservo3.write(pos);
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
  } // wait for Serial to be available

  Wire.begin();

  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Setup servo frequency
  myservo1.setPeriodHertz(servo_frequency); // Analog servos run at ~60 Hz updates
  myservo2.setPeriodHertz(servo_frequency); // Analog servos run at ~60 Hz updates
  myservo3.setPeriodHertz(servo_frequency); // Analog servos run at ~60 Hz updates

  // Setup the servo pins
  myservo1.attach(servoPin1, 1000, 2000);
  myservo2.attach(servoPin2, 1000, 2000);
  myservo3.attach(servoPin3, 1000, 2000);

  // Set the initial position of the servos
  myservo1.write(0);
  myservo2.write(0);
  myservo3.write(0);

  // setup mpu6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();

  // Verify connection
  if (!mpu.testConnection())
  {
    Serial.println("MPU6050 connection failed");
    while (1)
      ;
  }
  Serial.println("MPU6050 connection successful");

  // Initialize DMP
  uint8_t devStatus = mpu.dmpInitialize();
  if (devStatus == 0)
  {
    // Turn on the DMP
    mpu.setDMPEnabled(true);
    dmpReady = true;
    // Get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("DMP initialized successfully");
  }
  else
  {
    // If DMP initialization fails, print the error code.
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
    while (1)
      ;
  }

}

void mode0()
{
  for (pos1 = 0; pos1 <= 180; pos1 += 1)
  { // Sweep from 0 to 180 degrees
    setAllServoPositions(pos1);
    delay(sweep_delay); // Wait for the servo to reach the position
  }
  for (pos1 = 180; pos1 >= 0; pos1 -= 1)
  { // Sweep from 180 to 0 degrees
    setAllServoPositions(pos1);
    delay(sweep_delay); // Wait for the servo to reach the position
  }
}

void mode1()
{
  current_phase += phase_addition;
  // write servo positions
  pos1 = current_phase % 180;
  pos2 = (current_phase + phase_displacement) % 180;
  pos3 = (current_phase + (2 * phase_displacement)) % 180;
  setAllServoPositions(pos1, pos2, pos3);
  delay(phase_delay); // Wait for the servo to reach the position
}

void mode2()
{
  current_phase += phase_addition;
  // write servo positions
  pos1 = current_phase % 360;
  if (pos1 > 180)
  {
    pos1 = 360 - pos1;
  }
  pos2 = (current_phase + phase_displacement) % 360;
  if (pos2 > 180)
  {
    pos2 = 360 - pos2;
  }
  pos3 = (current_phase + (2 * phase_displacement)) % 360;
  if (pos3 > 180)
  {
    pos3 = 360 - pos3;
  }
  // write servo positions
  setAllServoPositions(pos1, pos2, pos3);
  delay(phase_delay); // Wait for the servo to reach the position
}

void mode3()
{
  // Only proceed if DMP is ready.
  if (!dmpReady)
    return;

  // Check if there is a full packet available in the FIFO buffer
  if (mpu.getFIFOCount() < packetSize)
  {
    return;
  }

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
      /* Display Euler angles in degrees */
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      // Serial.print("ypr\t");
      // Serial.print(ypr[0] * 180/M_PI);
      // Serial.print("\t");
      // Serial.print(ypr[1] * 180/M_PI);
      // Serial.print("\t");
      // Serial.println(ypr[2] * 180/M_PI);

      // Get the pitch angle
      float pitch = ypr[1] * 180 / M_PI;
      float yaw = ypr[0] * 180 / M_PI;
      // Filter the pitch angle
      float filteredPitch = getFilteredPitch(pitch);
      float filteredYaw = getFilteredYaw(yaw);

      // Three servos are 120 degrees apart rotate them acordingly to the filtered pitch and yaw
      pos1 = sin(0 * M_PI / 180) * filteredPitch + sin(90 * M_PI / 180) * filteredYaw;
      pos2 = sin(120 * M_PI / 180) * filteredPitch + sin(210 * M_PI / 180) * filteredYaw;
      pos3 = sin(240 * M_PI / 180) * filteredPitch + sin(330 * M_PI / 180) * filteredYaw;
      // Map the angles to the servo range
      pos1 = map(pos1, -180, 180, 0, 360);
      pos2 = map(pos2, -180, 180, 0, 360);
      pos3 = map(pos3, -180, 180, 0, 360);

      if (pos1 > 180)
      {
        pos1 = 360 - pos1;
      }
      if (pos2 > 180)
      {
        pos2 = 360 - pos2;
      }
      if (pos3 > 180)
      {
        pos3 = 360 - pos3;
      }

      // Write the servo positions
      setAllServoPositions(pos1, pos2, pos3);
      // Print the filtered pitch and yaw angles
      Serial.print("Filtered Pitch: ");
      Serial.print(filteredPitch);
      Serial.print("\tFiltered Yaw: ");
      Serial.print(filteredYaw);
      Serial.print("\tServo 1: ");
      Serial.print(pos1);
      Serial.print("\tServo 2: ");
      Serial.print(pos2);
      Serial.print("\tServo 3: ");
      Serial.println(pos3);

      
      // Serial.println(filteredPitch);
  }
}

void loop()
{
  // Switch for the sweep mode
  switch (sweep_mode)
  {
  case 0: // Sweep mode
    mode0();
    break;

  case 1: // Back and forth mode
    mode1();
    break;

  case 2: // Out of phase mode
    mode2();
    break;

  case 3: // MPU6050 mode
    mode3();
    break;

  default:
    break;
  }
}
