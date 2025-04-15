#include <Arduino.h>
#include <ESP32Servo.h>

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

int sweep_mode = 2; // 0 = sweep, 1 = out of phase, 2 = out of phase (back and forth), 3 = MPU6050
// sweep
int sweep_delay = 10; // Delay in milliseconds

// phase
int phase_delay = 10; // Delay in milliseconds
int phase_displacement = 60; // Phase displacement in degrees
int phase_addition = 1; // Phase addition in degrees
long current_phase = 0; // Current phase in degrees

void setAllServoPositions(int pos1, int pos2, int pos3) {
  myservo1.write(pos1);
  myservo2.write(pos2);
  myservo3.write(pos3);
}

void setAllServoPositions(int pos) {
  myservo1.write(pos);
  myservo2.write(pos);
  myservo3.write(pos);
}

void setup() {
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
  
  delay(1000);
}

void loop() {
  // Switch for the sweep mode
  switch (sweep_mode) {
    case 0: // Sweep mode
      for (pos1 = 0; pos1 <= 180; pos1 += 1) { // Sweep from 0 to 180 degrees
        setAllServoPositions(pos1);
        delay(sweep_delay); // Wait for the servo to reach the position
      }
      for (pos1 = 180; pos1 >= 0; pos1 -= 1) { // Sweep from 180 to 0 degrees
        setAllServoPositions(pos1);
        delay(sweep_delay); // Wait for the servo to reach the position
      }
      break;
    case 1: // Back and forth mode
      current_phase += phase_addition;
      // write servo positions
      pos1 = current_phase % 180;
      pos2 = (current_phase + phase_displacement) % 180;
      pos3 = (current_phase + (2 * phase_displacement)) % 180;
      setAllServoPositions(pos1, pos2, pos3);
      delay(phase_delay); // Wait for the servo to reach the position
      break;
    case 2: // Out of phase mode
      current_phase += phase_addition;
      // write servo positions
      pos1 = current_phase % 360;
      if (pos1 > 180) {
        pos1 = 360 - pos1;
      }
      pos2 = (current_phase + phase_displacement) % 360;
      if (pos2 > 180) {
        pos2 = 360 - pos2;
      }
      pos3 = (current_phase + (2 * phase_displacement)) % 360;
      if (pos3 > 180) {
        pos3 = 360 - pos3;
      }
      // write servo positions
      setAllServoPositions(pos1, pos2, pos3);
      delay(phase_delay); // Wait for the servo to reach the position
      break;
    default:
      break;
  }
}
