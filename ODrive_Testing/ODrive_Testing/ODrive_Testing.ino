#include <ODriveArduino.h>
#include <SoftwareSerial.h>

/*
  Interfaces with an ODrive S1, making its motor oscillate.
*/

SoftwareSerial odrive_serial(8, 9); // Cause UNO only has one hardware port
short baudrate = 19200; // Make sure ODrive is set to this as well

ODriveArduino odrive(odrive_serial); // Construct the ODrive interface
int speed = 0;
int speedHold = 0;
void setup() {
  odrive_serial.begin(baudrate);

  Serial.begin(115200); // Serial rate to PC

  delay(10);

  Serial.println("Waiting for ODrive...");
  // Ensure ODrive is connected before sending commands
  while (odrive.getState() == AXIS_STATE_UNDEFINED) {
    delay(100);
  }

  // We should have connection now, post some info
  Serial.print("DC Voltage: ");
  Serial.println(odrive.getParameterAsFloat("vbus_voltage"));

  Serial.println("Enabling closed loop control...");
  // Ensure ODrive is in Closed Loop mode before running motor
  while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrive.clearErrors();
    odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    delay(10);
  }

  Serial.println("ODrive running!");
}

void loop() {
  
  /*float sine_period = 2.0f; // Period of the position sine wave (seconds)

  float t = 0.001 * millis();
  float period_div = (TWO_PI / sine_period);

  float phase = t * period_div;

  // Send output commands to odrive
  odrive.setPosition(
    sin(phase), // Position
    cos(phase) * period_div // Velocity Feed-forward
  );*/
  speed = Serial.parseInt();
  if (speed != 0 && speed > 0)
  {
    speedHold = speed;
  }
  else if (speed < 0)
  {
    speedHold = 0;
  }
  odrive.setVelocity((float)speedHold);

  // Recieve feedback
  ODriveFeedback feedback = odrive.getFeedback();
  //Serial.print("pos: ");
  //Serial.print(feedback.pos);
  //Serial.print(", ");
  Serial.print("vel: ");
  Serial.print(feedback.vel);
  Serial.println();
}
