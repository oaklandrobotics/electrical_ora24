// ESP32 Model: DOIT ESP32 DEVKIT V1

#include <Wire.h>
#include <PS4Controller.h>

// Variables to stre controller values
int xPos, yPos; // Left joystick X and Y position values used for differential drive movement
bool rBump, lBump; // Left and right bumper states used for E-Stop and Autonomous State respectivly
int deadband = 5; // Used to offset joystick so resting state is a value of 0

void setup() 
{
  PS4.begin("bc:03:58:28:67:42"); // Initialize PS4 controller with MAC address
  Serial.begin(9600);
  Wire.begin(8); // Join I2C bus with address #8
  Wire.onRequest(requestEvent); // Register request event
  Serial.print(sizeof(int));
}

void loop() 
{
  // Read controller values
  if (abs(PS4.LStickY()) > deadband) // Implements deadband
  {
    yPos = PS4.LStickY(); // Reads Y position
  }
  else {
    yPos = 0;
  }
  if (abs(PS4.LStickX()) > deadband) // Implements deadband
  {
    xPos = PS4.LStickX(); // Reads X position
  }
  else {
    xPos = 0;
  }
  rBump = PS4.R1(); // Reads right bumper state
  lBump = PS4.L1(); // Reads left bumper state

  // Print controller values for debugging
  Serial.print("Y Position: ");
  Serial.println(yPos);
  Serial.print("X Position: ");
  Serial.println(xPos);
  Serial.print("R1 Button State: ");
  Serial.println(rBump);
  Serial.print("L1 Button State: ");
  Serial.println(lBump);
  Serial.println();

  delay(100); // Delay for stability
}

// Function that executes whenever data is requested by master
// This function is registered as an event, see setup()
void requestEvent() 
{
  // Convert the integer joystick positions and boolean button state to byte arrays
  // uint16_t needed as a result of ESP32 and Arduino UNO having different sized integers
  byte yArray[sizeof(uint16_t)];
  byte xArray[sizeof(uint16_t)];
  byte rBumpArray[sizeof(bool)];
  byte lBumpArray[sizeof(bool)];
  memcpy(yArray, &yPos, sizeof(uint16_t));
  memcpy(xArray, &xPos, sizeof(uint16_t));
  memcpy(rBumpArray, &rBump, sizeof(bool));
  memcpy(lBumpArray, &lBump, sizeof(bool));

  // Send the byte arrays over I2C
  Wire.write(yArray, sizeof(uint16_t));
  Wire.write(xArray, sizeof(uint16_t));
  Wire.write(rBumpArray, sizeof(bool));
  Wire.write(lBumpArray, sizeof(bool));
}
