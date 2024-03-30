#include <Wire.h>

int yPos, xPos;
bool rBump;

void setup() 
{
  Wire.begin(8);        // join I2C bus (address optional for master)
  Serial.begin(9600);  // start serial for output
}

void loop() 
{
  Wire.requestFrom(8, sizeof(int) + sizeof(int) + sizeof(bool)); // request data from slave device #8

  if (Wire.available() >= sizeof(int) + sizeof(int) + sizeof(bool))
  {
    // Read yPos
    yPos = Wire.read();
    yPos |= Wire.read() << 8; // Combine with the next byte

    // Read xPos
    xPos = Wire.read();
    xPos |= Wire.read() << 8; // Combine with the next byte

    // Read rBump
    rBump = Wire.read();

    // Print joystick positions for debugging
    Serial.print("Y Position: ");
    Serial.println(yPos);
    Serial.print("X Position: ");
    Serial.println(xPos);
    Serial.print("R1 Button State: ");
    Serial.println(rBump);
    Serial.println();
  }

  delay(100);
}