#include <mcp_can.h>
#include <SPI.h>
#include <Wire.h>

// digital pins available on control board: 0, 1, 10, 5, 13
// analog pins available on control board: 0, 1, 2, 3, 4

// CAN / ODrive Declarations
#define ODRV0_NODE_ID 0 // Left Motor
#define ODRV1_NODE_ID 1 // Right Motor
#define CAN0_INT 2    // Set INT to pin 2 (This is the Interrupt pin)
#define CAN_BAUDRATE 500000
MCP_CAN CAN0(17);   // Set CS to pin 17 (This is the Chip select)

#define EStop 0         // Hardware EStop

// IMU Declarations
#define IMU_SCL 3 // On AtMega32U4 Connected to Pin 18 (PD0) // CHECK
#define IMU_SDA 2 // On AtMega32U4 Connected to Pin 19 (PD1)

// LED Declarations
#define LEDPWR_R 4 // On AtMega32U4 Connected to Pin 25 (PD4)
#define LEDPWR_G 12 // On AtMega32U4 Connected to Pin 26 (PD6)
#define LEDPWR_B 6 // On AtMega32U4 Connected to Pin 27 (PD7)

// Battery Voltage Detection Declaration
#define Battery_Voltage A0 // On AtMega32U4 Connected to Pin 27 (PF0)

double verticalMov = 0;
double horizontalMov = 0;
bool EStopButton = false;
bool EStopState = false;

void setup()
{
  Wire.begin(8);
  pinMode(EStop, INPUT);

  pinMode(CAN0_INT, INPUT);

  pinMode(IMU_SCL, INPUT); // Work on later
  pinMode(IMU_SDA, INPUT); // Work on later

  pinMode(LEDPWR_R, INPUT);
  pinMode(LEDPWR_G, INPUT);
  pinMode(LEDPWR_B, INPUT);

  pinMode(Battery_Voltage, INPUT);

  Serial.begin(115200);

  // Continuously tries to establish connection to MCP chip until it does
  while(1)
  {
    // Tries to initialize MCP CAN chip with clock of 8MHz and data transfer speed is 500KB/second
    if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    {
      Serial.println("MCP2515 Initialized Successfully!");
      break;
    }
    else 
    {
      Serial.println("Error Initializing MCP2515...");
      delay(20);
    }
  }
}

void loop() 
{
  
  Wire.requestFrom(8, sizeof(int) + sizeof(int) + sizeof(bool)); // request data from slave device #8 (ESP32)
  
  // Checks to see if data is recieved over I2C. If so sets values from controller to predefined variables
  if(Wire.available() >= sizeof(int) + sizeof(int) + sizeof(bool)) // could combine with If statement below but this is easier to read for now
  {
    // Read vertical movement from left joystick
    int verticalMov = Wire.read();
    verticalMov |= Wire.read() << 8; // Combine with the next byte

    // Read horizontal movement from left joystick
    int horizontalMov = Wire.read();
    horizontalMov |= Wire.read() << 8; // Combine with the next byte

    // Read EStopButton from right bumper
    int EStopButton = Wire.read();
    
    // Print received values
    Serial.print("Received from ESP32 - Vertical Movement: ");
    Serial.println(verticalMov);

    Serial.print("Received from ESP32 - Horizontal Movement: ");
    Serial.println(horizontalMov);

    Serial.print("Received from ESP32 - EStop Button: ");
    Serial.println(EStopButton);

    delay(10);
  } 
  else
  {
    verticalMov = 0;
    horizontalMov = 0;
    EStopButton = false; // should it be true?
    Serial.println("Insufficient bytes received");
  }

  // Allows EStop to be toggled
  if(EStopButton && !EStopState)
  {
    // Send EStop command to ODrive
    ODriveEStop();
    EStopState = true;
  }
  else if(EStopButton && EStopState)
  {
    ODriveControlState();
    ODriveMovement(verticalMov, horizontalMov);
    EStopState = false;
  }
}

// Converts Joystick values into Revolutions/Second and sends data to ODrive over CAN
void ODriveMovement(double verticalVelocity, double horizontalVelocity) // redo math for turns/second
{
  // Scales inputs to directional vector
  verticalVelocity = constrain(verticalVelocity, -1.0, 1.0);
  horizontalVelocity = constrain(horizontalVelocity, -1.0, 1.0);

  // Standard for differential drive control
  double leftMotorVel = verticalVelocity - horizontalVelocity;
  double rightMotorVel = verticalVelocity + horizontalVelocity;

  // Converts the velocity requested into Revolutions / second
  const int maxVelocity = 5; // IGVC rules, should be meters/second units but not sure
  const double wheelCircumference = 2 * M_PI * 0.1524; // Circumference of the wheel in meters
  double leftMotorRPS= (leftMotorVel * maxVelocity) / wheelCircumference; // Turns/second
  double rightMotorRPS = (rightMotorVel * maxVelocity) / wheelCircumference; // Turns/second

  // Send velocity CAN commands to left and right motors
  CAN0.sendMsgBuf((ODRV0_NODE_ID << 5 | 0x0D), 0, 8, (byte*)&leftMotorRPS); // check 4
  CAN0.sendMsgBuf((ODRV1_NODE_ID << 5 | 0x0D), 0, 8, (byte*)&rightMotorRPS);
}

// Send CAN command to ODrive to initate EStop
void ODriveEStop()
{
  CAN0.sendMsgBuf((ODRV0_NODE_ID << 5 | 0x02), 0, 0, nullptr);
  CAN0.sendMsgBuf((ODRV1_NODE_ID << 5 | 0x02), 0, 0, nullptr);
  Serial.println("EStop Activated");
}

// Send CAN command to put axis into closed loop control state
void ODriveControlState() // Could combine with estop function but meh
{
  CAN0.sendMsgBuf((ODRV0_NODE_ID << 5 | 0x07), 0, 4, (byte*)"\x08");
  CAN0.sendMsgBuf((ODRV1_NODE_ID << 5 | 0x07), 0, 4, (byte*)"\x08");
  Serial.println("Put into closed loop control state");
}