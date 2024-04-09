#include <PS4Controller.h>
#include <mcp_can.h>
#include <SPI.h>


#define CAN0_INT 2    // Set INT to pin 2 (This is the Interupt pin)
MCP_CAN CAN0(10);   // Set CS to pin 10 (This is the Chip select)

// ODrive & Can Definitions
#define CAN_BAUDRATE 500000
#define ODRV0_NODE_ID 0

// Allows us to refer to can_intf as "CAN"
MCP2515Class& can_intf = CAN;


// add definitions for LED's, IMU, etc.

double verticalMov = 0;
double horizontalMov = 0;
bool EStop = 1;


// Add more flags for initalization of CAN Bus (TX/RX buffer errors, etc.)
void setup()
{
  pinMode(CAN0_INT, INPUT);
  Serial.begin(115200);
  PS4.begin();
  Serial.println("Ready");

  // Add initialization for LED's, IMU, etc.

  // Continuously tries to establish connection to MCP chip until it does
  while(1)
  {
    // Tries to initalizes MCP CAN chip with clock of 8MHz and data transfer speed is 500KB/second
    if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    {
      // CAN BUS is initalized, then break out of while loop
      Serial.println("MCP2515 Initialized Successfully!");
      break;
    }
    else 
    {
      // Stays in infinite loop but waits 20ms to give time for bus to be initialized
      Serial.println("Error Initializing MCP2515...");
      delay(20);
    }
  }


  
}


// In this, take the inputs from each button / stick on the controller and assign it a variable (EStop, State, Movement, etc)
void loop()
{

  if(PS4.isConnected())
  {
    verticalMov = PS4.LStickY();
    horizontalMov = PS4.LStickX();
    EStop = PS4.R1();

  }
  else
  {
    verticalMov = 0;
    horizontalMov = 0;
  }

  if(EStop != 0) // check what output of function is when pressed 1 is just assumed.
  {
    // Stops all movement and sends out EStop command to ODrive
    verticalMov = 0;
    horizontalMov = 0;
    int test = 0;
    break;
  }
  else
  {
    // Set values of joysticks to velocity using CAN commands. 

    // Might need to see limit of velocity functions and constrain vertical and horizontal move if needed. 
  }
}
