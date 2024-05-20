#include "MCP2515.h"
#include "ODriveMCPCAN.hpp"
#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>

#define CAN_BAUDRATE 500000
#define CAN0_INT 2 // Set INT to pin 2 (This is the Interrupt pin)
#define ODRV0_NODE_ID 0 // Left Motor
#define ODRV1_NODE_ID 1 // Right Motor

MCP2515Class& can_intf = CAN;

MCP_CAN CAN0(10);

#define MCP2515_CS 10
#define MCP2515_INT 2
#define MCP2515_CLK_HZ 8000000
#define EStop 0 // Hardware EStop
#define IMU_SDA 2 // On AtMega32U4 Connected to Pin 19 (PD1)
#define IMU_SCL 3 // On AtMega32U4 Connected to Pin 18 (PD0) // CHECK

/*
// LED Declarations
#define LEDPWR_R 4 // On AtMega32U4 Connected to Pin 25 (PD4)
#define LEDPWR_G 12 // On AtMega32U4 Connected to Pin 26 (PD6)
#define LEDPWR_B 6 // On AtMega32U4 Connected to Pin 27 (PD7)
*/

// Robot State Declarations
#define EStopButtonIndicator 4 // On AtMega32U4 Connected to Pin 25 (PD4)
#define AutonButtonIndicator 5 // On AtMega32U4 Connected to Pin 26 (PD6)

// Battery Voltage Detection Declaration
#define Battery_Voltage A0 // On AtMega32U4 Connected to Pin 27 (PF0)


#define Wheel_Diameter 8

// Global Variable Declaration

// Controller Variables
int verticalMov = 0;
int horizontalMov = 0;
bool EStopButton = false;
bool EStopState = false;
bool prevEStopButton = false;
bool AutonButton = false;
bool AutonState = false;
bool prevAutonButton = false;

// IDs of the messages we are interested in
const unsigned long rightWheelSpeedID = 2;
const unsigned long leftWheelSpeedID = 3;

// Buffers to store incoming CAN data
byte msgWheelSpeed[8];

// Variables to store wheel speeds
double rightWheelSpeed;
double leftWheelSpeed;


// Function Prototypes
static inline void receiveCallback(int);
bool setupCan();
void onHeartbeat(Heartbeat_msg_t&, void*);
void onFeedback(Get_Encoder_Estimates_msg_t&, void*);
void onCanMessage(const &);

// Data type created for ODrive (Consists of last heartbeat, if heartbeat was received, encoder feedback, and if there is feedback)
struct ODriveUserData 
{
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

// Initialize ODrive objects
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID); // Initalize odrv0 as ODriveCAN object with respective node ID
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV1_NODE_ID); // Initalize odrv1 as ODriveCAN object with respective node ID
ODriveCAN* odrives[] = {&odrv0, &odrv1}; // Make sure all ODriveCAN instances are accounted for here

// Initalizes variables to hold ODrive configuation data
ODriveUserData odrv0_user_data;
ODriveUserData odrv1_user_data;


void setup() 
{
  Serial.begin(115200);

  Wire.begin(8);

  pinMode(EStop, INPUT);

  pinMode(CAN0_INT, INPUT);

  pinMode(IMU_SCL, INPUT); // Work on later
  pinMode(IMU_SDA, INPUT); // Work on later

  pinMode(EStopButtonIndicator, OUTPUT);
  pinMode(AutonButtonIndicator, OUTPUT);

  pinMode(Battery_Voltage, INPUT);


  // Wait for up to 3 seconds for the serial port to be opened on the PC side
  for (int i = 0; i < 30 && !Serial; ++i) 
  {
    delay(100);
  }
  delay(200);

  // Register callbacks for the heartbeat and encoder feedback messages
  odrv0.onFeedback(onFeedback, &odrv0_user_data);
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);

  odrv1.onFeedback(onFeedback, &odrv1_user_data);
  odrv1.onStatus(onHeartbeat, &odrv1_user_data);


  if (!setupCan()) 
  {
    Serial.println("CAN failed to initialize: reset required");
    while (true);
  }

  Serial.println("Waiting for ODrive...");
  while (!odrv0_user_data.received_heartbeat || !odrv1_user_data.received_heartbeat) 
  {
    pumpEvents(can_intf);
    delay(100);
  }

  Serial.println("ODrive Detected");

  // request bus voltage and current (1sec timeout)
  Serial.println("Attempting to read bus voltage and current");
  Get_Bus_Voltage_Current_msg_t vbus;
  if (!odrv0.request(vbus, 1)) 
  {
    Serial.println("vbus request failed!");
    while (true);
  }
  else if (!odrv1.request(vbus, 1))
  {
    Serial.println("vbus request failed!");
    while (true);
  }

  Serial.print("DC voltage [V]: ");
  Serial.println(vbus.Bus_Voltage);
  Serial.print("DC current [A]: ");
  Serial.println(vbus.Bus_Current);

  ODriveControlState();
  Serial.println("ODrive Setup Completed: ");
}

void loop() 
{
  // This is required on some platforms to handle incoming feedback CAN messages
  pumpEvents(can_intf);

  // Fetches values from controller that are sent over I2C
  fetchControllerData();

  // Falling Edge Detection (Goes from High to Low)
  // EStop (If statement) only activates when button is pressed
  if (prevEStopButton == HIGH && EStopButton == LOW)
  {
    // Toggles EStop State
    if(!EStopState)
    {
      ODriveEStop();
      EStopState = true;
      digitalWrite(EStopButtonIndicator, HIGH);
    }

    else
    {
      ODriveControlState();
      ODriveMovement(verticalMov, horizontalMov);
      EStopState = false;
      digitalWrite(EStopButtonIndicator, LOW);
    }
  } 

  prevEStopButton = EStopButton;

  if(!EStopState)
  {
    if (AutonButton == LOW && prevAutonButton == HIGH) // I may have to change what ID the ODrives are listening to?
    {
      if(!AutonState)
      {
        autonMovementData(); // Receives data from path planner and send motor speeds to ODrives
        AutonState = true;
        digitalWrite(AutonButtonIndicator, HIGH);
      }
      
      else 
      {
        AutonState = false;
        digitalWrite(AutonButtonIndicator, LOW);
      }
    }
  }

  prevAutonButton = AutonButton;

  // Prints ODrive Velocities and Position via Encoders
  //ODriveEncoderData();

  delay(10);
}

void fetchControllerData()
{
  // Requests data from slave device 8 (ESP32)
  Wire.requestFrom(8, sizeof(int) + sizeof(int) + sizeof(bool) + sizeof(bool));

  // Checks to see if data is recieved over I2C. If so sets values from controller to predefined variables
  if(Wire.available() >= sizeof(int) + sizeof(int) + sizeof(bool) + sizeof(bool))
  {
    // Read vertical movement from left joystick
    verticalMov = Wire.read();
    verticalMov |= Wire.read() << 8; // Combines with the next byte

    // Read horizontal movement from left joystick
    horizontalMov = Wire.read();
    horizontalMov |= Wire.read() << 8; // Combines with the next byte

    // Read EStopButton from right bumper
    EStopButton = Wire.read();

    // Read Robot State from left bumper
    AutonButton = Wire.read();
    
    // Reads controller values from ESP-32
    // ViewControllerData()
  } 
  else
  {
    verticalMov = 0;
    horizontalMov = 0;
    EStopState = false;
    AutonState = false;
    Serial.println("Insufficient bytes received");
  }
}

void ODriveMovement(double verticalVelocity, double horizontalVelocity) // Confirm if working
{
  // Scales inputs to directional velocity vectors
  verticalVelocity = constrain(verticalVelocity, -1.0, 1.0);
  horizontalVelocity = constrain(horizontalVelocity, -1.0, 1.0);

  // Standard for differential drive control
  double leftMotorVel = verticalVelocity - horizontalVelocity;
  double rightMotorVel = verticalVelocity + horizontalVelocity;

  // Gearbox ratio
  const double gearboxRatio = 21.0;

  // Input RPM
  const double inputRPM = 4400.0;

  // Converts the velocity requested into turns/second
  const double maxVelocity = 5.0; //  Speed limit m/s according to IGVC rules
  double leftMotorTPS = leftMotorVel * maxVelocity; // m/s
  double rightMotorTPS = rightMotorVel * maxVelocity; // m/s

  // Adjust for gearbox ratio and input RPM
  leftMotorTPS = (leftMotorTPS / gearboxRatio) * (inputRPM / 60.0); // turns/second
  rightMotorTPS = (rightMotorTPS / gearboxRatio) * (inputRPM / 60.0); // turns/second

  // Send velocity CAN commands to left and right motors
  odrv0.setVelocity(leftMotorTPS);
  odrv1.setVelocity(rightMotorTPS);
}

// Sends command over CAN to ODrive to initiate EStop
void ODriveEStop()
{
  Serial.println("Enabling EStop...");
  while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_IDLE || odrv1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_IDLE) 
  {
    odrv0.clearErrors();
    delay(1);
    odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);

    odrv1.clearErrors();
    delay(1);
    odrv1.setState(ODriveAxisState::AXIS_STATE_IDLE);

    for (int i = 0; i < 15; ++i) 
    {
      delay(10);
      pumpEvents(can_intf); // Pump events for 150ms (Look at example for full explanation)
    }
  }
  Serial.println("EStop Activated");
}

// Send CAN command to put axis into closed loop control state
void ODriveControlState() // Could combine with estop function (Possibly make function have an input to dictate control state or EStop?)
{
  Serial.println("Enabling closed loop control...");
  while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL || odrv1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) 
  {
    odrv0.clearErrors();
    delay(1);
    odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    odrv1.clearErrors();
    delay(1);
    odrv1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    for (int i = 0; i < 15; ++i) 
    {
      delay(10);
      pumpEvents(can_intf);  // Pump events for 150ms (Look at example for full explanation)
    }
  }
  Serial.println("Put into closed loop control state");
}

// Prints position and velocity from each ODrive (For debugging)
void ODriveEncoderData()
{
  if (odrv0_user_data.received_feedback) 
  {
    Get_Encoder_Estimates_msg_t feedback = odrv0_user_data.last_feedback;
    odrv0_user_data.received_feedback = false;
    Serial.print("ODrive 0 Velocity:");
    Serial.println(feedback.Vel_Estimate);
    Serial.println("------------------------");
  }

  if (odrv1_user_data.received_feedback) 
  {
    Get_Encoder_Estimates_msg_t feedback = odrv1_user_data.last_feedback;
    odrv1_user_data.received_feedback = false;
    Serial.print("ODrive 1 Velocity:");
    Serial.println(feedback.Vel_Estimate);
    Serial.println("------------------------");
  }
}

// Prints received values of controller from ESP-32 (For debugging)
void ViewControllerData()
{
  Serial.print("Received from ESP32 - Vertical Movement: ");
  Serial.println(verticalMov);

  Serial.print("Received from ESP32 - Horizontal Movement: ");
  Serial.println(horizontalMov);

  Serial.print("Received from ESP32 - EStop Button: ");
  Serial.println(EStopButton);

  Serial.print("Received from ESP32 - Autonomous Button: ");
  Serial.println(AutonButton);

  Serial.println("------------------------");
}

void autonMovementData() // Assuming data is given in m/s
{
  unsigned long receivedID;
  byte msgLen;

  // Check if a CAN message is available
  if (CAN_MSGAVAIL == CAN0.checkReceive()) 
  {
    // Read the CAN message
    CAN0.readMsgBuf(&receivedID, &msgLen, msgWheelSpeed);
    
    // Check if the received message ID matches the right wheel speed ID
    if (receivedID == rightWheelSpeedID) 
    {
      // Check to see if lenth is 8 bytes (Double)
      if (msgLen == 8) 
      {
          // Convert byte array to double
          memcpy(&rightWheelSpeed, msgWheelSpeed, sizeof(double));
          
          /*
          // Print the received right wheel speed
          Serial.print("Received right wheel speed: ");
          Serial.println(rightWheelSpeed, 8); // Print with 8 decimal places
          */
      } 
      else 
      {
        Serial.println("Received message length does not match expected double size.");
      }
    }

    // Read the CAN message again for the left wheel speed
    CAN0.readMsgBuf(&receivedID, &msgLen, msgWheelSpeed);

    // Check if the received message ID matches the left wheel speed ID
    if (receivedID == leftWheelSpeedID) 
    {
      // Check to see if lenth is 8 bytes (Double)
      if (msgLen == 8) 
      {
        // Convert byte array to double
        memcpy(&leftWheelSpeed, msgWheelSpeed, sizeof(double));
        
        /*
        // Print the received left wheel speed
        Serial.print("Received left wheel speed: ");
        Serial.println(leftWheelSpeed, 8); // Print with 8 decimal places
        */
      }

      else
      {
        Serial.println("Received message length does not match expected double size.");
      }
    } 
  }

  rightWheelSpeed = rightWheelSpeed / ((M_PI) * Wheel_Diameter);
  leftWheelSpeed = leftWheelSpeed / ((M_PI) * Wheel_Diameter);

  odrv0.setVelocity(leftWheelSpeed);
  odrv1.setVelocity(rightWheelSpeed);

}

// Below are all of the ODriveCAN Specific function declarations

static inline void receiveCallback(int packet_size)
{
  if (packet_size > 8) 
  {
    return;
  }
  CanMsg msg = {.id = (unsigned int)CAN.packetId(), .len = (uint8_t)packet_size};
  CAN.readBytes(msg.buffer, packet_size);
  onCanMessage(msg);
}

// Configure and initialize the CAN bus interface
bool setupCan() 
{
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
  {
    Serial.println("MCP2515 Initialized Successfully!");
  }
  else
  {
    Serial.println("Error Initializing MCP2515...");
  }
  
  CAN0.setMode(MCP_NORMAL);


  CAN.setPins(MCP2515_CS, MCP2515_INT);
  CAN.setClockFrequency(MCP2515_CLK_HZ);
  if (!CAN.begin(CAN_BAUDRATE))
  {
    return false;
  }

  CAN.onReceive(receiveCallback);
  return true;
}

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) 
{
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) 
{
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}

// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg& msg) 
{
  for (auto odrive: odrives) 
  {
    onReceive(msg, *odrive);
  }
}