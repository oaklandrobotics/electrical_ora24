/* Pinouts:
--Internal--
Address PB1: SCK
Address PB2: MOSI
Address PB3: MISO
Address PB3/Digital 8: CS
Address PB5/Digital 9: INT

--External--
Analog0:
Analog1:
Analog2:
Analog3:
Analog4:

Digital 0: (RX)
Digital 1: (TX)
~Digital 5:
~Digital 10:
Digital 13:
*/

// Libraries Needed
#include "MCP2515.h"
#include "ODriveMCPCAN.hpp"
#include <SPI.h>

// ODrive & Can Definitions
#define CAN_BAUDRATE 500000
#define ODRV0_NODE_ID 0
MCP2515Class& can_intf = CAN; // Allows us to refer to can_intf as "CAN"

// MCP2515 Definitions
#define MCP2515_CS 8
#define MCP2515_INT 9
#define MCP2515_CLK_HZ 8000000

// Callback function to handle received CAN messages
static inline void receiveCallback(int packet_size) {
  if (packet_size > 8) {
    return;
  }
  CanMsg msg = {.id = (unsigned int)CAN.packetId(), .len = (uint8_t)packet_size};
  CAN.readBytes(msg.buffer, packet_size);
  onCanMessage(msg);
}

// Function to set up CAN communication
bool setupCan() {
  CAN.setPins(MCP2515_CS, MCP2515_INT);
  CAN.setClockFrequency(MCP2515_CLK_HZ);
  if (!CAN.begin(CAN_BAUDRATE)) {
    return false;
  }

  CAN.onReceive(receiveCallback);
  return true;
}

// Declaration of ODriveCAN instance and array of ODrive instances
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID);
ODriveCAN* odrives[] = {&odrv0};

// Struct to hold ODrive user data
struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

ODriveUserData odrv0_user_data;

// Callback function to handle received Heartbeat messages
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

// Function to handle incoming CAN messages
void onCanMessage(const CanMsg& msg) {
  for (auto odrive: odrives) {
    onReceive(msg, *odrive);
  }
}

// Setup function
void setup() {
  Serial.begin(115200);
  delay(200);

  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true); 
  }

  Serial.println("Waiting for ODrive...");
  while (!odrv0_user_data.received_heartbeat) {
    pumpEvents(can_intf);
    delay(100);
  }

  Serial.println("found ODrive");
}

// Loop function
void loop() {
  pumpEvents(can_intf);
}
