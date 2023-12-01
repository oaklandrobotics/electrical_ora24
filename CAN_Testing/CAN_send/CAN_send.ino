#include <mcp_can.h>

#include <SPI.h>

MCP_CAN CAN0(10);  // Set CS to pin 10

void setup() {
  Serial.begin(115200);

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);
  Serial.println("MCP2515 Library Send Example...");
}

void loop() {
  // Check if there is serial data available
  if (Serial.available() > 0) {
    // Read the incoming byte from the Serial Monitor
    char serialInput = Serial.read();

    // Ignore newline characters
    if (serialInput == '\n') {
      return;  // Ignore newline characters
    }

    // Send a CAN message based on the serial input
    if (serialInput == '1' || serialInput == '0') {
      sendMessage(serialInput);
    } else {
      Serial.println("Invalid input. Please enter '1' or '0'.");
    }
  }

  // Add any other code that needs to run continuously here
}

void sendMessage(char state) {
  unsigned char len = 1;
  char msgString[128];

  // Message ID and format
  long unsigned int txId = 0x100;  // You can choose any suitable ID
  byte buf[1] = {state};

  // Send the message
  if (CAN0.sendMsgBuf(txId, 0, len, buf) == CAN_OK) {
    sprintf(msgString, "Message Sent: 0x%.3lX Data: 0x%.2X", txId, buf[0]);
    Serial.println(msgString);
  } else {
    Serial.println("Error Sending Message");
  }
}


  byte sndStat;
  
  // Sending '1'
  byte data1[8] = {'1', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sndStat = CAN0.sendMsgBuf(0x100, 0, 8, data1);
  if (sndStat == CAN_OK) {
    Serial.println("Message '1' Sent Successfully!");
  } else {
    Serial.println("Error Sending Message '1'...");
  }

  delay(1000); // Wait for 1 second

  // Sending '0'
  byte data0[8] = {'0', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sndStat = CAN0.sendMsgBuf(0x100, 0, 8, data0);
  if (sndStat == CAN_OK) {
    Serial.println("Message '0' Sent Successfully!");
  } else {
    Serial.println("Error Sending Message '0'...");
  }

  delay(1000); // Wait for 1 second
}