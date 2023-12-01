#include <mcp_can.h>
#include <SPI.h>

MCP_CAN CAN0(10);  // Set CS to pin 10


void setup() {
  Serial.begin(115200);

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);
  Serial.println("MCP2515 Library Send Example...");
}

void loop() {


  //Send a message to enable the LED
  sendMessage('1');
  
  delay(3000);  // Wait for 1 second

  // Send a message to disable the LED
  sendMessage('0');
  delay(3000);  // Wait for 1 second
}

void sendMessage(char state) {
  unsigned char len = 1;
  char msgString[128];

  // Message ID and format
  long unsigned int txId = 0x123;  // You can choose any suitable ID
  byte buf[8] = {state};

  // Send the message
  if (CAN0.sendMsgBuf(txId, 0, len, buf) == CAN_OK) {
    sprintf(msgString, "Message Sent: 0x%.3lX Data: 0x%.2X", txId, buf[0]);
    Serial.println(msgString);
  } else {
    Serial.println("Error Sending Message");
  }
}
