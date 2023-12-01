#include <mcp_can.h>
#include <SPI.h>

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string

#define CAN0_INT 2                              // Set INT to pin 2
MCP_CAN CAN0(10);                               // Set CS to pin 10

const int ledPin = 7; 

void setup() {
  Serial.begin(115200);
  
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");
  
  CAN0.setMode(MCP_NORMAL);

  pinMode(CAN0_INT, INPUT);
  pinMode(ledPin, OUTPUT);
  
  Serial.println("MCP2515 Library Receive and Send Example...");
}

void loop() {
  if(!digitalRead(CAN0_INT)) {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);

    if((rxId & 0x80000000) == 0x80000000)
      sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
    else
      sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);

    Serial.print(msgString);

    if((rxId & 0x40000000) == 0x40000000){
      sprintf(msgString, " REMOTE REQUEST FRAME");
      Serial.print(msgString);
    } else {
      for(byte i = 0; i < len; i++){
        sprintf(msgString, " 0x%.2X", rxBuf[i]);
        Serial.print(msgString);
      }

      // Control the LED based on the received message
      if (len == 1) {
        if (rxBuf[0] == '1') {
          digitalWrite(ledPin, HIGH);
          Serial.println(" - LED ON");
          sendDataBack('A');  // Send a response back
        } else if (rxBuf[0] == '0') {
          digitalWrite(ledPin, LOW);
          Serial.println(" - LED OFF");
          sendDataBack('B');  // Send a different response back
        } else {
          Serial.println(" - Unknown Data");
          sendDataBack('C');  // Send another response for unknown data
        }
      } else {
        Serial.println(" - Invalid Data Length");
        sendDataBack('D');  // Send a response for invalid data length
      }
    }
    
    Serial.println();
  }
}

void sendDataBack(char response) {
  unsigned char len = 1;
  char msgString[128];

  // Message ID and format for response
  long unsigned int txId = 0x456;  // Choose a different ID for response
  byte buf[8] = {response};

  // Send the response message
  if (CAN0.sendMsgBuf(txId, 0, len, buf) == CAN_OK) {
    sprintf(msgString, "Response Sent: 0x%.3lX Data: 0x%.2X", txId, buf[0]);
    Serial.println(msgString);
  } else {
    Serial.println("Error Sending Response");
  }
}
