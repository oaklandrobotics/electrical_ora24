#include <SPI.h>
#include <mcp_can.h>

const int button_Pin = 4;  // Change the button pin to 7
const int LED_pin = 7;

int BUTTON_STATE = HIGH;
int lastBUTTON_STATE = HIGH;
long lastDebounceTime = 0;
long debounceDelay = 50;

#define CAN_INT 2 // Set INT to pin 2
MCP_CAN CAN(10);

unsigned long CAN_MSG_ID;
unsigned char CAN_MSG_LEN;
unsigned char CAN_MSG_DATA[1];

void setup() {
  pinMode(button_Pin, INPUT); // Set the button input pin
  pinMode(LED_pin, OUTPUT);
  Serial.begin(115200);

  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
  } else {
    Serial.println("Error Initializing MCP2515!");
    while (1);
  }
}

void loop() {
  int currentButtonState = FindButtonState();

  if (currentButtonState == LOW && lastBUTTON_STATE == HIGH) {
    // Defining CAN Message
    CAN.sendMsgBuf(0x001, 0, 1, (byte*)&currentButtonState);
    Serial.println("Sending CAN Message");
    
  }
  Serial.println("I made it here");
  CAN.sendMsgBuf(0x001, 0, 1);
  
  // Receive CAN Message
  if(!digitalRead(CAN_INT)) {
    CAN.readMsgBuf(&CAN_MSG_ID, &CAN_MSG_LEN, CAN_MSG_DATA);

    if (CAN_MSG_ID == 2 && CAN_MSG_LEN >= 1 && CAN_MSG_DATA[0] == 0x01) {
      digitalWrite(LED_pin, HIGH);
      Serial.println("Received CAN Message: ON");
    } else if (CAN_MSG_ID == 2 && CAN_MSG_LEN >= 1 && CAN_MSG_DATA[0] == 0x00) {
      digitalWrite(LED_pin, LOW);
      Serial.println("Received CAN Message: Off");
    }
    // Other message ID's go here
  }

  delay(100);
}

int FindButtonState() {
  int reading = digitalRead(button_Pin);

  if (reading != lastBUTTON_STATE) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != BUTTON_STATE) {
      BUTTON_STATE = reading;

      if (BUTTON_STATE == LOW) {
        Serial.println("Button is pressed - Toggle state!");
        // Perform the desired action when the button is pressed
        delay(1000);
      }
    }
  }

  lastBUTTON_STATE = reading;
  return lastBUTTON_STATE;
}
