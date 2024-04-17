// States
bool EStopped = false;
bool AutonMode = false;

// Inputs
int LEDPWR_R = 0;
int LEDPWR_G = 1;
int LEDPWR_B = 10;
int EStop = 5;
int Auton = 13;

// Outputs
int LED_R = 4;
int LED_G = 12;
int LED_B = 6;

// Variables for blinking
unsigned long previousMillis = 0;
const long interval = 200; // Blink interval in milliseconds
bool LEDState = false;

void setup() {
  pinMode(LEDPWR_R, INPUT);
  pinMode(LEDPWR_G, INPUT);
  pinMode(LEDPWR_B, INPUT);
  pinMode(EStop, INPUT);
  pinMode(Auton, INPUT);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  EStopped = true;
  AutonMode = false;
}

void loop() {
  EStopped = digitalRead(EStop);
  AutonMode = digitalRead(Auton);

  if (EStopped) { 
    EStopON();
  }
  else { 
    if (AutonMode) {
      BlinkBlueLED();
    }
    else {
      SolidGreenLED();
    }
  }
}

void EStopON() {
  digitalWrite(LED_B, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_R, HIGH); // Turn on the emergency stop LED
}

void BlinkBlueLED() {
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Toggle LED state
    LEDState = !LEDState;

    // Set LED accordingly
    digitalWrite(LED_B, LEDState ? HIGH : LOW);
  }
}

void SolidGreenLED() {
  // Turn on the green LED and turn off the blue LED
  digitalWrite(LED_B, LOW);
  digitalWrite(LED_G, HIGH);
  
}
