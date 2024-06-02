// States
bool EStopped = false;
bool AutonMode = false;

// Inputs
const int EStopPin = 0;
const int AutonPin = 1;

// Outputs
const int LED_R = 4;
const int LED_G = 12;
const int LED_B = 6;

// Blinking Declarations
unsigned long prevRunTime = 0;
const int blinkInterval = 200;
bool autonLEDState = false;
void AutonRedBlink();

void setup() 
{
  pinMode(EStopPin, INPUT);
  pinMode(AutonPin, INPUT);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  // Initialize LED states
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);
}

void loop() 
{
  EStopped = digitalRead(EStopPin);
  AutonMode = digitalRead(AutonPin);

  if (EStopped) 
  { 
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);
  }
  else 
  { 
    if (AutonMode) 
    {
      AutonRedBlink();
    }
    else 
    {
      digitalWrite(LED_R, LOW);
      digitalWrite(LED_G, HIGH);
      digitalWrite(LED_B, LOW);
    }
  }
  delay(10);
}

void AutonRedBlink()
{
  digitalWrite(LED_B, LOW);
  digitalWrite(LED_G, LOW);

  unsigned long currRunTime = millis();

  if(currRunTime - prevRunTime >= blinkInterval)
  {
    autonLEDState = !autonLEDState;

    digitalWrite(LED_R, autonLEDState ? HIGH : LOW);

    prevRunTime = currRunTime;
  }
}
