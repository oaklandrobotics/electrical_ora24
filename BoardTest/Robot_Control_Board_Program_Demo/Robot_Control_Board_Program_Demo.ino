// States
bool EStopped = false;
bool AutonMode = false;

// Inputs
int EStop = 0;
int Auton = 1;

// Outputs
int LED_R = 4;
int LED_G = 12;
int LED_B = 6;

void setup() 
{
  pinMode(EStop, INPUT);
  pinMode(Auton, INPUT);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
}

void loop() 
{
  EStopped = digitalRead(EStop);
  AutonMode = digitalRead(Auton);

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
      digitalWrite(LED_R, LOW);
      digitalWrite(LED_G, LOW);
      digitalWrite(LED_B, HIGH);
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
