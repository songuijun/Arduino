#include <MsTimer2.h>

const int pulsePin = 18;
const int signalPin = 1;

void Pulse()
{
  static boolean pulseState = LOW;
  digitalWrite(pulsePin, pulseState);
  pulseState = !pulseState;
}

void timer()
{
  static boolean outputState = LOW;

  if (outputState == LOW) 
  {
    digitalWrite(signalPin, HIGH);
    outputState = HIGH;
  } 
  else 
  {
    digitalWrite(signalPin, LOW);
    outputState = LOW;
  }
}

void setup()
{
  pinMode(pulsePin, OUTPUT);
  pinMode(signalPin, OUTPUT);

  MsTimer2::set(2, Pulse);
  MsTimer2::start();
  MsTimer2::set(1, timer);
  MsTimer2::start();
}

void loop()
{
  // Your loop code here, if needed
}
