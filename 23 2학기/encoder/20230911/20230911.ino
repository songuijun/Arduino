#include <MsTimer2.h>

#define RAD2DEG(x) (x*180.0/3.14159)
#define DEG2RAD(x) (x*3.14159/180.0)
#define WHEEL_TRACK 0.1 // M단위, 0.1이면 10cm

const byte outPin = 13; // Output pin: digital pin 13 (D13)
const byte interruptPin1 = 3; // Interrupt pin: digital pin 3 (D3)
const byte interruptPin2 = 18; // Interrupt pin: digital pin 18 (D18)
const byte encoder1_A = 2; // Interrupt pin: digital pin 2 (D2)
const byte encoder1_B = 3; // Interrupt pin: digital pin 3 (D3)
const byte encoder2_A = 18; // Interrupt pin: digital pin 18 (D18)
const byte encoder2_B = 19; // Interrupt pin: digital pin 19 (D19)

long cnt1 = 0;
long cnt2 = 0;

const byte resetPin = 5;
volatile byte staFte = 0;
long cnt1_old = 0;
long cnt2_old = 0;
const double pulse_to_distance_left = 0.2 / 512;
const double pulse_to_distance_right = 0.2 / 961;

double odom_left_delta = 0;
double odom_right_delta = 0;

double heading(double x, double y)
{
  double head = atan2(x, y);
  return head;
}

void MsTimer2_ISR()
{
  odom_left_delta = (cnt1 - cnt1_old) * pulse_to_distance_left;
  odom_right_delta = (cnt2 - cnt2_old) * pulse_to_distance_right;

  double theta_delta = heading((odom_right_delta - odom_left_delta), WHEEL_TRACK);
  
  Serial.print(odom_left_delta);
  Serial.print("  ");
  Serial.print(odom_right_delta);
  Serial.println("  ");
  Serial.print(DEG2RAD(theta_delta));
  Serial.println(theta_delta);
  Serial.println("   ");
  cnt1_old = cnt1;
  cnt2_old = cnt2;
}

void setup()
{
  pinMode(outPin, OUTPUT); // Output mode
  pinMode(encoder1_A, INPUT_PULLUP);
  pinMode(encoder1_B, INPUT_PULLUP);
  pinMode(encoder2_A, INPUT_PULLUP);
  pinMode(encoder2_B, INPUT_PULLUP);
  pinMode(resetPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder1_A), intfunc1, RISING); // Enable interrupt
  attachInterrupt(digitalPinToInterrupt(encoder2_A), intfunc2, RISING); // Enable interrupt

  Serial.begin(115200);
}

void intfunc1()
{
  if (digitalRead(encoder1_A) == digitalRead(encoder1_B))
  {
    cnt1_old--;
  }
  else
  {
    cnt1_old++;
  }
}

void intfunc2()
{
  if (digitalRead(encoder2_B) == LOW)
  {
    cnt2_old--;
  }
  else
  {
    cnt2_old++;
  }
}

void loop()
{
  const double odom_left = cnt1_old * pulse_to_distance_left;
  const double odom_right = cnt2_old * pulse_to_distance_right;
  Serial.print("cnt1_old: ");
  Serial.print(cnt1_old);
  Serial.print("   cnt2_old: ");
  Serial.println(cnt2_old);
  Serial.print(odom_left);
  Serial.print("  ");
  Serial.println(odom_right);
}
