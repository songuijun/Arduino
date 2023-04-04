#include <NewPing.h>

#define SONAR_NUM 3
#define MAX_DISTANCE 150

#define Front 0
#define Left 1
#define Right 2

#define TRIG1 2
#define ECHO1 3

#define TRIG2 4
#define ECHO2 5

#define TRIG3 6
#define ECHO3 7

NewPing sonar[SONAR_NUM] = {
  NewPing(TRIG1, ECHO1, MAX_DISTANCE),
  NewPing(TRIG2, ECHO2, MAX_DISTANCE),
  NewPing(TRIG3, ECHO3, MAX_DISTANCE)
};

#define ENA 8
#define IN1 9
#define IN2 10
#define IN3 11
#define IN4 12
#define ENB 13


void setup() {
  // put your setup code here, to run once:
  /*pinMode(TRIG1, OUTPUT);
    pinMode(ECHO1, INPUT);

    pinMode(TRIG2, OUTPUT);
    pinMode(ECHO2, INPUT);

    pinMode(TRIG3, OUTPUT);
    pinMode(ECHO3, INPUT);
  */
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  Serial.begin(115200);

}
long sonar_front(void) {
  long duration, distance;
  digitalWrite(TRIG1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG1, LOW);
  duration = pulseIn(ECHO1, HIGH);
  distance = ((float)(340 * duration) / 1000) / 2;
  return distance;
}
long sonar2(void) {
  long duration, distance;
  digitalWrite(TRIG2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG2, LOW);
  duration = ((float)(340 * duration) / 1000) / 2;
  return distance;
}
long sonar3(void) {
  long duration, distance;
  digitalWrite(TRIG3, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG3, LOW);
  duration = ((float)(340 * duration) / 1000) / 2;
  return distance;
}
void motor_A_control(int direction_a, int motor_speed_a) {
  if (direction_a == HIGH) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, motor_speed_a);
  }
  else
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, motor_speed_a);
  }
}
void motor_B_control(int direction_b, int motor_speed_b) {
  if (direction_b == HIGH) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, motor_speed_b);
  }
  else
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, motor_speed_b);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  float front_sonar   = 0.0;
  float left_sonar    = 0.0;
  float right_sonar   = 0.0;

  front_sonar = sonar[Front].ping_cm() * 10;
  if  (front_sonar == 0.0) front_sonar = MAX_DISTANCE;

    Serial.print("Distance1: ");
  Serial.print(front_sonar);
  //Serial.print("Distance2: ");
  //Serial.print(sonar2());
  //Serial.print("Distance3: ");
  //Serial.print(sonar3());
  
  if ((front_sonar > 0) && (front_sonar <= 250.0))
{
  Serial.println("Rotate stop");
    motor_A_control(HIGH, 0);
    motor_B_control(LOW , 0);
    delay(500);

    Serial.println("Rotate CCW");
    motor_A_control(HIGH, 200);
    motor_B_control(HIGH, 200);
    delay(1200);
  }
  else {
    Serial.println("Go Staight");
    motor_A_control(HIGH, 100);
    motor_B_control(LOW, 100);
  }


  delay(1000);
}
