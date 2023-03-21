#define TRIG1 2  // 초음파 센서 1번 Trig 핀 번호
#define ECHO1 3  // 초음파 센서 1번 Echo 핀 번호

#define TRIG2 4  // 초음파 센서 2번 Trig 핀 번호
#define ECHO2 5  // 초음파 센서 2번 Echo 핀 번호

#define TRIG3 6  // 초음파 센서 3번 Trig 핀 번호
#define ECHO3 7  // 초음파 센서 3번 Echo 핀 번호

#define ENA 8
#define IN1 9
#define IN2 10
#define IN3 11
#define IN4 12
#define ENB 13

void setup() 
{
  //put your setup code here, to run once:
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);

  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);

  pinMode(TRIG3, OUTPUT);
  pinMode(ECHO3, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
 
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);


  

  Serial.begin(115200); // 통신속도를 115200으로 정의함 
}

long sonar1(void)  //초음파 센서 1번 측정 함수
{
  long duration1, distance1; 
  digitalWrite(TRIG1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG1, LOW);
  duration1 = pulseIn(ECHO1, HIGH);
  distance1 = ( (float) (340*duration1)/1000)/2;
  return distance1;
}



long sonar2(void)  //초음파 센서 2번 측정 함수
{
  long duration2, distance2; 
  digitalWrite(TRIG2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG2, LOW);
  duration2 = pulseIn(ECHO2, HIGH);
  distance2 = ( (float) (340*duration2)/1000)/2;
  return distance2;
}

long sonar3(void)  //초음파 센서 3번 측정 함수
{
  long duration3, distance3; 
  digitalWrite(TRIG3, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG3, LOW);
  duration3 = pulseIn(ECHO3, HIGH);
  distance3 = ( (float) (340*duration3)/1000)/2;
  return distance3;
}

void motor_A_contro1(int direction_a, int motor_speed_a) 
{
   if(direction_a == HIGH)
   {
     digitalWrite(IN1, HIGH);               // 모터의 방향 제어
     digitalWrite(IN2, LOW);
     analogWrite(ENA, motor_speed_a);        // 모터의 속도 제어
   }
   else
   {
     digitalWrite(IN1, LOW);
     digitalWrite(IN2, HIGH);
     analogWrite(ENA, motor_speed_a);
   }

}

  
void motor_B_contro1(int direction_b, int motor_speed_b) 
{
   if(direction_b == HIGH)
   {
     digitalWrite(IN3, HIGH);            
     digitalWrite(IN4, LOW);
     analogWrite(ENB, motor_speed_b);       
   }
   else
   {
     digitalWrite(IN3, LOW);
     digitalWrite(IN4, HIGH);
     analogWrite(ENB, motor_speed_b);

   }

}



void loop()
{
  //put your main code here, to run repeatedly:
 

 motor_A_contro1(HIGH, 100);

 
  delay(500);
}
