#include <NewPing.h>

#define SONAR_NUM 3      // Number of sensors.
#define MAX_DISTANCE 150 // Maximum distance (in cm) to ping.
#define WALL_GAP_DISTANCE 400 //mm 단위
#define WALL_GAP_DISTANCE_HALF 200 //mm 단위
#define MOTOR_PWM_OFFSET 10
#define MOTOR_SPEED 70


#define Front 0
#define Left  1
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
  float front_sonar = 0.0;
  float left_sonar = 0.0;
  float right_sonar = 0.0;

#define ENA 8
#define IN1 9
#define IN2 10
#define IN3 11
#define IN4 12
#define ENB 13
int maze_status = 0;


void setup() 
{
   
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);

    Serial.begin(115200); //통신속도를 115200으로 정의함
}

void motor_A_control(int direction_a, int motor_speed_a) //모터 A의 방향(direction)과 속도(speed)제어
{
  if(direction_a == HIGH)
  {
      digitalWrite(IN1,HIGH);          // 모터의 방향 제어
      digitalWrite(IN2,LOW);           // 모터의 방향 제어
      analogWrite(ENA,motor_speed_a);  // 모터의 속도 제어
  }
  else
  {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA,motor_speed_a);
  }
}

void motor_B_control(int direction_b, int motor_speed_b) //모터 B의 방향(direction)과 속도(speed)제어
{
  if(direction_b == HIGH)
  {
      digitalWrite(IN3,LOW);          
      digitalWrite(IN4,HIGH);        
      analogWrite(ENB,motor_speed_b); 
  }
  else
  {
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB,motor_speed_b);
  }
}
void check_maze_status(void) {
  if ((left_sonar >= 0) && (left_sonar <= WALL_GAP_DISTANCE_HALF) && (right_sonar >= 0) && (right_sonar <= WALL_GAP_DISTANCE_HALF) && (front_sonar >= 0) && (front_sonar <= WALL_GAP_DISTANCE_HALF)) { // 세 면이 다 막힌 경우
    maze_status = 4;
    Serial.println("maze_status = 4");
  }
  else if ((left_sonar >= 0) && (left_sonar <= WALL_GAP_DISTANCE_HALF) && (right_sonar >= 0) && (right_sonar <= WALL_GAP_DISTANCE_HALF) && (front_sonar >= WALL_GAP_DISTANCE_HALF)) {
    maze_status = 1;
    Serial.println("maze_status = 1");
  }
  else if ((left_sonar >= 0) && (left_sonar <= WALL_GAP_DISTANCE_HALF) && (front_sonar >= 0) && (front_sonar <= WALL_GAP_DISTANCE_HALF)) {
    maze_status = 2;
    Serial.println("maze_status = 2");
  }
  else if ((right_sonar >= 0) && (right_sonar <= WALL_GAP_DISTANCE_HALF) && (front_sonar >= 0) && (front_sonar <= WALL_GAP_DISTANCE_HALF)) {
    maze_status = 3;
    Serial.println("maze_status = 3");
  }
  else {
    maze_status = 0;
    Serial.println("maze_status = 0");
  }
}
void wall_collision_avoid(int base_speed)
{
  float error = 0.0;
  float Kp = 0.3;      
  int pwm_control = 0; 
  int right_pwm = 0;  
  int left_pwm = 0; 
  
  error = (right_sonar - left_sonar); // 벽면 간의 거리
  error = Kp * error; // 스티어 양 조절 

  if (error >=  40) error =  40; // 과도한 스티어링 방지
  if (error <= -40) error = -40;

  right_pwm = MOTOR_SPEED - error;  // 오른쪽 바퀴 회전수 조절
  left_pwm = MOTOR_SPEED + error;   // 왼쪽 바퀴 회전수 조절

  if(right_pwm <= 0) right_pwm = 0;
  if(left_pwm  <= 0) left_pwm  = 0;

  if(right_pwm >= 255) right_pwm = 255;
  if(left_pwm  >= 255) left_pwm = 255;

  motor_A_control(HIGH,right_pwm); // 오른쪽 전진
  motor_B_control(HIGH,left_pwm);  // 왼쪽 전진

}

void loop() {
  // put your main code here, to run repeatedly:

  front_sonar = sonar[Front].ping_cm() * 10; //전방 센서 측정 
  left_sonar = sonar[Left].ping_cm() * 10;   //좌측 센서 측정
  right_sonar = sonar[Right].ping_cm() * 10; //우측 센서 측정
   
  if(front_sonar == 0.0)  front_sonar = MAX_DISTANCE;
  if(left_sonar == 0.0)   left_sonar = MAX_DISTANCE;
  if(right_sonar == 0.0)  right_sonar = MAX_DISTANCE;

  Serial.print("L: "); Serial.print(left_sonar); Serial.print(" ");
  Serial.print("F: "); Serial.print(front_sonar); Serial.print(" ");
  Serial.print("R: "); Serial.println(right_sonar);

  check_maze_status();
  if( maze_status == 4)
  { 
     motor_A_control(HIGH,0); //오른쪽 전진
    motor_B_control(LOW,0); //왼쪽은 후진
    delay(500);
   
    Serial.println("Rotate CCW");
    motor_A_control(HIGH,100); //오른쪽 전진
    motor_B_control(LOW,100); //왼쪽은 후진
    delay(1100);               //일정한 시간 동안 회전
 
    Serial.println("Rotate stop");
    motor_A_control(HIGH,0); //오른쪽 전진
    motor_B_control(LOW,0); //왼쪽은 후진
    delay(500);
  }
  else if( maze_status == 1)
  { // 좌우 벽만 있을 때 직전
    Serial.println("Go straight");
    wall_collision_avoid(MOTOR_SPEED);
    
  }
  else if( maze_status == 2)
  { 
     motor_A_control(HIGH,0); //오른쪽 전진
    motor_B_control(LOW,0); //왼쪽은 후진
    delay(500);
   
    Serial.println("Rotate CCW");
    motor_A_control(LOW,100); //오른쪽 전진
    motor_B_control(HIGH,100); //왼쪽은 후진
    delay(550);               //일정한 시간 동안 회전
 
   Serial.println("Rotate stop");
    motor_A_control(HIGH,0); //오른쪽 전진
    motor_B_control(LOW,0); //왼쪽은 후진
    delay(1000);
   
    Serial.println("Go Straight");
    motor_A_control(HIGH,50);
    motor_B_control(HIGH,50);
    delay(500);
    
  }
  else if( maze_status == 3)
  {
    motor_A_control(HIGH,0); //오른쪽 전진
    motor_B_control(LOW,0); //왼쪽은 후진
    delay(500);
   
    Serial.println("Rotate CCW");
    motor_A_control(HIGH,100); //오른쪽 전진
    motor_B_control(LOW,100); //왼쪽은 후진
    delay(550);               //일정한 시간 동안 회전
 
    Serial.println("Rotate stop");
    motor_A_control(HIGH,0); //오른쪽 전진
    motor_B_control(LOW,0); //왼쪽은 후진
    delay(500);
  }
  else 
  {
    // 직진
    Serial.println("Go straight");
    wall_collision_avoid(MOTOR_SPEED);
  }

}
