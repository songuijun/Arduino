//주석
/*
  2023 첫 아두이노 테스트
 */

void setup()
{
  // put your setup code here, to run once:
  pinMode(3,OUTPUT); //3번핀을 출력으로 설정
  pinMode(4,INPUT); //4번핀을 입력으로 설정
}

void loop()
{
  // put your main code here, to run repeatedly:
  digitalWrite(3,HIGH); //3번핀 출력을 HIGH로
  delay(1000);          //1000msec
  digitalWrite(3,LOW);  //3번 핀 출력을 LOW로
  delay(1000);         //1000msec
}
