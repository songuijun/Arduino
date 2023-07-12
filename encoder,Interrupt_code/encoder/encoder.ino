#define encoderPinA 2
#define encoderPinB 3

int encoderPos = 0;
int Bsang;
unsigned long prevTime = 0; // 이전 rising이 발생했던 시간
unsigned long thisTime = 0; // 현재 rising이 발생한 시간
unsigned long period = 0;   // 주기
/*핀 설정*/

void doEncoderInterrupt() {
 thisTime = millis();
 period = thisTime - prevTime;
 Bsang = digitalRead(encoderPinB);

  if (digitalRead(encoderPinB) == HIGH) {
    encoderPos--;
  } else {
    encoderPos++;
  }

  prevTime = thisTime;
}

void setup() {
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderInterrupt,RISING);
  Serial.begin(115200);
}
/*함수는 초기 설정을 수행*/
void loop() {
  Serial.print("A　 　　");
  Serial.println(1);
  Serial.print("B　 　　");
  Serial.println(Bsang);    //무엇을 추가 해볼까?
  Serial.print("주기　　");
  Serial.println(period);
  Serial.print("encoderPos  ");
  Serial.println(encoderPos);
}
/*함수는 계속해서 실행되는 루프*/
