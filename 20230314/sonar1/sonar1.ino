#define TRIG1 3  //  1번 초음파 센서 Trig 핀 번호
#define ECHO1 4  //  1번 초음파 센서 Echo 핀 번호

void setup() {
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  Serial.begin(115200); // 통신 속도를 115200으로 정의
}

long sonar1(void) {
  long duration, distance;

  digitalWrite(TRIG1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG1, LOW);

  duration = pulseIn(ECHO1, HIGH);
  distance = ((float)(340 * duration) / 1000) / 2;
  return distance;
}

void loop() {
  //Serial.print("Duration: ");
  //Serial.println(sonar1());
  Serial.print("Distance: ");
  Serial.println(sonar1());
  //Serial.println("mm\n");
  delay(500);
}
