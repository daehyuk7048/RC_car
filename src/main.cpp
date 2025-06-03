#include <Arduino.h>
#include <PinChangeInterrupt.h>

// 중립값 정의
#define NEUTRAL_THROTTLE_CH1 1508
#define NEUTRAL_THROTTLE_CH2 1472
#define NEUTRAL_SWITCH_CH5 1400

// 핀 매핑
#define CH1 A0
#define CH2 A1
#define CH5 A2
#define MOTOR 9
#define DIRECTION 10

// 글로벌 변수
volatile unsigned long startCH1, startCH2, startCH5;
volatile int pwmCH1 = NEUTRAL_THROTTLE_CH1;
volatile int pwmCH2 = NEUTRAL_THROTTLE_CH2;
volatile int pwmCH5 = NEUTRAL_SWITCH_CH5;
volatile bool newCH1 = false, newCH2 = false, newCH5 = false;

// 현재 모드: false = 수동(조종기), true = 자동(라인트레이싱)
bool isAutoMode = false;

// 인터럽트 핸들러
void changeDir() {
  if (digitalRead(CH1)) startCH1 = micros();
  else {
    pwmCH1 = micros() - startCH1;
    newCH1 = true;
  }
}

void changeSpeed() {
  if (digitalRead(CH2)) startCH2 = micros();
  else {
    pwmCH2 = micros() - startCH2;
    newCH2 = true;
  }
}

void changeMode() {
  if (digitalRead(CH5)) startCH5 = micros();
  else {
    pwmCH5 = micros() - startCH5;
    newCH5 = true;

    // 수신한 PWM 기준으로 모드 전환
    if (pwmCH5 < 1400) {
      isAutoMode = false;  // 조종기 수동 모드
    } else {
      isAutoMode = true;   // 라인트레이싱 모드
    }
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(CH1, INPUT_PULLUP);
  pinMode(CH2, INPUT_PULLUP);
  pinMode(CH5, INPUT_PULLUP);

  pinMode(MOTOR, OUTPUT);
  pinMode(DIRECTION, OUTPUT);

  attachPCINT(digitalPinToPCINT(CH1), changeDir, CHANGE);
  attachPCINT(digitalPinToPCINT(CH2), changeSpeed, CHANGE);
  attachPCINT(digitalPinToPCINT(CH5), changeMode, CHANGE);
}

void loop() {
  if (!isAutoMode) {
    // 수동 모드: RC 조종기로 조정
    int dir = map(pwmCH1, 1064, 1932, 90, 270);
    int speed = map(pwmCH2, 1068, 1896, 181, 198);

    analogWrite(MOTOR, speed);
    analogWrite(DIRECTION, dir);

    Serial.print("🕹️ Manual - Speed: ");
    Serial.print(speed);
    Serial.print(", Direction: ");
    Serial.println(dir);
  } else {
    // 자동 모드: 라즈베리파이에서 시리얼 명령 수신 대기
    if (Serial.available() >= 2) {
      char header = Serial.read();
      char command = Serial.read();

      if (header == 'L') {
        if (command == 'F') {
          analogWrite(MOTOR, 198);  // 직진
        } else if (command == 'L') {
          analogWrite(MOTOR, 195);
          analogWrite(DIRECTION, 90);  // 좌회전
        } else if (command == 'R') {
          analogWrite(MOTOR, 195);
          analogWrite(DIRECTION, 270);  // 우회전
        } else if (command == 'S') {
          analogWrite(MOTOR, 185);  // 정지
        } else if (command == 'B') {
          analogWrite(MOTOR, 167)   // 뒤로
        }
      }
    }
  }

  delay(20);
}
