// ────────────────────────────────────────────────
//  라이브러리 포함
#include <Arduino.h>
#include <PinChangeInterrupt.h>  // PCINT 인터럽트용 라이브러리

// ────────────────────────────────────────────────
//  중립값 정의 (조종기에서 입력 신호가 아무 입력도 없을 때의 PWM 값)
#define NEUTRAL_THROTTLE_CH1 1508  // 방향 채널 중립값
#define NEUTRAL_THROTTLE_CH2 1472  // 속도 채널 중립값

// ────────────────────────────────────────────────
//  핀 매핑
#define CH1 A0       // 채널 1: 방향 신호 입력 핀
#define CH2 A1       // 채널 2: 속도 신호 입력 핀
#define MOTOR 9      // 모터 속도 제어 핀 (PWM 출력)
#define DIRECTION 10 // 모터 방향 제어 핀 (PWM 출력 또는 서보 제어용)

// ────────────────────────────────────────────────
//  글로벌 변수 (PWM 신호 해석용)
volatile unsigned long startCH1, startCH2;  // 펄스 시작 시간 기록용
volatile int pwmCH1 = NEUTRAL_THROTTLE_CH1; // CH1 측정된 PWM값
volatile int pwmCH2 = NEUTRAL_THROTTLE_CH2; // CH2 측정된 PWM값
volatile bool newCH1 = false, newCH2 = false; // 새 신호 수신 여부 플래그

// ────────────────────────────────────────────────
//  인터럽트 핸들러: 방향 채널 (CH1)
void changeDir() {
  if (digitalRead(CH1))
    startCH1 = micros();  // HIGH: 펄스 시작 시간 기록
  else {
    pwmCH1 = micros() - startCH1;  // LOW: 펄스 폭 계산
    newCH1 = true;                 // 새 데이터 플래그
  }
}

//  인터럽트 핸들러: 속도 채널 (CH2)
void changeSpeed() {
  if (digitalRead(CH2))
    startCH2 = micros();  // HIGH: 펄스 시작 시간 기록
  else {
    pwmCH2 = micros() - startCH2;  // LOW: 펄스 폭 계산
    newCH2 = true;
  }
}

// ────────────────────────────────────────────────
//  초기화 함수
void setup() {
  Serial.begin(9600);  // 시리얼 통신 시작 (디버깅용)

  // 입력 핀 설정
  pinMode(CH1, INPUT_PULLUP);
  pinMode(CH2, INPUT_PULLUP);

  // 출력 핀 설정
  pinMode(MOTOR, OUTPUT);
  pinMode(DIRECTION, OUTPUT);

  // 핀 변경 인터럽트 설정
  attachPCINT(digitalPinToPCINT(CH1), changeDir, CHANGE);     // CH1 변경 감지
  attachPCINT(digitalPinToPCINT(CH2), changeSpeed, CHANGE);   // CH2 변경 감지
}

// ────────────────────────────────────────────────
//  메인 루프
void loop() {
  // 방향과 속도를 PWM 값으로부터 변환
  int dir = map(pwmCH1, 1064, 1932, 90, 270);       // 방향값을 90~270도로 매핑
  int speed = map(pwmCH2, 1068, 1896, 181, 198);    // 속도값을 181~198으로 매핑 (이 범위는 매우 좁음)

  analogWrite(MOTOR, speed);       // 모터 속도 제어
  analogWrite(DIRECTION, dir);     // 방향 제어 (PWM 기반 서보일 경우만 유효)

  // 디버깅 출력
  Serial.print("Speed: ");
  Serial.print(pwmCH2);
  Serial.print(", Direction: ");
  Serial.println(dir);

  // 새 데이터 수신 플래그 초기화
  newCH1 = newCH2 = false;
}
