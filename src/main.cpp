/*─────────────────────────────────────────────────────────────
 *  RC Car – Manual / Auto (Arduino UNO  +  ESC + Servo)
 *  -----------------------------------------------------------
 *  RC 수신 PWM
 *    └ CH1 : A0  ─ Steer
 *    └ CH2 : A1  ─ Throttle
 *    └ CH5 : A2  ─ 3-way Switch (Manual / Auto)
 *
 *  출력 핀
 *    └ 10 : PWM → Steering  Servo
 *    └  9 : PWM → ESC  (브러시리스/브러시드 모두)
 *    └  4 : 좌측 방향 LED
 *    └  5 : 우측 방향 LED
 *
 *  자동 모드 프로토콜
 *    “P  <steerByte>  <throttleByte>”  (총 3 Byte)
 *      • steerByte : 0(최좌) … 128(중립) … 255(최우)
 *      • throttle  : 0(강후진) … 127/128(정지) … 255(강전진)
 *─────────────────────────────────────────────────────────────*/

#include <Arduino.h>
#include <Servo.h>
#include <PinChangeInterrupt.h>


/* ───────── 핀 매핑 ──────────────────────────────────────────*/
#define CH1         A0      // RC Steer
#define CH2         A1      // RC Throttle
#define CH5         A2      // RC Mode-Switch
#define PIN_SERVO   10      // Steering Servo (PWM)
#define PIN_ESC     9       // ESC  (PWM)
#define LED_L       4
#define LED_R       5


/* ───────── RC PWM 범위 (µs) ────────────────────────────────*/
constexpr int CH1_MIN = 1060, CH1_MAX = 1930;      // 조향
constexpr int CH2_MIN = 1060, CH2_MAX = 1890;      // 스로틀
constexpr int CH_CENTER = 1500;                    // RC 중립
constexpr int CH5_AUTO_THRESHOLD = 1400;           // ≥1400 → Auto Mode


/* ───────── 서보(조향) 매핑 & 보정 ──────────────────────────*/
constexpr int SERVO_MIN_DEG =  40;   // 완전 좌
constexpr int SERVO_MAX_DEG = 140;   // 완전 우
int SERVO_CENTER_OFFSET = -10;       // 중립 오프셋(왼쪽 쏠림 교정, –값=오른쪽으로)

/* ───────── ESC 펄스폭 (↓ 매우 낮은 속도 범위) ──────────────
 * 1450 STOP 후진은 1350까지 전진은 1510까지   */
constexpr int ESC_STOP        = 1450;   // 정지
constexpr int ESC_REV_STRONG  = 1350;   // 강한 후진
constexpr int ESC_REV_WEAK    = 1400;   // 약   후진
constexpr int ESC_FWD_WEAK    = 1470;   // 약   전진
constexpr int ESC_FWD_STRONG  = 1510;   // 강한 전진

constexpr int THR_DEADBAND = 20;        // ±20 µs 를 “중립”으로 간주


/* ───────── 글로벌 변수 ────────────────────────────────────*/
volatile int pwmCH1 = CH_CENTER;   // 실시간 RC PWM 폭
volatile int pwmCH2 = CH_CENTER;
volatile int pwmCH5 = 1000;
volatile unsigned long tCH1, tCH2, tCH5;

bool   autoMode = false;           // true → 자동(Pi 제어)
Servo  steer, esc;                 // Servo 객체


/* ─────────― 인터럽트 : PWM 폭 측정 ―─────────*/
void pcintCH1() {                       // 채널 1 (Steer)
  if (digitalRead(CH1)) tCH1 = micros();
  else                  pwmCH1 = micros() - tCH1;
}
void pcintCH2() {                       // 채널 2 (Throttle)
  if (digitalRead(CH2)) tCH2 = micros();
  else                  pwmCH2 = micros() - tCH2;
}
void pcintCH5() {                       // 채널 5 (모드 전환)
  if (digitalRead(CH5)) tCH5 = micros();
  else {
    pwmCH5 = micros() - tCH5;
    autoMode = (pwmCH5 >= CH5_AUTO_THRESHOLD);
  }
}


/* ───────── 초기화 ───────────────────────────*/
void setup() {
  Serial.begin(115200);

  pinMode(LED_L, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(CH1, INPUT_PULLUP);
  pinMode(CH2, INPUT_PULLUP);
  pinMode(CH5, INPUT_PULLUP);

  steer.attach(PIN_SERVO);
  esc.attach(PIN_ESC);
  esc.writeMicroseconds(ESC_STOP);       // 확실히 정지

  attachPCINT(digitalPinToPCINT(CH1), pcintCH1, CHANGE);
  attachPCINT(digitalPinToPCINT(CH2), pcintCH2, CHANGE);
  attachPCINT(digitalPinToPCINT(CH5), pcintCH5, CHANGE);

  Serial.println("RC Car Ready.");
}


/* ───────── 매핑 함수 ────────────────────────*/
// 1) 조향 PWM → Servo 각도
int mapSteer(int us) {
  int deg = map(us, CH1_MIN, CH1_MAX, SERVO_MIN_DEG, SERVO_MAX_DEG)
            + SERVO_CENTER_OFFSET;
  return constrain(deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
}

// 2) 스로틀 PWM(수동) → ESC µs
int mapThrottle(int us) {
  int diff = us - CH_CENTER;
  if (abs(diff) <= THR_DEADBAND) return ESC_STOP;      // 중립

  return (diff > 0)                                    // 전진
       ? map(us, CH_CENTER + THR_DEADBAND, CH2_MAX,    // 1500+ → 1470~1510
              ESC_FWD_WEAK, ESC_FWD_STRONG)
       : map(us, CH2_MIN, CH_CENTER - THR_DEADBAND,    // 후진
              ESC_REV_STRONG, ESC_REV_WEAK);
}

// 3) 스로틀 바이트(자동) → ESC µs
int mapThrottleByte(uint8_t b) {
  if (b < 127)           // 0(강후진) → 126(약후진)
      return map(b, 0, 126, ESC_REV_STRONG, ESC_REV_WEAK);
  if (b > 128)           // 129(약전진) → 255(강전진)
      return map(b, 129, 255, ESC_FWD_WEAK, ESC_FWD_STRONG);
  return ESC_STOP;       // 127,128 = STOP
}


/* ───────── 메인 루프 ────────────────────────*/
void loop() {

  int steerDeg, escUs;

  /* ── (1) 수동 / 자동 모드에 따라 입력 선택 ──*/
  if (!autoMode) {
    // ••• 수동 : RC 스틱 값 사용 •••
    steerDeg = mapSteer(pwmCH1);
    escUs    = mapThrottle(pwmCH2);

  } else {
    // ••• 자동 : Pi → 아두이노 직렬 패킷 •••
    static int lastSteer = (SERVO_MIN_DEG + SERVO_MAX_DEG) / 2;
    static int lastEsc   = ESC_STOP;

    if (Serial.available() >= 3 && Serial.read() == 'P') {
      uint8_t sB = Serial.read();
      uint8_t tB = Serial.read();

      steerDeg = map(sB, 0, 255, SERVO_MIN_DEG, SERVO_MAX_DEG) + SERVO_CENTER_OFFSET;
      steerDeg = constrain(steerDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
      escUs    = mapThrottleByte(tB);

      lastSteer = steerDeg;
      lastEsc   = escUs;
    } else {            // 패킷 없으면 직전 값 유지
      steerDeg = lastSteer;
      escUs    = lastEsc;
    }
  }

  /* ── (2) 실제 출력 ─────────────────────────*/
  steer.write(steerDeg);
  esc.writeMicroseconds(escUs);

  /* ── (3) 방향 LED ──────────────────────────*/
  int centerWithOff = (SERVO_MIN_DEG + SERVO_MAX_DEG) / 2 + SERVO_CENTER_OFFSET;
  digitalWrite(LED_L, steerDeg < centerWithOff - 7);
  digitalWrite(LED_R, steerDeg > centerWithOff + 7);

  delay(15);   // 약 65 Hz 루프
}