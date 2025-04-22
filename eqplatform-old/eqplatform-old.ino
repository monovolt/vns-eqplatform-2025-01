#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <IRremote.hpp>

#if !defined(STR_HELPER)
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#endif

/***** 핀 설정 *****/
// 1번 모터 드라이버
const int motor1_enable_pin = 2;
const int motor1_step_pin = 3;
const int motor1_dir_pin = 4;

// 푸시버튼 핀 (내부 풀업 사용)
const int btn1_pin = 5;  // 오른쪽 끝 도달 확인용
const int btn2_pin = 6;  // 왼쪽 끝 도달 확인용
const int btn3_pin = 7;  // 모터 정지
const int btn4_pin = 8;  // 원래 이동방향으로 모터 구동 (오른쪽 끝이면 반대방향)

// 피에조 스피커 핀
const int piezo_pin = 9;

// IR 적외선 센서 (리모컨 수신)
const int IR_RECV_PIN = 12;

/***** 동작 변수 *****/
int tracking_delay = 5300;  // 왼쪽(반시계) 이동시 딜레이 (ms)
int default_delay = 5300;   // 왼쪽(반시계) 이동시 딜레이 (ms)
const int MAX_DELAY = 180;  // 오른쪽(시계) 이동시 딜레이 (ms)

bool motorRunning = false;  // 모터 구동 상태 플래그
bool moveRight = true;      // true: 오른쪽(시계), false: 왼쪽(반시계)
bool atRightEnd = false;    // 오른쪽 끝에 도달했는지 여부


/***** 피에조 멜로디용 음계 정의 *****/
#define NOTE_C4 262
#define NOTE_D4 294
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_B4 494

// "반짝반짝 작은별" 중 "반짝반짝 작은별" 부분 (푸시버튼1)
int melody_start[] = { NOTE_C4, NOTE_C4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_A4, NOTE_G4 };
int noteDurations_start[] = { 250, 250, 250, 250, 250, 250, 500 };

// "반짝반짝 작은별" 중 "아름답게 비치네" 부분 (푸시버튼2)
int melody_arrival[] = { NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_C4 };
int noteDurations_arrival[] = { 250, 250, 250, 250, 250, 250, 500 };

void setup() {
  // 모터 제어 핀 출력으로 설정
  pinMode(motor1_enable_pin, OUTPUT);
  pinMode(motor1_step_pin, OUTPUT);
  pinMode(motor1_dir_pin, OUTPUT);

  // 푸시버튼은 내부 풀업모드로 설정 (버튼 누를 때 LOW)
  pinMode(btn1_pin, INPUT_PULLUP);
  pinMode(btn2_pin, INPUT_PULLUP);
  pinMode(btn3_pin, INPUT_PULLUP);
  pinMode(btn4_pin, INPUT_PULLUP);

  // 피에조 스피커 핀 출력
  pinMode(piezo_pin, OUTPUT);

  // TMC2226 Enable: 일반적으로 LOW로 설정하면 모터 활성화 (하드웨어 사양에 맞게 조정)
  digitalWrite(motor1_enable_pin, LOW);
}

void loop() {
  // 푸시버튼 입력 체크
  checkPushButtons();

  // IR 리모컨 입력 체크
  if (irrecv.decode(&results)) {
    checkIRRemote(results.value);
    irrecv.resume();
  }

  // 모터 구동 상태이면 한 스텝씩 펄스 발생
  if (motorRunning) {
    // 이동 방향에 따라 모터 방향 설정 및 딜레이 적용
    if (moveRight) {
      // 오른쪽(시계방향) -> MAX_DELAY 사용
      digitalWrite(motor1_dir_pin, HIGH);
      stepMotors();
      delay(MAX_DELAY);
    } else {
      // 왼쪽(반시계방향) -> tracking_delay 사용
      digitalWrite(motor1_dir_pin, LOW);
      stepMotors();
      delay(tracking_delay);
    }
  }
}

/***** 함수 정의 *****/
// 푸시버튼 체크 (버튼은 눌림시 LOW)
void checkPushButtons() {
  if (digitalRead(btn1_pin) == LOW) {
    // [푸시버튼 1]: 모터 정지, "반짝반짝 작은별" 중 "반짝반짝 작은별" 재생, 오른쪽 끝 도달 상태로 설정
    stopMotors();
    playMelody(melody_start, noteDurations_start, 7);
    atRightEnd = true;
    delay(300);  // 디바운스
  }
  if (digitalRead(btn2_pin) == LOW) {
    // [푸시버튼 2]: 모터 정지, "반짝반짝 작은별" 중 "아름답게 비치네" 재생, 왼쪽 끝 도달 상태로 설정
    stopMotors();
    playMelody(melody_arrival, noteDurations_arrival, 7);
    atRightEnd = false;
    delay(300);
  }
  if (digitalRead(btn3_pin) == LOW) {
    // [푸시버튼 3]: 모터 정지, 피에조로 "띠~" 2초간 출력
    stopMotors();
    playBeep(2000);
    delay(300);
  }
  if (digitalRead(btn4_pin) == LOW) {
    // [푸시버튼 4]: 원래 이동 방향으로 모터 구동; 단, 오른쪽 끝이면 반대(왼쪽)로 이동
    if (atRightEnd) {
      moveRight = false;
    } else {
      moveRight = true;
    }
    motorRunning = true;
    delay(300);
  }
}

// IR 리모컨 명령 체크
void checkIRRemote(unsigned long value) {
  if (value == 3333) {
    // [IR 버튼 3]: 모터 정지, "띠~" 2초간 출력
    stopMotors();
    playBeep(2000);
  } else if (value == 4444) {
    // [IR 버튼 4]: 원래 이동 방향으로 모터 구동; 단, 오른쪽 끝이면 반대(왼쪽)로 이동
    if (atRightEnd) {
      moveRight = false;
    } else {
      moveRight = true;
    }
    motorRunning = true;
  } else if (value == 5555) {
    // [IR 버튼 5]: "띠~" 4초간 출력 후, 오른쪽으로 이동 (MAX_DELAY 적용)
    stopMotors();
    playBeep(4000);
    moveRight = true;
    motorRunning = true;
  } else if (value == 6666) {
    // [IR 버튼 6]: "띠~" 0.2초 출력 후, 이동속도 증가 (tracking_delay 50ms 감소)
    playBeep(200);
    tracking_delay = max(tracking_delay - 50, 50);  // 최소 50ms 제한
  } else if (value == 7777) {
    // [IR 버튼 7]: "띠~" 0.2초 출력 후, 이동속도 감소 (tracking_delay 50ms 증가)
    playBeep(200);
    tracking_delay += 50;
  }
}

// 두 모터에 동시에 한 스텝 펄스 발생
void stepMotors() {
  digitalWrite(motor1_step_pin, HIGH);
  digitalWrite(motor2_step_pin, HIGH);
  delayMicroseconds(100);  // 짧은 펄스 지속시간
  digitalWrite(motor1_step_pin, LOW);
  digitalWrite(motor2_step_pin, LOW);
}

// 모터 정지
void stopMotors() {
  motorRunning = false;
}

// 멜로디 재생 함수
void playMelody(int melody[], int noteDurations[], int length) {
  for (int i = 0; i < length; i++) {
    int noteDuration = noteDurations[i];
    tone(piezo_pin, melody[i], noteDuration);
    delay(noteDuration * 1.30);
    noTone(piezo_pin);
  }
}

// 비프음 재생 함수 (duration: ms)
void playBeep(int duration) {
  tone(piezo_pin, 1000);  // 1000Hz 비프음
  delay(duration);
  noTone(piezo_pin);
}