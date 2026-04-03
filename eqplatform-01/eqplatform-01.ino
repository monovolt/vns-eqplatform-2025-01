#include <Wire.h>
#include <IRremote.hpp>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TMCStepper.h>
#include <MPU9250_asukiaaa.h>

MPU9250_asukiaaa sensor;
float pitch = 0, roll = 0;
const float LIMIT_ANGLE = 6.5;  // ±6.5도 제한


#if !defined(STR_HELPER)
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#endif

#define R_SENSE 0.11f
TMC2209Stepper driver((Stream *)&Serial1, R_SENSE, 0x00);  // 기본 슬레이브 주소 0

// pins
constexpr int enablePin = 2;  // TMC2226 ENABLE (LOW = enabled)
constexpr int stepPin = 3;    // TMC2226 STEP
constexpr int dirPin = 4;     // TMC2226 DIR

constexpr int btn1Pin = 5;  // 오른쪽 끝 확인
constexpr int btn2Pin = 6;  // 왼쪽 끝 확인
constexpr int btn3Pin = 7;  // 모터 정지
constexpr int btn4Pin = 8;  // 원래 방향 재구동
constexpr int btn5Pin = 9;  // 초기화(가운데로 이동) - Welcome..

constexpr int piezoPin = 10;  // piezo speaker
constexpr int ledPin = 11;   // Red LED. 보조배터리 전원공급 유지용
constexpr int irPin = 12;    // IR receiver

const unsigned long MAX_DELAY = 300;  // micro second 단위
unsigned long default_delay = 11365;   // AI 추천값 11365, 마지막 측정값 5300
unsigned long tracking_delay = 11365;  // AI 추천값 11365, 마지막 측정값 5300
unsigned long stop_delay = 99999999;

float leveling_tolerance = 0.1;  // 켈리브레이션 후 ±0.1도 이내로 정밀 수평

unsigned long lastStepTime = 0;

// 디바운스 관련 변수
unsigned long lastIRCommandTime = 0;
const unsigned long IR_DEBOUNCE_INTERVAL = 200;  // 200ms

// motor state
bool motorRunning = false;
bool moveRight = true;
bool atRightEnd = false;

// display state
String currentStatus = "Welcome..";

// note definitions
#define NOTE_C5 523
#define NOTE_D5 587
#define NOTE_E5 659
#define NOTE_F5 698
#define NOTE_G5 784
#define NOTE_A5 880
#define NOTE_B5 988

#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440

// LG 종료음
#define NOTE_AS4 466
#define NOTE_CS5 554
#define NOTE_DS5 622
#define NOTE_FS5 740
#define NOTE_GS4 415
#define NOTE_B4 494
#define NOTE_GS5 831

// "반짝반짝 작은별" 첫 구절
int starMelody[] = { NOTE_C5, NOTE_C5, NOTE_G5, NOTE_G5, NOTE_A5, NOTE_A5, NOTE_G5 };
int starDurations[] = { 250, 250, 250, 250, 250, 250, 500 };
const int starLen = sizeof(starMelody) / sizeof(int);

// LG 종료음 일부
int lgMelody[] = {
  NOTE_CS5,  // C#5
  NOTE_FS5,  // F#5
  NOTE_F5,   // F5 (E#5로 읽음)
  NOTE_DS5,  // D#5
  NOTE_CS5,  // C#5
  NOTE_AS4,  // A#4

  NOTE_B4, NOTE_CS5, NOTE_DS5,
  NOTE_GS4, NOTE_AS4, NOTE_B4,
  NOTE_AS4, NOTE_CS5,

  NOTE_CS5,
  NOTE_FS5, NOTE_F5, NOTE_DS5,
  NOTE_CS5,
  NOTE_FS5,

  NOTE_FS5, NOTE_GS5, NOTE_FS5,
  NOTE_F5, NOTE_DS5, NOTE_F5,
  NOTE_FS5
};

int lgMelodyDurations[] = {
  500, 166, 166, 166, 500, 500,

  166, 166, 166,
  166, 166, 166,
  500, 500,

  500,
  166, 166, 166,
  500,
  500,

  166, 166, 166,
  166, 166, 166,
  1000
};
const int lgLen = sizeof(lgMelody) / sizeof(int);

int stopMelodyA[] = { 880, 784, 698, 698 };
int stopDurationsA[] = { 200, 200, 200, 300 };

int stopMelodyB[] = { 659, 587, 523 };
int stopDurationsB[] = { 300, 300, 300 };

int levelingMelodyA[] = { 523, 587, 659, 698, 784 };
int levelingDurationsA[] = { 200, 200, 200, 200, 300 };

int trackingMelodyB[] = { 523, 659, 784, 1046 };
int trackingDurationsB[] = { 250, 250, 250, 250 };

int speedSaveMelodyA[] = { 523, 659, 784 };
int speedSaveDurationsA[] = { 150, 150, 150 };

// 반달 - Welcome 음
int welcomeMelody[] = {
  NOTE_C5, NOTE_D5, NOTE_C5, NOTE_A4,    // 도 레 도 라
  NOTE_C5, NOTE_A4, NOTE_F4, NOTE_C4,    // 도 라 파 도
  NOTE_D4, NOTE_F4, NOTE_G4, NOTE_C5, NOTE_A4 // 레(낮은 레!) 파 솔 도 라
};

int welcomeDurations[] = {
  400, 200, 400, 200,     // 도 레 도 라
  200, 200, 200, 600,     // 도 라 파 도
  400, 200, 400, 200, 800 // 레 파 솔 도 라
};


#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// OLED Display 갱신 interval
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 800; // 800ms


// Roll, Pitch 값 필터용 변수
// 필터 파라미터
const int FILTER_SIZE = 10;         // 이동평균 샘플 개수 (원하면 10~20까지 조절)
const float JUMP_THRESHOLD = 5.0;  // 한 번에 튈 때 무시할 각도 임계값

// 연속 점프 카운트
const int MAX_JUMP_COUNT = 3;
int jumpCountRoll = 0;
int jumpCountPitch = 0;

// Roll용 버퍼
float rollBuffer[FILTER_SIZE] = {0};
int rollIndex = 0;
float rollSum = 0;
float lastRoll = 0;
float rollFiltered = 0;

// Pitch용 버퍼
float pitchBuffer[FILTER_SIZE] = {0};
int pitchIndex = 0;
float pitchSum = 0;
float lastPitch = 0;
float pitchFiltered = 0;

// 기울기 센서 사용 여부
boolean isSensorOn = false;

// 소리 사용 여부
boolean isSoundOn = true;

// Roll 영점 보정값 (센서 calibration용)
float rollOffset = 0.0;

unsigned long lastSensorUpdate = 0;
const unsigned long SENSOR_UPDATE_INTERVAL = 200; // ms (0.1초, 필요시 더 느리게도 OK)

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 2000)
    ;  // Wait for Serial to become available. Is optimized away for some cores.
  // Just to know which program is running on my Arduino
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));
  // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
  IrReceiver.begin(irPin, ENABLE_LED_FEEDBACK);

  Serial.print(F("Ready to receive IR signals of protocols: "));
  printActiveIRProtocols(&Serial);
  Serial.println(F("at pin " STR(irPin)));


  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000);  // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  // motor pins
  pinMode(enablePin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(enablePin, LOW);  // enable driver

  // TMC2226 Setup Using UART
  Serial1.begin(115200);
  driver.begin();
  driver.toff(4);               // 필수 초기 설정
  driver.rms_current(600);      // mA
  driver.hold_multiplier(1.0);  // 정지 시 전류 100% 유지. -- 보조배터리 전원 차단 문제 때문.
  driver.en_spreadCycle(true);  // StealthChop Off
  driver.microsteps(8);         // 8 마이크로스텝으로 설정
  driver.intpol(true);          // 보간 켬 (기본값)

  // buttons
  pinMode(btn1Pin, INPUT_PULLUP);
  pinMode(btn2Pin, INPUT_PULLUP);
  pinMode(btn3Pin, INPUT_PULLUP);
  pinMode(btn4Pin, INPUT_PULLUP);
  pinMode(btn5Pin, INPUT_PULLUP);

  // piezo
  pinMode(piezoPin, OUTPUT);

  // LED Red
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);


  if(isSensorOn == true) {
    Wire.begin();
    sensor.setWire(&Wire);
    sensor.beginAccel();
    // sensor.beginGyro(); // 기울기만 쓸 땐 생략해도 OK
    // sensor.beginMag();  // 자력계 필요 없으니 생략
  }  

  updateDisplay();
  playMelody(welcomeMelody, welcomeDurations, sizeof(welcomeMelody) / sizeof(welcomeMelody[0]));
  
}

void loop() {
  // 버튼 확인
  checkButtons();

  // IR 리모컨 입력 확인
  if (IrReceiver.decode()) {
    if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
      Serial.println(F("Received noise or an unknown (or not yet enabled) protocol"));
      // We have an unknown protocol here, print extended info
      IrReceiver.printIRResultRawFormatted(&Serial, true);
      IrReceiver.resume();  // Do it here, to preserve raw data for printing with printIRResultRawFormatted()
    } else {
      IrReceiver.resume();  // Early enable receiving of the next IR frame
      IrReceiver.printIRResultShort(&Serial);
      IrReceiver.printIRSendUsage(&Serial);
      // 디바운싱: 이전 명령 처리 이후 설정한 간격이 지났는지 확인
      if (millis() - lastIRCommandTime >= IR_DEBOUNCE_INTERVAL) {
        checkIR(IrReceiver.decodedIRData.command);
        lastIRCommandTime = millis();  // 마지막 처리 시각 업데이트
      }
    }
    Serial.println();
  }

  unsigned long nowMillis = millis();

  if(isSensorOn && nowMillis - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL) {
    // MPU-9250 기울기 센서 인식 값 갱신
    sensor.accelUpdate();
    float ax = sensor.accelX();
    float ay = sensor.accelY();
    float az = sensor.accelZ();
    
    float rawPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI; // 앞/뒤
    float rawRoll  = atan2(ay, az) * 180.0 / PI * -1; // 좌/우

    // 센서 거꾸로 장착 보정 (180도 회전)
    rawRoll += 180.0;
    if (rawRoll > 180.0) rawRoll -= 360.0;  // -180 ~ 180 범위로 조정
    rawRoll = -rawRoll;  // 좌우 반전

    // 2. 필터 적용
    pitchFiltered = filterValue(rawPitch, lastPitch, pitchBuffer, pitchIndex, pitchSum, jumpCountPitch);
    rollFiltered  = filterValue(rawRoll, lastRoll, rollBuffer, rollIndex, rollSum,  jumpCountRoll);

    pitch = pitchFiltered; // 앞/뒤 - 전역변수
    roll = rollFiltered - rollOffset;   // 좌/우 - 전역변수 (영점 보정 적용)

    lastSensorUpdate = nowMillis;

    // 각도 제한 체크 (좌우 임계 각도를 넘어가는지)
    checkLeftLimit();
    checkRightLimit();

    // 레벨링 체크 
    checkLevelingComplete();
  }


  // 모터 구동 상태이면 한 스텝씩 펄스 발생
  if (motorRunning) {
    // 이동 방향에 따라 모터 방향 설정 및 딜레이 적용
    unsigned long now = micros();
    if (moveRight) {
      if (now - lastStepTime >= MAX_DELAY) { // 불필요 코드
        // 오른쪽(반시계방향) -> MAX_DELAY 사용
        digitalWrite(dirPin, LOW);
        stepMotors();
        //delayMicroseconds(MAX_DELAY);
        lastStepTime = now;
      }

    } else {
      if (now - lastStepTime >= tracking_delay) { // 불필요 코드
        // 왼쪽(시계방향) -> tracking_delay 사용
        digitalWrite(dirPin, HIGH);
        stepMotors();
        //delayMicroseconds(tracking_delay);
        lastStepTime = now;
      }
    }
  }

  
  if(isSensorOn == true) {
    // 자이로 상태를 보기 위해 OLED 디스플레이 주기적 갱신 -- 느려지는 버그 있음.
    if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
      updateDisplay();
      lastDisplayUpdate = millis();
    }
  }
}

void checkButtons() {
  if (digitalRead(btn1Pin) == LOW) {
    // 준비 위치(오른쪽 끝)
    // delay(10);
    // if (digitalRead(btn1Pin) != LOW) return;
    stopMotors();
    atRightEnd = true;
    currentStatus = "Start Rdy";
    updateDisplay();
    playMelody(starMelody, starDurations, starLen);
    delay(300);
  }
  if (digitalRead(btn2Pin) == LOW) {
    // 도착 완료(왼쪽 끝)
    // delay(10);
    // if (digitalRead(btn2Pin) != LOW) return;
    stopMotors();
    atRightEnd = false;
    currentStatus = "End.";
    updateDisplay();
    playMelody(lgMelody, lgMelodyDurations, lgLen);
    delay(300);
  }
  if (digitalRead(btn3Pin) == LOW) {
    // 정지
    // delay(10);
    // if (digitalRead(btn3Pin) != LOW) return;
    printDrvStatus();
    stopMotors();
    currentStatus = "Stopped";
    updateDisplay();
    playMelody(stopMelodyA, stopDurationsA, sizeof(stopMelodyA) / sizeof(stopMelodyA[0]));
    delay(300);
  }
  if (digitalRead(btn4Pin) == LOW) {
    // 원래 방향 재구동
    // delay(10);
    // if (digitalRead(btn4Pin) != LOW) return;
    if (atRightEnd) {
      moveRight = false;
      tracking_delay = default_delay;
      currentStatus = "Tracking..";
      playMelody(trackingMelodyB, trackingDurationsB, sizeof(trackingMelodyB) / sizeof(trackingMelodyB[0]));
    } else {
      moveRight = true;
      tracking_delay = MAX_DELAY;
      currentStatus = "Mov B Pos";
      playBeep(1000);
    }
    motorRunning = true;
    updateDisplay();
    delay(300);
  }
  if (digitalRead(btn5Pin) == LOW) {
    // 레벨링 - 가운데로 이동 
    // delay(10);
    // if (digitalRead(btn3Pin) != LOW) return;
    printDrvStatus();
    stopMotors();
    playMelody(levelingMelodyA, levelingDurationsA, sizeof(levelingMelodyA) / sizeof(levelingMelodyA[0]));
    startLeveling();
    delay(300);
  }
}


void checkIR(long cmd) {

  switch (cmd) {
    case 69:
      // 0x45 버튼1 종료위치이동완료
      stopMotors();
      atRightEnd = false;
      currentStatus = "End.";
      updateDisplay();
      playMelody(lgMelody, lgMelodyDurations, lgLen);
      break;
    case 70:
      // 0x46 버튼2 정지. 버튼#과 동일
      printDrvStatus();
      stopMotors();
      currentStatus = "Stopped";
      updateDisplay();
      playMelody(stopMelodyA, stopDurationsA, sizeof(stopMelodyA) / sizeof(stopMelodyA[0]));
      break;
    case 71:
      // 0x47 버튼3  시작위치이동완료
      stopMotors();
      atRightEnd = true;
      currentStatus = "Start Rdy";
      updateDisplay();
      playMelody(starMelody, starDurations, starLen);
      break;
    case 68:
      // 0x44 버튼 4: 속도 감소 500씩
      tracking_delay += 500;
      updateDisplay();
      Serial.println(tracking_delay);
      break;
    case 64:
      // 0x40 버튼 5 : 어떤 상태에서건 Tracking.. 상태로
      moveRight = false;
      tracking_delay = default_delay;
      currentStatus = "Tracking..";
      playMelody(trackingMelodyB, trackingDurationsB, sizeof(trackingMelodyB) / sizeof(trackingMelodyB[0]));
      updateDisplay();
      motorRunning = true;
      break;
    case 67:
      // 0x43 버튼 6: 속도 증가 500씩
      tracking_delay = max((long)tracking_delay - 500, MAX_DELAY);
      updateDisplay();
      Serial.println(tracking_delay);
      break;
    case 7:
      // 0x7 버튼 7: 소리 ON/OFF 토글
      isSoundOn = !isSoundOn;
      if(isSoundOn == true) {
        playBeep(200);  // 소리 ON 확인음 (짧게)
      }
      // 소리 OFF일 때는 당연히 소리가 안남
      updateDisplay();
      break;
    case 21:
      // 0x15 버튼 8: 센서 ON/OFF 토글
      isSensorOn = !isSensorOn;
      if(isSensorOn == true) {
        Wire.begin();
        sensor.setWire(&Wire);
        sensor.beginAccel();
        // sensor.beginGyro(); // 기울기만 쓸 땐 생략해도 OK
        // sensor.beginMag();  // 자력계 필요 없으니 생략
        playBeep(800);  // 센서 ON 소리
      } else {
        playBeepLow(400);  // 센서 OFF 소리
      }
      updateDisplay();  // 즉시 디스플레이 업데이트
      break;
    case 9:
      // 0x9 버튼 9: 센서 영점 보정값 저장 (Welcome 상태 & ±3도 이내에서만)
      if (currentStatus == "Welcome.." && abs(roll) <= 3.0) {
        rollOffset = roll;  // 현재 roll 값을 offset으로 저장
        playMelody(speedSaveMelodyA, speedSaveDurationsA, sizeof(speedSaveMelodyA) / sizeof(speedSaveMelodyA[0]));
        updateDisplay();
      } else {
        // 조건 불만족 시 경고음
        playBeepLow(300);
      }
      break;
    case 22:
      // 0x16 버튼 *: 가운데로 빠르게 이동후 Welcome.. 상태로
      printDrvStatus();
      stopMotors();
      playMelody(levelingMelodyA, levelingDurationsA, sizeof(levelingMelodyA) / sizeof(levelingMelodyA[0]));
      startLeveling();
      break;
    case 25:
      // 0x19 버튼 0: 속도 저장 Tracking.. 때만 저장 됨.
      if (currentStatus == "Tracking..") {
        default_delay = tracking_delay;
        playMelody(speedSaveMelodyA, speedSaveDurationsA, sizeof(speedSaveMelodyA) / sizeof(speedSaveMelodyA[0]));
        updateDisplay();
      } else {
        playBeepLow(200);
      }
      break;
    case 13:
      // 0xD 버튼# 정지. 버튼2와 동일
      printDrvStatus();
      stopMotors();
      currentStatus = "Stopped";
      updateDisplay();
      playMelody(stopMelodyA, stopDurationsA, sizeof(stopMelodyA) / sizeof(stopMelodyA[0]));
      break;


    case 28:
      // 0x1C OK: tracking_delay reset 후 재구동
      if (atRightEnd) {
        moveRight = false;
        tracking_delay = default_delay;
        currentStatus = "Tracking..";
        playMelody(trackingMelodyB, trackingDurationsB, sizeof(trackingMelodyB) / sizeof(trackingMelodyB[0]));
      } else {
        moveRight = true;
        tracking_delay = MAX_DELAY;
        currentStatus = "Mov B Pos";
        playBeep(1000);
      }
      updateDisplay();
      motorRunning = true;
      break;
    case 90:
      // 0x5A 오른쪽 화살표 - 시작 위치 방향으로 빠르게 이동
      playBeep(1000);
      moveRight = true;
      tracking_delay = MAX_DELAY;
      motorRunning = true;
      currentStatus = "Mov B Pos";
      updateDisplay();
      break;
    case 8:
      // 0x8 왼쪽 화살표 - 종료위치 방향으로 빠르게 이동
      playBeep(1000);
      moveRight = false;
      tracking_delay = MAX_DELAY;
      motorRunning = true;
      currentStatus = "Mov E Pos";
      updateDisplay();
      break;
    case 24:
      // 0x18 위 화살표: 속도 증가 20씩
      tracking_delay = max((long)tracking_delay - 20, MAX_DELAY);
      updateDisplay();
      Serial.println(tracking_delay);
      break;
    case 82:
      // 0x52 아래 화살표: 속도 감소 20씩
      tracking_delay += 20;
      updateDisplay();
      Serial.println(tracking_delay);
      break;

    default:
      currentStatus = "Unknwn Cmd";
      break;
  }
  IrReceiver.resume();
}
// 두 모터에 동시에 한 스텝 펄스 발생
void stepMotors() {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(80);  // 최소 펄스 유지 시간
  digitalWrite(stepPin, LOW);
}

// 모터 정지
void stopMotors() {
  motorRunning = false;
  tracking_delay = stop_delay;
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(2);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);
  display.println(currentStatus);
  display.print("D : ");
  if(tracking_delay >= stop_delay) {
    display.println("-----");
  } else {
    display.println(tracking_delay);
  }

  display.setTextSize(1);
  display.print("Mem. Delay: ");
  display.println(default_delay);
  display.print("Lv.Snsr: ");
  display.print(isSensorOn ? "On" : "Off");
  display.print(" Snd: ");
  display.println(isSoundOn ? "On" : "Off");
  display.print("L/R Angle :");
  display.print(roll, 1);
  display.print(" (");
  display.print(rollOffset, 1);
  display.println(")");
  display.print("Pitch: ");
  display.print(pitch, 1);
  display.println();
  display.display();
  Serial.println(currentStatus);
  // delay(10);
}

void playMelody(int *melody, int *durations, int length) {
  if (!isSoundOn) return;  // 소리 꺼져있으면 실행 안함
  for (int i = 0; i < length; i++) {
    tone(piezoPin, melody[i], durations[i]);
    delay(durations[i] * 1.2);
    noTone(piezoPin);
  }
}

void playBeep(int duration) {
  if (!isSoundOn) return;  // 소리 꺼져있으면 실행 안함
  tone(piezoPin, 1000);
  delay(duration);
  noTone(piezoPin);
}

void playBeepLow(int duration) {
  if (!isSoundOn) return;  // 소리 꺼져있으면 실행 안함
  tone(piezoPin, 200);
  delay(duration);
  noTone(piezoPin);
}

void checkLeftLimit() {
  // 왼쪽이 LIMIT_ANGLE 넘어가면 종료위치 도달!
  if (roll > LIMIT_ANGLE) {
    if (currentStatus == "Tracking..") {
      // checkIR 함수의 case 69 (왼쪽 끝 도달)
      stopMotors();
      atRightEnd = false;
      currentStatus = "End.";
      updateDisplay();
      playMelody(lgMelody, lgMelodyDurations, lgLen);
    } else if (currentStatus == "Mov E Pos") {
      // checkIR 함수의 case 70 (정지)
      printDrvStatus();
      stopMotors();
      currentStatus = "Stopped";
      updateDisplay();
      playMelody(stopMelodyA, stopDurationsA, sizeof(stopMelodyA) / sizeof(stopMelodyA[0]));
    }
    // 나머지 상태에선 아무것도 안 함
  }
}

void checkRightLimit() {
  // 오른쪽이 LIMIT_ANGLE 넘어가면 시작위치 도달!
  if (roll < -LIMIT_ANGLE) {
    if (currentStatus == "Mov B Pos") {
      // checkIR 함수의 case 71 (시작위치 도달)
      stopMotors();
      atRightEnd = true;
      currentStatus = "Start Rdy";
      updateDisplay();
      playMelody(starMelody, starDurations, starLen);
    }
    // 나머지 상태에선 아무것도 안 함
  }
}

void startLeveling() {
  // 센서가 꺼져있으면 leveling 불가능 - 경고음만 내고 리턴
  if (!isSensorOn) {
    playBeepLow(300);
    return;
  }

  if (abs(roll) > leveling_tolerance) { // 이미 충분히 수평이면 굳이 이동 안 해도 됨
    if (roll > 0) {
      moveRight = true;
      tracking_delay = MAX_DELAY; // 빠른 이동
      currentStatus = "Leveling>";
    } else {
      moveRight = false;
      tracking_delay = MAX_DELAY; // 빠른 이동
      currentStatus = "Leveling<";
    }
    motorRunning = true;
  } else {
    // 이미 수평임
    currentStatus = "Welcome..";
    motorRunning = false;
  }
  updateDisplay();
}

void checkLevelingComplete() {
  if ((currentStatus == "Leveling>" || currentStatus == "Leveling<") && abs(roll) <= leveling_tolerance) {
    stopMotors();
    currentStatus = "Welcome..";
    updateDisplay();
    playMelody(welcomeMelody, welcomeDurations, sizeof(welcomeMelody) / sizeof(welcomeMelody[0]));
  }
}

// 이동 평균 + 이상값 배제 (Roll, Pitch용 별도)
float filterValue(float newValue, float& lastValue, float* buffer, int& index, float& sum, int& jumpCount) {
  if (abs(newValue - lastValue) > JUMP_THRESHOLD) {
    jumpCount++;
    if (jumpCount < MAX_JUMP_COUNT) {
      // 한두 번은 무시
      newValue = lastValue;
    } else {
      // 연속으로 튀면 변화 허용(센서가 급격하게 움직였다고 판단)
      jumpCount = 0;
    }
  } else {
    jumpCount = 0;
  }
  lastValue = newValue;

  // 이동평균 적용
  sum -= buffer[index];
  buffer[index] = newValue;
  sum += newValue;
  index = (index + 1) % FILTER_SIZE;
  return sum / FILTER_SIZE;
}


void printDrvStatus() {
  uint32_t drvStatus = driver.DRV_STATUS();

  Serial.println(F("===== TMC2226 DRV_STATUS ====="));

  // 온도 경고
  bool otpWarning = (drvStatus >> 26) & 0x01;
  Serial.print(F("Overtemperature Warning (OTPW): "));
  Serial.println(otpWarning ? "⚠️ YES" : "OK");

  // 온도 과열 셧다운
  bool otShutdown = (drvStatus >> 25) & 0x01;
  Serial.print(F("Overtemperature Shutdown (OT): "));
  Serial.println(otShutdown ? "🔥 YES (driver shutdown!)" : "OK");

  // 스텝 손실 감지
  bool stallGuardA = (drvStatus >> 24) & 0x01;
  bool stallGuardB = (drvStatus >> 23) & 0x01;
  Serial.print(F("StallGuard A (lost steps): "));
  Serial.println(stallGuardA ? "❌ LOST" : "OK");

  Serial.print(F("StallGuard B (lost steps): "));
  Serial.println(stallGuardB ? "❌ LOST" : "OK");

  // 전류 스케일
  uint8_t cs_actual = (drvStatus >> 16) & 0x1F;
  Serial.print(F("Current Scale (CS_ACTUAL): "));
  Serial.print(cs_actual);
  Serial.println(F(" (0=off, 31=max)"));

  // 스텝 속도 상태 (스텝 누락 판정 감도)
  uint16_t tstep = driver.TSTEP();
  Serial.print(F("TSTEP (time between steps): "));
  Serial.println(tstep);

  Serial.println(F("=============================="));
}