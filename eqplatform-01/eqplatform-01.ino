#include <Wire.h>
#include <IRremote.hpp>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TMCStepper.h>


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

constexpr int piezoPin = 9;  // piezo speaker
constexpr int ledPin = 11;   // Red LED. 보조배터리 전원공급 유지용
constexpr int irPin = 12;    // IR receiver

const unsigned long MAX_DELAY = 80;
unsigned long default_delay = 11365;   // AI 추천값 11365, 마지막 측정값 5300
unsigned long tracking_delay = 11365;  // AI 추천값 11365, 마지막 측정값 5300
unsigned long stop_delay = 9999999;


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
#define NOTE_C6 1047
#define NOTE_D6 1175
#define NOTE_E6 1319
#define NOTE_F6 1397
#define NOTE_G6 1568
#define NOTE_A6 1760

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

int trackingMelodyA[] = { 523, 587, 659, 698, 784 };
int trackingDurationsA[] = { 200, 200, 200, 200, 300 };

int trackingMelodyB[] = { 523, 659, 784, 1046 };
int trackingDurationsB[] = { 250, 250, 250, 250 };

int speedSaveMelodyA[] = { 523, 659, 784 };
int speedSaveDurationsA[] = { 150, 150, 150 };

int speedSaveMelodyB[] = { 392, 392, 392 };
int speedSaveDurationsB[] = { 200, 200, 200 };

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);



int buttonState1 = 0;  // variable for reading the pushbutton status
int buttonState2 = 0;  // variable for reading the pushbutton status
int buttonState3 = 0;  // variable for reading the pushbutton status
int buttonState4 = 0;  // variable for reading the pushbutton status

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
  driver.toff(4);                // 필수 초기 설정
  driver.rms_current(600);       // mA
  driver.hold_multiplier(0.3);   // 정지 시 전류 줄이기
  driver.en_spreadCycle(false);  // StealthChop 모드 활성화
  driver.microsteps(8);          // 8 마이크로스텝으로 설정

  // buttons
  pinMode(btn1Pin, INPUT_PULLUP);
  pinMode(btn2Pin, INPUT_PULLUP);
  pinMode(btn3Pin, INPUT_PULLUP);
  pinMode(btn4Pin, INPUT_PULLUP);

  // piezo
  pinMode(piezoPin, OUTPUT);

  // LED Red
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  updateDisplay();
}

void loop() {
  checkButtons();
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

  // 모터 구동 상태이면 한 스텝씩 펄스 발생
  if (motorRunning) {
    // 이동 방향에 따라 모터 방향 설정 및 딜레이 적용
    unsigned long now = millis();
    if (moveRight) {
      if (now - lastStepTime >= MAX_DELAY) {
        // 오른쪽(시계방향) -> MAX_DELAY 사용
        digitalWrite(dirPin, HIGH);
        stepMotors();
        delayMicroseconds(MAX_DELAY);
      }

    } else {
      if (now - lastStepTime >= tracking_delay) {
        // 왼쪽(반시계방향) -> tracking_delay 사용
        digitalWrite(dirPin, LOW);
        stepMotors();
        delayMicroseconds(tracking_delay);
      }
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
}


void checkIR(long cmd) {

  switch (cmd) {
    case 69:
      // 버튼1 종료위치이동완료
      stopMotors();
      atRightEnd = false;
      currentStatus = "End.";
      updateDisplay();
      playMelody(lgMelody, lgMelodyDurations, lgLen);
      break;
    case 71:
      // 버튼3  시작위치이동완료
      stopMotors();
      atRightEnd = true;
      currentStatus = "Start Rdy";
      updateDisplay();
      playMelody(starMelody, starDurations, starLen);
      break;
    case 70:
      // 버튼2 정지. 버튼#과 동일
      stopMotors();
      currentStatus = "Stopped";
      updateDisplay();
      playMelody(stopMelodyA, stopDurationsA, sizeof(stopMelodyA) / sizeof(stopMelodyA[0]));
      break;
    case 13:
      // 버튼# 정지. 버튼2와 동일
      stopMotors();
      currentStatus = "Stopped";
      updateDisplay();
      playMelody(stopMelodyA, stopDurationsA, sizeof(stopMelodyA) / sizeof(stopMelodyA[0]));
      break;
    case 28:
      // OK: tracking_delay reset 후 재구동
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
      // 오른쪽 화살표
      playBeep(1000);
      moveRight = true;
      tracking_delay = MAX_DELAY;
      motorRunning = true;
      currentStatus = "Mov B Pos";
      updateDisplay();
      break;
    case 8:
      // 왼쪽 화살표
      playBeep(1000);
      moveRight = false;
      tracking_delay = MAX_DELAY;
      motorRunning = true;
      currentStatus = "Mov E Pos";
      updateDisplay();
      break;
    case 24:
      // 위 화살표: 속도 증가 50씩
      tracking_delay = max((long)tracking_delay - 50, MAX_DELAY);
      updateDisplay();
      Serial.println(tracking_delay);
      break;
    case 82:
      // 아래 화살표: 속도 감소 50씩
      tracking_delay += 50;
      updateDisplay();
      Serial.println(tracking_delay);
      break;
    case 67:
      // 버튼 6: 속도 증가 500씩
      tracking_delay = max((long)tracking_delay - 500, MAX_DELAY);
      updateDisplay();
      Serial.println(tracking_delay);
      break;
    case 68:
      // 버튼 4: 속도 감소 500씩
      tracking_delay += 500;
      updateDisplay();
      Serial.println(tracking_delay);
      break;
    case 25:
      // 0: 속도 저장 Tracking.. 때만 저장 됨.
      if (currentStatus == "Tracking..") {
        default_delay = tracking_delay;
        playMelody(speedSaveMelodyA, speedSaveDurationsA, sizeof(speedSaveMelodyA) / sizeof(speedSaveMelodyA[0]));
        updateDisplay();
      } else {
        playBeepLow(200);
      }
      break;
    case 64:
      // 버튼 5 : 어떤 상태에서건 Tracking.. 상태로
      moveRight = false;
      tracking_delay = default_delay;
      currentStatus = "Tracking..";
      playMelody(trackingMelodyB, trackingDurationsB, sizeof(trackingMelodyB) / sizeof(trackingMelodyB[0]));
      updateDisplay();
      motorRunning = true;
      break;
    default:
      currentStatus = "Unknown Cmd";
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
  display.print("T: ");
  display.println(tracking_delay);
  display.print("D: ");
  display.print(default_delay);
  display.display();
  Serial.println(currentStatus);
  // delay(10);
}

void playMelody(int *melody, int *durations, int length) {
  for (int i = 0; i < length; i++) {
    tone(piezoPin, melody[i], durations[i]);
    delay(durations[i] * 1.2);
    noTone(piezoPin);
  }
}

void playBeep(int duration) {
  tone(piezoPin, 1000);
  delay(duration);
  noTone(piezoPin);
}

void playBeepLow(int duration) {
  tone(piezoPin, 200);
  delay(duration);
  noTone(piezoPin);
}