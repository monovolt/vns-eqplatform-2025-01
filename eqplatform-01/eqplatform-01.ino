#include <Wire.h>
#include <IRremote.hpp>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#if !defined(STR_HELPER)
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#endif

// pins
constexpr int enablePin = 2;  // TMC2226 ENABLE (LOW = enabled)
constexpr int stepPin = 3;    // TMC2226 STEP
constexpr int dirPin = 4;     // TMC2226 DIR

constexpr int btn1Pin = 5;  // 오른쪽 끝 확인
constexpr int btn2Pin = 6;  // 왼쪽 끝 확인
constexpr int btn3Pin = 7;  // 모터 정지
constexpr int btn4Pin = 8;  // 원래 방향 재구동

constexpr int piezoPin = 9;  // piezo speaker
constexpr int IR_PIN = 12;   // IR receiver

const unsigned long MAX_DELAY = 180;
unsigned long default_delay = 5300;
unsigned long tracking_delay = 5300;

// IR debouncing
unsigned long lastIRTime = 0;
const unsigned long IR_DEBOUNCE = 100;

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

// "반짝반짝 작은별" 첫 구절
int starMelody[] = { NOTE_C5, NOTE_C5, NOTE_G5, NOTE_G5, NOTE_A5, NOTE_A5, NOTE_G5 };
int starDurations[] = { 250, 250, 250, 250, 250, 250, 500 };
const int starLen = sizeof(starMelody) / sizeof(int);

// LG 종료음 일부
int lgMelody[] = { NOTE_E6, NOTE_G6, NOTE_A6, NOTE_G6, NOTE_E6, NOTE_D6 };
int lgDurations[] = { 150, 150, 150, 150, 150, 150 };
const int lgLen = sizeof(lgMelody) / sizeof(int);


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
  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);

  Serial.print(F("Ready to receive IR signals of protocols: "));
  printActiveIRProtocols(&Serial);
  Serial.println(F("at pin " STR(IR_PIN)));


  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  // motor pins
  pinMode(enablePin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(enablePin, LOW);  // enable driver

  // buttons
  pinMode(btn1Pin, INPUT_PULLUP);
  pinMode(btn2Pin, INPUT_PULLUP);
  pinMode(btn3Pin, INPUT_PULLUP);
  pinMode(btn4Pin, INPUT_PULLUP);

  // piezo
  pinMode(piezoPin, OUTPUT);

  updateDisplay();

}

void loop() {
  checkButtons();
  checkIR();

  // 모터 구동 상태이면 한 스텝씩 펄스 발생
  if (motorRunning) {
    // 이동 방향에 따라 모터 방향 설정 및 딜레이 적용
    if (moveRight) {
      // 오른쪽(시계방향) -> MAX_DELAY 사용
      digitalWrite(dirPin, HIGH);
      stepMotors();
      delay(MAX_DELAY);
    } else {
      // 왼쪽(반시계방향) -> tracking_delay 사용
      digitalWrite(dirPin, LOW);
      stepMotors();
      delay(tracking_delay);
    }
  }
}

void checkButtons() {
  if (digitalRead(btn1Pin) == LOW) {
    // 준비 위치(오른쪽 끝)
    delay(10);
    if (digitalRead(btn1Pin) != LOW) return;
    stopMotors();
    atRightEnd = true;
    currentStatus = "Start Rdy";
    updateDisplay();
    playMelody(starMelody, starDurations, starLen);
  }
  if (digitalRead(btn2Pin) == LOW) {
    // 도착 완료(왼쪽 끝)
    delay(10);
    if (digitalRead(btn2Pin) != LOW) return;
    stopMotors();
    atRightEnd = false;
    currentStatus = "End.";
    updateDisplay();
    playMelody(lgMelody, lgDurations, lgLen);
  }
  if (digitalRead(btn3Pin) == LOW) {
    // 정지
    delay(10);
    if (digitalRead(btn3Pin) != LOW) return;
    stopMotors();
    currentStatus = "Stopped";
    updateDisplay();
    playBeep(2000);
  }
  if (digitalRead(btn4Pin) == LOW) {
    // 원래 방향 재구동
    delay(10);
    if (digitalRead(btn4Pin) != LOW) return;
    if (atRightEnd) {
      moveRight = false;
      tracking_delay = default_delay;
      currentStatus = "Tracking..";
    } else {
      moveRight = true;
      tracking_delay = MAX_DELAY;
      currentStatus = "Mov B Pos";
    }
    motorRunning = true;
    updateDisplay();
  }
  delay(300);
}


void checkIR() {
  if (!IrReceiver.decode()) return;
  /*
    * Print a summary of received data
    */
  if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
    Serial.println(F("Received noise or an unknown (or not yet enabled) protocol"));
    // We have an unknown protocol here, print extended info
    IrReceiver.printIRResultRawFormatted(&Serial, true);
    IrReceiver.resume();  // 다음 IR 프레임 수신을 위해 버퍼 리셋
    return;
  } else {
    IrReceiver.resume(); // 다음 IR 프레임을 조기에 수신할 수 있도록 활성화
    IrReceiver.printIRResultShort(&Serial);
    IrReceiver.printIRSendUsage(&Serial);
  }

  if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) {
    IrReceiver.resume();
    return;
  }

  unsigned long cmd = IrReceiver.decodedIRData.command;

  // 디바운스: 마지막 처리 후 일정 시간 Pass 여부
  // if (millis() - lastIRTime < IR_DEBOUNCE) {
  //   IrReceiver.resume();
  //   return;
  // }
  // lastIRTime = millis();

  switch (cmd) {
    case 69:
      // 버튼1
      motorRunning = false;
      atRightEnd = true;
      currentStatus = "Start Rdy";
      playMelody(starMelody, starDurations, starLen);
      break;
    case 70:
      // 버튼2
      motorRunning = false;
      atRightEnd = false;
      currentStatus = "End.";
      playMelody(lgMelody, lgDurations, lgLen);
      break;
    case 13:
      // 버튼#
      motorRunning = false;
      currentStatus = "Stopped";
      playBeep(2000);
      break;
    case 28:
      // OK: tracking_delay reset 후 재구동
      if (atRightEnd) {
        moveRight = false;
        tracking_delay = default_delay;
        currentStatus = "Tracking..";
      } else {
        moveRight = true;
        tracking_delay = MAX_DELAY;
        currentStatus = "Mov B Pos";
      }
      motorRunning = true;
      break;
    case 90:
      // 오른쪽 화살표
      playBeep(1000);
      moveRight = true;
      tracking_delay = MAX_DELAY;
      motorRunning = true;
      currentStatus = "Mov B Pos";
      break;
    case 8:
      // 왼쪽 화살표
      playBeep(1000);
      moveRight = false;
      tracking_delay = MAX_DELAY;
      motorRunning = true;
      currentStatus = "Mov E Pos";
      break;
    case 24:
      // 위 화살표: 속도 증가
      tracking_delay = max((long)tracking_delay - 50, 50L);
      Serial.println(tracking_delay);
      break;
    case 82:
      // 아래 화살표: 속도 감소
      tracking_delay += 50;
      Serial.println(tracking_delay);
      break;
    case 25:
      // 0: 속도 저장
      default_delay = tracking_delay;
      break;
    default:
      currentStatus = "Unknown Cmd";
      break;
  }
  IrReceiver.resume();

  updateDisplay();
}
// 두 모터에 동시에 한 스텝 펄스 발생
void stepMotors() {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(100);  // 짧은 펄스 지속시간
  digitalWrite(stepPin, LOW);
}

// 모터 정지
void stopMotors() {
  motorRunning = false;
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0, 0);
  display.println(currentStatus);
  display.print("T: ");
  display.println(tracking_delay);
  display.print("D: ");
  display.print(default_delay);
  display.display();
  Serial.println(currentStatus);
  delay(10);
}

void playMelody(int *melody, int *durations, int length) {
  for (int i = 0; i < length; i++) {
    tone(piezoPin, melody[i], durations[i]);
    delay(durations[i] * 1.3);
    noTone(piezoPin);
  }
}

void playBeep(int duration) {
  tone(piezoPin, 1000);
  delay(duration);
  noTone(piezoPin);
}