#define NOTE_AS4 466
#define NOTE_CS5 554
#define NOTE_DS5 622
#define NOTE_F5  698
#define NOTE_FS5 740

#define NOTE_GS4  415
#define NOTE_B4   494

#define NOTE_GS5 831

const int speakerPin = 9;

int lg_melody[] = {
  NOTE_CS5, // C#5
  NOTE_FS5, // F#5
  NOTE_F5,  // F5 (E#5로 읽음)
  NOTE_DS5, // D#5
  NOTE_CS5, // C#5
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

int lg_melody_durations[] = {
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

void setup() {
  for (int i = 0; i < sizeof(lg_melody) / sizeof(int); i++) {
    tone(speakerPin, lg_melody[i], lg_melody_durations[i]);
    delay(lg_melody_durations[i] * 1.2);  // 음과 음 사이 여유
  }
}

void loop() {
  // 반복 없음
}
