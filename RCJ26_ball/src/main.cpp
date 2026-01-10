#include <Arduino.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>

// --- 定数・列挙型の定義 ---
enum RobotMode { MODE_READY, MODE_NORMAL, MODE_DEBUG, MODE_STOP };
enum DebugMode { DEBUG_BALL, DEBUG_LINE, DEBUG_BNO, DEBUG_MOTOR };

RobotMode currentMode = MODE_READY; 
DebugMode currentDebug = DEBUG_BALL;
bool is_calibrating = false; // 調整中フラグ
int selectedSlot = 0; // 0~11


// --- IRセンサー関連 ---
const int IR_pins[12] = { PC0, PC1, PC2, PC3, PC4, PC5, PA4, PA5, PA6, PA7, PB0, PB1 };
int IR_analog_data[12];
float IR_angle = NAN;
int IR_distance = 0;

void calc_IR_data() {
  float sum = 0, x = 0, y = 0;
  for (int i = 0; i < 12; i++) {
    IR_analog_data[i] = analogRead(IR_pins[i]);
    int val = map(IR_analog_data[i], 0, 1023, 1023, 0);
    sum += val;
    float angle_rad = (PI / 6.0) * i;
    x += (float)val * cos(angle_rad);
    y += (float)val * sin(angle_rad);
  }
  if (sum < 500) {
    IR_angle = NAN;
    IR_distance = (int)sum;
    return;
  }
  IR_angle = degrees(atan2(y, x));
  IR_distance = (int)sum;
}

float normalize_angle(float angle) {
  while (angle > 180.0f)  angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

// --- NeoPixel関連 ---
#define LED_PIN   PA0
#define LED_COUNT 12
Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

uint16_t line_data = 0;
float current_yaw = 0.0f;

void update_led_display() {
  static RobotMode lastMode = MODE_READY;
  static bool lastCalib = false;
  static bool already_cleared = false;

  // モードや調整状態が変化したかチェック
  if (currentMode != lastMode || is_calibrating != lastCalib) {
    already_cleared = false; 
    lastMode = currentMode;
    lastCalib = is_calibrating;
  }

  // 1. 最優先：消灯判定（NORMALモードまたは調整中）
  if (currentMode == MODE_NORMAL || is_calibrating) {
    if (!already_cleared) {
      pixels.clear();
      pixels.show();
      already_cleared = true; 
    }
    return; // 消灯中はこれ以降の処理をしない
  }

  // 2. 更新頻度の制限（約30msごと）
  static unsigned long last_led_update = 0;
  if (millis() - last_led_update < 30) return;
  last_led_update = millis();

  // 消灯フラグをリセット（表示モードに入っているため、次回NORMAL時に消灯させる準備）
  already_cleared = false; 

  pixels.clear();

  // 3. モード別の表示処理
  if (currentMode == MODE_READY) {
    // --- READYモード：指定されたスロットのみ点滅、他は緑点灯 ---
    static unsigned long last_blink = 0;
    static bool blink_state = true;
    
    // 200msごとに点滅状態を反転
    if (millis() - last_blink > 500) {
      blink_state = !blink_state;
      last_blink = millis();
    }

    for(int i=0; i<12; i++) {
      if (i == selectedSlot) {
        // 選択スロット：blink_stateがtrueの時だけ緑、falseの時は消灯（点滅）
        if (blink_state) pixels.setPixelColor(i, pixels.Color(0, 50, 0));
        else             pixels.setPixelColor(i, pixels.Color(0, 0, 0));
      } else {
        // それ以外：常時緑
        pixels.setPixelColor(i, pixels.Color(0, 50, 0));
      }
    }
  }
  else if (currentMode == MODE_DEBUG) {
    switch (currentDebug) {
      case DEBUG_BALL:
        if (!isnan(IR_angle)) {
          int idx = (int)(round(IR_angle / 30.0f) + 12) % 12;
          pixels.setPixelColor(idx, pixels.Color(50, 0, 0));
        }
        break;
      case DEBUG_LINE:
        for (int i = 0; i < 12; i++) {
          if (line_data & (1 << i)) pixels.setPixelColor(i, pixels.Color(50, 50, 50));
        }
        break;
      case DEBUG_BNO: {
        float yaw_debug = -current_yaw;
        int idx = (int)(round(yaw_debug / 30.0f) + 12) % 12;
        pixels.setPixelColor(idx, pixels.Color(50, 0, 30)); 
      } break;
      case DEBUG_MOTOR:
        for(int i=0; i<12; i++) pixels.setPixelColor(i, pixels.Color(20, 0, 20)); 
        break;
    }
  }
  else if (currentMode == MODE_STOP) {
    for(int i=0; i<12; i++) pixels.setPixelColor(i, pixels.Color(50, 0, 0));
  }

  pixels.show();
}

// --- 通信関連 ---
HardwareSerial ballSerial(PA3, PA2);

void receive_from_main() {
  while (ballSerial.available() >= 4) {
    uint8_t header = ballSerial.read();
    if (header != 0xAA && header != 0xAC && header != 0xAE && header != 0xAF && header != 0xAD) continue;

    uint8_t high = ballSerial.read();
    uint8_t low  = ballSerial.read();
    uint8_t checksum = ballSerial.read();

    if ((uint8_t)(header + high + low) != checksum) continue;

    uint16_t combined_val = (uint16_t)((high << 8) | low);

    switch (header) {
      case 0xAA: line_data = combined_val; break;
      case 0xAC: current_yaw = (float)((int16_t)combined_val) / 100.0f; break;
      case 0xAE: currentMode = (RobotMode)high; currentDebug = (DebugMode)low; break;
      case 0xAF: is_calibrating = (high == 1); break;
      case 0xAD: selectedSlot = (int)high; break; // スロット番号を保存
    }
  }
}

void send_to_main_balldata(float angle, int dist) {
  uint8_t header = 0xA1;
  uint8_t angle_byte = 0;
  uint8_t status_byte = 0;

  if (isnan(angle)) {
    angle_byte = 0;
    status_byte = 0; 
  } else {
    angle_byte = (uint8_t)((normalize_angle(angle) + 180.0f) * (255.0f / 360.0f));
    status_byte = 0x80; // ボールありフラグ
    uint8_t dist_7bit = (uint8_t)constrain(dist / 55, 0, 127);
    status_byte |= dist_7bit;
  }

  uint8_t checksum = (uint8_t)(header + angle_byte + status_byte);
  ballSerial.write(header);
  ballSerial.write(angle_byte);
  ballSerial.write(status_byte);
  ballSerial.write(checksum);
}

void setup() {
  pixels.begin();
  pixels.setBrightness(70);
  pixels.clear();
  pixels.show();
  
  for(int i=0; i<12; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 70, 70));
    pixels.show();
    delay(100);
  }
  
  ballSerial.begin(115200);
  for (int i = 0; i < 12; i++) pinMode(IR_pins[i], INPUT);

  delay(800);
  pixels.clear();
  pixels.show();
}

void loop() {
  calc_IR_data();
  receive_from_main();

  // --- 送信条件の判定 ---
  // NORMALモード時、または DEBUGモードかつBALLデバッグ選択時のみメインへ送信
  bool should_send = (currentMode == MODE_NORMAL) || (currentMode == MODE_DEBUG && currentDebug == DEBUG_BALL);

  if (should_send) {
    static unsigned long last_send_time = 0;
    if (millis() - last_send_time >= 10) { 
      send_to_main_balldata(IR_angle, IR_distance);
      last_send_time = millis();
    }
  }

  update_led_display();
  delay(1);
}