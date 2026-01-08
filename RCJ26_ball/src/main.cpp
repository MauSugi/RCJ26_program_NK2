#include <Arduino.h>
#include <math.h>

// --- 定数・列挙型の定義 ---
enum RobotMode {
  MODE_READY,     // 準備中
  MODE_NORMAL,    // 通常走行
  MODE_DEBUG,     // デバッグモード
  MODE_STOP       // 停止
};

enum DebugMode {
  DEBUG_BALL,
  DEBUG_LINE,
  DEBUG_BNO,
};

// 現在の状態を保持（メインマイコンと同期）
RobotMode currentMode = MODE_DEBUG; 
DebugMode currentDebug = DEBUG_BNO;


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
  if (sum < 1200) {
    IR_angle = NAN;
    IR_distance = sum;
    return;
  }
  IR_angle = degrees(atan2(y, x));
  IR_distance = sum;
}

// --- NeoPixel関連 ---
#include <Adafruit_NeoPixel.h>
#define LED_PIN   PA0
#define LED_COUNT 12
Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

uint16_t line_data = 0;
float current_yaw = 0.0f;

// LED表示の更新関数
void update_led_display() {
  pixels.clear();

  // 1. ロボットが「準備中」のとき：全体を薄く青く光らせる（生存確認）
  if (currentMode == MODE_READY) {
    for(int i=0; i<12; i++) pixels.setPixelColor(i, pixels.Color(0, 0, 70));
  }
  
  // 2. 「デバッグモード」のとき：項目の内容を表示
  else if (currentMode == MODE_DEBUG) {
    switch (currentDebug) {
      case DEBUG_BALL:
        if (!isnan(IR_angle)) {
          int idx = (int)(round(IR_angle / 30.0f) + 12) % 12;
          pixels.setPixelColor(idx, pixels.Color(70, 0, 0)); // 赤色：ボール
        }
        break;
      case DEBUG_LINE:
        for (int i = 0; i < 12; i++) {
          if (line_data & (1 << i)) pixels.setPixelColor(i, pixels.Color(70, 70, 70)); // 白色：ライン
        }
        break;
      case DEBUG_BNO:
        float yaw_debug = - current_yaw;
        int idx = (int)(round(yaw_debug / 30.0f) + 12) % 12;
        pixels.setPixelColor(idx, pixels.Color(0, 70, 0)); // 緑色：ジャイロ正面
        break;
    }
  }
  
  // 3. 「停止」のとき：全体を赤く
  else if (currentMode == MODE_STOP) {
    for(int i=0; i<12; i++) pixels.setPixelColor(i, pixels.Color(70, 0, 0));
  }

  // ※NORMALモードの時は基本消灯（またはボール方向のみ薄く光らせる等）
  pixels.show();
}

// --- 通信関連 ---
HardwareSerial ballSerial(PA3, PA2);
void receive_from_main() {
  while (ballSerial.available() >= 4) {
    uint8_t header = ballSerial.read();
    if (header != 0xAA && header != 0xAC && header != 0xAE) continue;

    uint8_t high = ballSerial.read();
    uint8_t low  = ballSerial.read();
    uint8_t checksum = ballSerial.read();

    if ((uint8_t)(header + high + low) != checksum) continue;

    uint16_t combined_val = (uint16_t)((high << 8) | low);

    switch (header) {
      case 0xAA: // ラインセンサー
        line_data = combined_val;
        break;
      case 0xAC: // BNO角度
        current_yaw = (float)((int16_t)combined_val) / 100.0f;
        break;
      case 0xAE: // システム状態の更新 (High: RobotMode, Low: DebugMode)
        currentMode = (RobotMode)high;
        currentDebug = (DebugMode)low;
        break;
    }
  }
}

void setup() {
  ballSerial.begin(115200);
  delay(1000);
  pixels.begin();
  pixels.setBrightness(100); // 明るさ設定
  for (int i = 0; i < 12; i++) pinMode(IR_pins[i], INPUT);
}

void loop() {
  calc_IR_data();
  receive_from_main();
  update_led_display(); // モードに合わせたLED表示
  delay(1);
}