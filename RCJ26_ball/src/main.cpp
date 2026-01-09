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
  if (sum < 500) {
    IR_angle = NAN;
    IR_distance = sum;
    return;
  }
  IR_angle = degrees(atan2(y, x));
  IR_distance = sum;
}

/* メモ */
//IR_distance は (0 ~ 7000ほど)
//ボールなしで100から200
//ボール遠いかつ遮蔽ありで500あたり
//ボール遠いかつ遮蔽なしで2000あたり
//ボール中くらいで3000遮蔽ありで1300あたり
//以上の結果から
//IR_distanceが500以上1000未満でボールあり遮蔽あり
//IR_distanceが1000以上4000未満で遠~中くらいの距離
//IR_distanceが4000以上で至近距離
//としてもいいかも？

// --- NeoPixel関連 ---
#include <Adafruit_NeoPixel.h>
#define LED_PIN   PA0
#define LED_COUNT 12
Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

uint16_t line_data = 0;
float current_yaw = 0.0f;

// LED表示の更新関数
void update_led_display() {
  // 試合モード(NORMAL)の時は、通信速度を最優先するため、LED更新自体をスキップする
  // もし試合中に「ボールの方向だけ出したい」なら、ここを外して
  // 下のタイマー処理で頻度を落とすのがおすすめです。
  if (currentMode == MODE_NORMAL) {
    return; 
  }

  // 試合中以外でも、毎ループ show() を呼ぶのは重いので、
  // 約30ms（秒間30回）に制限する
  static unsigned long last_led_update = 0;
  if (millis() - last_led_update < 30) {
    return;
  }
  last_led_update = millis();

  pixels.clear();

  // --- 描画ロジック ---
  if (currentMode == MODE_READY) {
    for(int i=0; i<12; i++) pixels.setPixelColor(i, pixels.Color(0, 0, 70));
  }
  else if (currentMode == MODE_DEBUG) {
    switch (currentDebug) {
      case DEBUG_BALL:
        if (!isnan(IR_angle)) {
          int idx = (int)(round(IR_angle / 30.0f) + 12) % 12;
          pixels.setPixelColor(idx, pixels.Color(70, 0, 0));
        }
        break;
      case DEBUG_LINE:
        for (int i = 0; i < 12; i++) {
          if (line_data & (1 << i)) pixels.setPixelColor(i, pixels.Color(70, 70, 70));
        }
        break;
      case DEBUG_BNO:
        float yaw_debug = - current_yaw;
        int idx = (int)(round(yaw_debug / 30.0f) + 12) % 12;
        pixels.setPixelColor(idx, pixels.Color(0, 70, 0));
        break;
    }
  }
  else if (currentMode == MODE_STOP) {
    for(int i=0; i<12; i++) pixels.setPixelColor(i, pixels.Color(70, 0, 0));
  }

  pixels.show();
}

// 通信関連
HardwareSerial ballSerial(PA3, PA2);

void receive_from_main(){
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

// ボールデータを2byteにしてメインマイコンへ送信する関数
// ボールなしでも送信
void send_to_main_balldata(float angle, int dist) {
  // 引数 angle:(-180 ~ 180), dist:(0 ~ 7000) 
  uint8_t header = 0xA1;
  uint8_t angle_byte = 0;
  uint8_t status_byte = 0;

  if (isnan(angle)) {
    // ボールなしフラグ0
    angle_byte = 0;
    status_byte = 0; 
  } else {
    // (-180~180 -> 0~255)
    angle_byte = (uint8_t)((angle + 180.0f) * (255.0f / 360.0f));

    // 最上位ビット(Bit7)を1にする
    status_byte = 0x80; 
    
    // 残りの7bitに距離を詰める (0~7000 -> 0~127)
    uint8_t dist_7bit = (uint8_t)constrain(dist / 55, 0, 127);
    status_byte |= dist_7bit;
  }

  uint8_t checksum = (uint8_t)(header + angle_byte + status_byte);

  ballSerial.write(header);
  ballSerial.write(angle_byte);
  ballSerial.write(status_byte);
  ballSerial.write(checksum);
}

//HardwareSerial PCSerial(PA10, PA9);

void setup() {
  // 1. まずNeoPixelを初期化
  pixels.begin();
  pixels.setBrightness(70);
  pixels.clear();
  pixels.show(); // 一旦リセットを反映
  delay(100);     // 安定するまで少し待つ

  // 2. 青色をセット
  for(int i=0; i<12; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 100, 100));
    pixels.show();
    delay(100);
  }
  
  // その他の初期化
  ballSerial.begin(115200);
  //PCSerial.begin(115200);
  for (int i = 0; i < 12; i++) pinMode(IR_pins[i], INPUT);

  // 4. 指定の3秒間待機
  delay(1700);

  // 5. 消灯
  pixels.clear();
  pixels.show();

  //PCSerial.println("starting..");
}

void loop() {
  calc_IR_data();
  receive_from_main();

  // 10ms (秒間100回) ごとに送信する設定
  static unsigned long last_send_time = 0;
  if (millis() - last_send_time >= 10) { 
    send_to_main_balldata(IR_angle, IR_distance);
    last_send_time = millis();
  }

  update_led_display(); // モードに合わせたLED表示
  delay(1);
}
