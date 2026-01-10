#include <Arduino.h>
#include <math.h>

// モードの定義
enum RobotMode { MODE_READY, MODE_NORMAL, MODE_DEBUG, MODE_STOP };
enum DebugMode { DEBUG_BALL, DEBUG_LINE, DEBUG_BNO, DEBUG_MOTOR };

RobotMode currentMode = MODE_DEBUG; 
DebugMode currentDebug = DEBUG_BNO;

// --- 閾値関連 ---
int line_TH[12] = { 400, 450, 360, 530, 430, 330, 310, 420, 340, 310, 300, 390 };
bool is_calibrating = false;
int line_MAX[12];
int line_MIN[12];

// ピン配置
const int line_pins[15] = {
  PA4, PA5, PA6, PA7, PB0, PB1, PC0, PC1, PC2, PC3, PC4, PC5,
  PC12, PC13, PC14
};

int line_analog_data[12];
uint16_t line_data = 0;
uint16_t last_sent_data = 0xFFFF;

// --- 通信関数 ---
void send_to_main(uint16_t data) {
  uint8_t high = (data >> 8) & 0xFF; 
  uint8_t low  = data & 0xFF;
  uint8_t header = 0xAA;
  uint8_t checksum = (uint8_t)(header + high + low);

  Serial.write(header);
  Serial.write(high); 
  Serial.write(low);
  Serial.write(checksum);
}

// 調整終了時の計算処理
void finish_calibration() {
  for (int i = 0; i < 12; i++) {
    // 最大値（緑）と最小値（白）の中間値を計算
    // 確実に白を検知するため、中間より少し高い値（-5など）にオフセット
    if (line_MAX[i] > line_MIN[i]) {
      line_TH[i] = ((line_MAX[i] + line_MIN[i]) / 2) - 5;
    }
  }
}

void receive_from_main() {
  while (Serial.available() >= 4) {
    uint8_t header = Serial.read();
    if (header != 0xAE && header != 0xAF) continue;

    uint8_t high = Serial.read();
    uint8_t low  = Serial.read();
    uint8_t checksum = Serial.read();

    if ((uint8_t)(header + high + low) != checksum) continue;

    if (header == 0xAE) {
      currentMode = (RobotMode)high;
      currentDebug = (DebugMode)low;
    } 
    else if (header == 0xAF) {
      bool next_calib = (high == 1);
      
      // 調整開始(0->1)の時に最大最小を初期化
      if (!is_calibrating && next_calib) {
        for (int i = 0; i < 12; i++) {
          line_MAX[i] = 0;
          line_MIN[i] = 1023;
        }
      }
      // 調整終了(1->0)の時に閾値を計算
      if (is_calibrating && !next_calib) {
        finish_calibration();
      }
      is_calibrating = next_calib;
    }
  }
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 15; i++) {
    pinMode(line_pins[i], INPUT);
  }
  delay(2000);
}

void loop() {
  receive_from_main();
  
  line_data = 0;

  for (int i = 0; i < 12; i++) {
    line_analog_data[i] = analogRead(line_pins[i]);

    if (is_calibrating) {
      // 調整モード中：最大値と最小値を記録し続ける
      if (line_analog_data[i] > line_MAX[i]) line_MAX[i] = line_analog_data[i];
      if (line_analog_data[i] < line_MIN[i]) line_MIN[i] = line_analog_data[i];
    } else {
      // 通常モード：計算された閾値で二値化
      if (line_analog_data[i] < line_TH[i]) {
        line_data |= (1 << i);
      }
    }
  }

  // 送信処理（調整中は送信しない、または現在の判定を送る）
  if (!is_calibrating) {
    bool should_send = (currentMode == MODE_NORMAL) || (currentMode == MODE_DEBUG && currentDebug == DEBUG_LINE);

    if (should_send) {
      static unsigned long last_send_time = 0;
      if (line_data != last_sent_data || millis() - last_send_time > 50) {
        send_to_main(line_data);
        last_sent_data = line_data;
        last_send_time = millis();
      }
    } else {
      last_sent_data = 0xFFFF;
    }
  }

  delay(1);
}