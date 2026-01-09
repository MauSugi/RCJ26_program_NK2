#include <Arduino.h>
#include <math.h>
float radtodeg(float rad) {
  return rad * 180.0f / PI;
}
float degtorad(float deg) {
  return deg * PI / 180.0f;
}

// モードの定義
enum RobotMode {
  MODE_READY,     // 準備中
  MODE_NORMAL,    // 通常走行
  MODE_DEBUG,     // デバッグモード
  MODE_STOP       // 停止
};

// デバッグモードの定義
enum DebugMode {
  DEBUG_BALL,
  DEBUG_LINE,
  DEBUG_BNO,
  DEBUG_MOTOR
};

// 現在のモードを保持
RobotMode currentMode = MODE_DEBUG; 
DebugMode currentDebug = DEBUG_BNO;

const int line_TH[12] = {
  400, 450, 360, 530, 430, 330,
  310, 420, 340, 310, 300, 390
};

// 1. 使用するピンを配列で定義
const int line_pins[15] = {
  //前から時計回りに0から11まで
  PA4, PA5, PA6, PA7, PB0, PB1, PC0, PC1, PC2, PC3, PC4, PC5,
  //サイドライン
  PC12, PC13, PC14
};

// アナログ値を格納する配列
int line_analog_data[12];
//　ラインデータを２値化して格納する16ビットの変数(下位15ビット使用)
uint16_t line_data = 0;
uint16_t last_sent_data = 0xFFFF; // 前回のデータを保持

// PCデバッグ用
void print_analog_data() {
  for (int i = 0; i < 12; i++) {
    Serial.print(line_analog_data[i]);
    Serial.print("  ");
  }
  Serial.println("");
}
void print_line_data() {
  for (int i = 0; i < 15; i++) {
    if (line_data & (1 << i)) {
      Serial.print("1 ");
    } else {
      Serial.print("0 ");
    }
  }
  Serial.println("");
}

// メインに送信する関数(4byte:ヘッダー1byte + データ2byte + チェックサム1byte)
void send_to_main(uint16_t data) {
  // バイト分解する
  uint8_t high = (data >> 8) & 0xFF; 
  uint8_t low  = data & 0xFF;
  uint8_t header = 0xAA;

  // チェックサムを計算（ヘッダー + 上位 + 下位）
  uint8_t checksum = (uint8_t)(header + high + low);

  // まとめて送信
  Serial.write(header);
  Serial.write(high); 
  Serial.write(low);
  Serial.write(checksum);
}
// メインからモード状態を受信する関数
void receive_from_main() {
  while (Serial.available() >= 4) {
    uint8_t header = Serial.read();
    if (header != 0xAE) continue;

    uint8_t high = Serial.read();
    uint8_t low  = Serial.read();
    uint8_t checksum = Serial.read();

    if ((uint8_t)(header + high + low) != checksum) continue;

    currentMode = (RobotMode)high;
    currentDebug = (DebugMode)low;
  }
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  // 2. 配列を使って一括でピンモードを設定
  for (int i = 0; i < 15; i++) {
    pinMode(line_pins[i], INPUT);
  }
}

void loop() {
  // モードの確認
  receive_from_main();
  
  line_data = 0; //ラインデータ初期化

  // アナログライン（アナログ12個）の処理
  for (int i = 0; i < 12; i++) {
    line_analog_data[i] = analogRead(line_pins[i]);
    // 閾値判定
    if (line_analog_data[i] < line_TH[i]) {
      line_data |= (1 << i);
    }
  }

  // サイドライン（デジタル3個）の処理
  // 信頼度によってはなくす
  /*
  for (int i = 12; i < 15; i++) {
    if (digitalRead(line_pins[i]) == LOW) { // センサーがLOWで反応する場合
      line_data |= (1 << i);
    }
  }
  */
 
  // --- 送信条件の判定 ---
  // NORMALモード時、または DEBUGモードかつLINEデバッグ選択時のみ送信
  bool should_send = (currentMode == MODE_NORMAL) || (currentMode == MODE_DEBUG && currentDebug == DEBUG_LINE);

  if (should_send) {
    // 「データが変化したとき」または「50msに1回（生存確認）」送信
    static unsigned long last_send_time = 0;
    if (line_data != last_sent_data || millis() - last_send_time > 50) {
      send_to_main(line_data);
      last_sent_data = line_data;
      last_send_time = millis();
    }
  } else {
    // 送信しないときは、last_sent_dataを初期化しておくと
    // 次に送信モードに切り替わった瞬間に最新データが即座に送られます
    last_sent_data = 0xFFFF; 
  }

  delay(1); // ループは高速に回す
  //print_analog_data();
  //print_line_data();
}