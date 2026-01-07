#include <Arduino.h>

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
  if (data & 0x7FFF) { 
    data |= (1 << 15); // いずれかのラインが反応していればビット15を立てる
  }

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

void setup() {
  Serial.begin(115200);
  delay(5000);
  // 2. 配列を使って一括でピンモードを設定
  for (int i = 0; i < 15; i++) {
    pinMode(line_pins[i], INPUT);
  }
}

void loop() {
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
  
  //print_analog_data();
  //print_line_data();

  send_to_main(line_data);
  delay(1);
}