#include <Arduino.h>

// 1. 使用するピンを配列で定義
const int line_pins[12] = {
  //前から時計回りに0から11
  PA4, PA5, PA6, PA7, PB0, PB1, PC0, PC1, PC2, PC3, PC4, PC5
};

int line_data[12];

void setup() {
  Serial.begin(115200);

  // 2. 配列を使って一括でピンモードを設定
  for (int i = 0; i < 12; i++) {
    pinMode(line_pins[i], INPUT);
  }
}

void loop() {
  // 3. データの読み取り
  for (int i = 0; i < 12; i++) {
    line_data[i] = analogRead(line_pins[i]);
  }

  // 4. アナログ値の表示
  for (int i = 0; i < 12; i++) {
    Serial.print(i + 1);
    Serial.print(":");
    Serial.print(line_data[i]);
    Serial.print("  ");
    if (i == 5) Serial.print("| "); 
  }
  Serial.println("");

  delay(200);
}