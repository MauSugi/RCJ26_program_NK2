#include <Arduino.h>
#include <math.h>

float radtodeg(float rad) {
  return rad * 180.0f / PI;
}
float degtorad(float deg) {
  return deg * PI / 180.0f;
}

#include <Adafruit_NeoPixel.h>
#define LED_PIN   PA0  // データ送信用のピン（NucleoのPA0ピン）
#define LED_COUNT 12   // 接続しているLEDの数

// インスタンスの作成：LEDの数、ピン番号、色の並び順(GRB)＋通信速度(800KHz)を指定
Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

const int IR_pins[12] = {//ピン配置の定義
  //前から時計回りに0から11
  PC0, PC1, PC2, PC3, PC4, PC5, PA4, PA5, PA6, PA7, PB0, PB1
};
int IR_data[12]; //センサー値保存用配列

// 赤外線センサーの値をシリアルに表示する関数
void print_IR_data() {
  for(int i=0; i<12; i++) {
    Serial.print(analogRead(IR_pins[i]));
    Serial.print("  ");
  }
  Serial.println("\n");
}

// ボールの角度(-180 ~ 180)を計算して返す関数
float get_IR_angle() {
  float sum = 0;
  float x = 0;
  float y = 0;

  for (int i = 0; i < 12; i++) {
    //センサー値の読み取り
    IR_data[i] = analogRead(IR_pins[i]);

    // 値の反転（1023が最大、0が最小とする）
    int val = map(IR_data[i], 0, 1023, 1023, 0);
    sum += val;

    float angle_rad = (PI / 6.0) * i; // i * 30度
    x += (float)val * cos(angle_rad);
    y += (float)val * sin(angle_rad);
  }

  // ボールが遠すぎるか存在しないとき
  if (sum < 1200) {
    return NAN; // NAN を返す
  }

  // atan2 は標準で -PI 〜 PI (-180 〜 180度) を返す
  return degrees(atan2(y, x));
}

// デバッグ用NeoPixel表示関数
void debug_neopixel(float angle, int color) {
  pixels.clear(); // 全データを一旦「オフ」に設定

  if (!isnan(angle)) {
    int led_index = round(angle / 30.0f);
    if (led_index < 0) {
      led_index += 12;
    }
    led_index = led_index % 12;

    // 色を設定(0:赤, 1:緑)
    switch (color) {
    case 0:
      pixels.setPixelColor(led_index, pixels.Color(30, 0, 0));
      break;
    case 1:
      pixels.setPixelColor(led_index, pixels.Color(0, 30, 0));
      break;
    default:
      break;
    }
  }

  pixels.show();  // 設定したデータを実際にLEDに送信
}

void setup() {
  //Serial.begin(115200);

  pixels.begin(); // 通信開始
  pixels.clear(); // 全データを一旦「オフ」に設定
  pixels.show();  // 設定したデータ（オフ）を実際にLEDに送信して消灯させる

  for (int i = 0; i < 12; i++) {
    pinMode(IR_pins[i], INPUT);
  }
}

void loop() {
  float ball_angle;
  ball_angle = get_IR_angle();
  //Serial.println(ball_angle);
  debug_neopixel(ball_angle, 1);
  delay(10);
}

