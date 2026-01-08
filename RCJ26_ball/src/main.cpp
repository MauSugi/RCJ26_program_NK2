#include <Arduino.h>
#include <math.h>
float radtodeg(float rad) {
  return rad * 180.0f / PI;
}
float degtorad(float deg) {
  return deg * PI / 180.0f;
}

const int IR_pins[12] = {//ピン配置の定義
  //前から時計回りに0から11
  PC0, PC1, PC2, PC3, PC4, PC5, PA4, PA5, PA6, PA7, PB0, PB1
};
int IR_analog_data[12]; //センサー値保存用配列

// PCデバッグ用
void print_IR_data() {
  for(int i=0; i<12; i++) {
    Serial.print(analogRead(IR_pins[i]));
    Serial.print("  ");
  }
  Serial.println("\n");
}

float IR_angle = NAN; // ボールの角度(-180 ~ 180)を格納するグローバル変数
int IR_distance = 0; // ボールまでの距離を格納するグローバル変数
// 角度(-180 ~ 180)、距離(0～)を計算して変数を更新する関数
void calc_IR_data() {
  float sum = 0;
  float x = 0;
  float y = 0;

  for (int i = 0; i < 12; i++) {
    //センサー値の読み取り
    IR_analog_data[i] = analogRead(IR_pins[i]);

    // 値の反転（1023が最大、0が最小とする）
    int val = map(IR_analog_data[i], 0, 1023, 1023, 0);
    sum += val;

    float angle_rad = (PI / 6.0) * i; // i * 30度
    x += (float)val * cos(angle_rad);
    y += (float)val * sin(angle_rad);
  }

  // ボールが遠すぎるか存在しないとき
  if (sum < 1200) {
    IR_angle = NAN; // NAN を返す
    IR_distance = sum;
    return;
  }

  // atan2 は標準で -PI 〜 PI (-180 〜 180度) を返す
  IR_angle = degrees(atan2(y, x));
  IR_distance = sum;
}

// NeoPixel関連
#include <Adafruit_NeoPixel.h>
#define LED_PIN   PA0  // データ送信用のピン（NucleoのPA0ピン）
#define LED_COUNT 12   // 接続しているLEDの数
// インスタンスの作成：LEDの数、ピン番号、色の並び順(GRB)＋通信速度(800KHz)を指定
Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
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

// ラインセンサーのNeoPixel表示関数（複数対応）
void debug_line_neopixel(uint16_t line_data) {
  pixels.clear(); // 全データを一旦「オフ」に設定

  for (int i = 0; i < 12; i++) {
    if (line_data & (1 << i)) {
      pixels.setPixelColor(i, pixels.Color(30, 30, 30)); // 白色で点灯
    }
  }

  pixels.show();  // 設定したデータを実際にLEDに送信
}

// シリアル通信関連
uint16_t line_data = 0; // ラインセンサー情報保存用変数

HardwareSerial ballSerial(PA3, PA2); // RX, TX
void receive_from_main() {
  while (ballSerial.available() >= 4) {
    uint8_t header = ballSerial.read();

    if (header != 0xAA && header != 0xBB) {
      continue; // 不明なヘッダーなら無視
    }

    uint8_t high = ballSerial.read();
    uint8_t low  = ballSerial.read();
    uint8_t checksum = ballSerial.read();

    uint8_t calc_sum = (uint8_t)(header + high + low);
    if (calc_sum != checksum) {
      continue; // チェックサムエラーならこのパケットは無視
    }

    // 5. ヘッダーの種類に応じて格納先を変える
    uint16_t combined_val = (uint16_t)((high << 8) | low);

    switch (header) {
      case 0xAA: // ラインセンサーのデータ
        line_data = combined_val;
        break;

      case 0xBB: // ボールセンサーのデータ
        // 今回は使わない
        break;

      default:
        break;
    }
  }
}

void setup() {
  //Serial.begin(115200);
  ballSerial.begin(115200);
  delay(5000);
  pixels.begin(); // 通信開始
  pixels.clear(); // 全データを一旦「オフ」に設定
  pixels.show();  // 設定したデータ（オフ）を実際にLEDに送信して消灯させる

  for (int i = 0; i < 12; i++) {
    pinMode(IR_pins[i], INPUT);
  }
}

void loop() {
  /*
  uint16_t old_line_data = line_data;
  receive_from_main();

  // データが変化したときだけLEDを更新
  if (line_data != old_line_data) {
    debug_line_neopixel(line_data);
  }*/

  // メインマイコンからのデータを受信（line_dataが更新される）
  receive_from_main();

  // 受信したライン情報をそのままNeoPixelに表示
  // ※receive_from_mainでデータが来ない間は、最後に受信した値が保持されます
  debug_line_neopixel(line_data);

  delay(1); 
}


