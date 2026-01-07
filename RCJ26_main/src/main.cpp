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
  MODE_READY,      // 準備中
  MODE_NORMAL,    // 通常走行
  MODE_DEBUG,     // デバッグモード
  MODE_STOP       // 停止
};

// デバッグモードの定義
enum DebugMode {
  DEBUG_BALL,
  DEBUG_LINE,
  DEBUG_BNO,
};

// BNO055関連
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#define BNO055_ADDR 0x28
TwoWire I2C_BNO(PB7, PB6);
void write8(uint8_t reg, uint8_t val) {
  I2C_BNO.beginTransmission(BNO055_ADDR);
  I2C_BNO.write(reg);
  I2C_BNO.write(val);
  I2C_BNO.endTransmission();
}
uint8_t read8(uint8_t reg) {
  I2C_BNO.beginTransmission(BNO055_ADDR);
  I2C_BNO.write(reg);
  I2C_BNO.endTransmission(false); // falseにして接続を維持
  I2C_BNO.requestFrom((uint8_t)BNO055_ADDR, (uint8_t)1);
  return I2C_BNO.available() ? I2C_BNO.read() : 0;
}
// BNO055初期化関数
void BNO_init() {
  write8(0x3D, 0x00); // Config mode
  delay(25);
  write8(0x3F, 0x20); // Reset
  delay(700);
  write8(0x3D, 0x0C); // NDOF mode
  delay(25);
}
// 角度取得関数（yawを-180〜180で返す）
float BNO_get_yaw() {
  uint8_t l = read8(0x1A); // EULER_H_LSB
  uint8_t h = read8(0x1B); // EULER_H_MSB
  
  int16_t raw_heading = (int16_t)((h << 8) | l);
  float yaw = (float)raw_heading / 16.0f; // 1度 = 16 LSB

  // 0〜360度を -180〜180度に変換
  if (yaw > 180.0f) {
    yaw -= 360.0f;
  }
  return yaw;
}

// モーター関連
#define M_MAX 200.0f
const int motor_pins[8] = {//ピン配置の定義
  //M1からM4まで (PWM, GPIO) の順番
  PA8, PC0, PA0, PC1, PA1, PC2, PA6, PC3
};
void M1move(float speed) {
  analogWrite(motor_pins[0], 0);
  speed = round(speed);
  if (speed > 0) {
    digitalWrite(motor_pins[1], LOW);          // 正転
    analogWrite(motor_pins[0], speed);
  } else if (speed < 0) {
    digitalWrite(motor_pins[1], HIGH);         // 逆転
    analogWrite(motor_pins[0], -speed);
  } else {
    analogWrite(motor_pins[0], 0);             // 停止
  }
}
void M2move(float speed) {
  analogWrite(motor_pins[2], 0);
  speed = round(speed);
  if (speed > 0) {
    digitalWrite(motor_pins[3], HIGH);          // 正転
    analogWrite(motor_pins[2], speed);
  } else if (speed < 0) {
    digitalWrite(motor_pins[3], LOW);         // 逆転
    analogWrite(motor_pins[2], -speed);
  } else {
    analogWrite(motor_pins[2], 0);             // 停止
  }
}
void M3move(float speed) {
  analogWrite(motor_pins[4], 0);
  speed = round(speed);
  if (speed > 0) {
    digitalWrite(motor_pins[5], LOW);          // 正転
    analogWrite(motor_pins[4], speed);
  } else if (speed < 0) {
    digitalWrite(motor_pins[5], HIGH);         // 逆転
    analogWrite(motor_pins[4], -speed);
  } else {
    analogWrite(motor_pins[4], 0);             // 停止
  }
}
void M4move(float speed) {
  analogWrite(motor_pins[6], 0);
  speed = round(speed);
  if (speed > 0) {
    digitalWrite(motor_pins[7], HIGH);          // 正転
    analogWrite(motor_pins[6], speed);
  } else if (speed < 0) {
    digitalWrite(motor_pins[7], LOW);         // 逆転
    analogWrite(motor_pins[6], -speed);
  } else {
    analogWrite(motor_pins[6], 0);             // 停止
  }
}
// 全モーターを停止する関数
void Mstop() {
  M1move(0);
  M2move(0);
  M3move(0);
  M4move(0);
}
// 指定した角度と速度で全モーターを動かす関数(-180 ~ 180)
void Mmove(float angle, float spd_rate){
  M1move(sin(degtorad(angle-45.0f)) * spd_rate * M_MAX);
  M2move(sin(degtorad(angle-135.0f)) * spd_rate * M_MAX);
  M3move(sin(degtorad(angle-225.0f)) * spd_rate * M_MAX);
  M4move(sin(degtorad(angle-315.0f)) * spd_rate * M_MAX);
}
void Mspin(float spn_rate){
  M1move(spn_rate * M_MAX);
  M2move(spn_rate * M_MAX);
  M3move(spn_rate * M_MAX);
  M4move(spn_rate * M_MAX);
}

//姿勢(PID)制御関連
#define P_GAIN 4.0f
#define I_GAIN 1.0f
#define D_GAIN 1.0f
#define I_LIMIT 0.2f       // アンチワインドアップ（積分の最大値）
#define OUTPUT_LIMIT 1.0f  // 出力の最大値（M_MAXに対する割合）
float current_yaw = 0.0f;  // 現在のヨー角(Global)
float target_angle = 0.0f;
float pre_error = 0.0f;
float integral = 0.0f;
unsigned long last_micros = 0; // micros()管理用
// 角度を -180 〜 180 の範囲に正規化する関数
float normalize_angle(float angle) {
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

// ラインセンサー関連
HardwareSerial lineSerial(PA3, PA2); // RX, TX
// ラインマイコンから受信する関数
uint16_t line_data = 0; // 最新の正しいデータを保持
void receive_from_line() {
  // 4バイト以上溜まっている間、パケットを探し続ける
  while (lineSerial.available() >= 4) {
    // 1. ヘッダー(0xAA)を探す
    if (lineSerial.read() != 0xAA) {
      continue; // 0xAAでないなら読み飛ばして次へ
    }

    // 2. ヘッダーが見つかったら残りを読み込む
    uint8_t high = lineSerial.read();
    uint8_t low  = lineSerial.read();
    uint8_t checksum = lineSerial.read();

    // 3. チェックサム確認
    uint8_t calc_sum = (uint8_t)(0xAA + high + low);
    if (calc_sum == checksum) {
      // 正しいデータなら更新
      line_data = (uint16_t)((high << 8) | low);
    } else {
      // エラーならこのパケットは捨てる（何もしない）
    }
  }
}

HardwareSerial ballSerial(PC11, PC10); // TX, RX
// デバッグ用にボールマイコンに送信する関数(BNO,ライン,モード変更)
void send_to_ball(uint16_t data) {
  // バイト分解する
  uint8_t high = (data >> 8) & 0xFF; 
  uint8_t low  = data & 0xFF;
  uint8_t header = 0xAA;

  // チェックサムを計算（ヘッダー + 上位 + 下位）
  uint8_t checksum = (uint8_t)(header + high + low);

  // まとめて送信
  ballSerial.write(header);
  ballSerial.write(high); 
  ballSerial.write(low);
  ballSerial.write(checksum);
}

//　ボールマイコンから受信する関数(ボールデータ)
/* 作成中
void receive_from_ball() {
  // 4バイト以上溜まっている間、パケットを探し続ける
  while (Serial.available() >= 4) {
    // 1. ヘッダー(0xAA)を探す
    if (Serial.read() != 0xAA) {
      continue; // 0xAAでないなら読み飛ばして次へ
    }

    // 2. ヘッダーが見つかったら残りを読み込む
    uint8_t high = Serial.read();
    uint8_t low  = Serial.read();
    uint8_t checksum = Serial.read();

    // 3. チェックサム確認
    uint8_t calc_sum = (uint8_t)(0xAA + high + low);
    if (calc_sum == checksum) {
      // 正しいデータなら更新
      line_data = (uint16_t)((high << 8) | low);
    }
  }
}*/


void setup() {
  Serial.begin(115200);
  //delay(2000);
  Serial.println("starting...");
  for (int i = 0; i < 8; i++) {
    pinMode(motor_pins[i], OUTPUT);
  }

  I2C_BNO.begin();
  BNO_init();
  Serial.println("BNO055 initialized.");
  Mstop();
}

// 現在のモードを保持
RobotMode currentMode = MODE_DEBUG; 

void loop() {
    // センサー更新
    receive_from_line(); // ライン情報の受信
    //receive_from_ball(); // ボール情報の受信
    
    // 姿勢制御用の時間計測
    unsigned long current_micros = micros();
    float dt = (float)(current_micros - last_micros) / 1000000.0f;
    if (dt <= 0.0001f) { delay(1); return; }

    // 角度・PID制御計算
    float current_yaw = BNO_get_yaw();
    float error = normalize_angle(target_angle - current_yaw) / 180.0f;
    float p_out = P_GAIN * error;
    integral += error * dt;
    if (integral > I_LIMIT)  integral = I_LIMIT;
    else if (integral < -I_LIMIT) integral = -I_LIMIT;
    float i_out = I_GAIN * integral;
    float d_out = D_GAIN * (error - pre_error) / dt;
    float sisei_output = p_out + i_out + d_out;
    if (sisei_output > OUTPUT_LIMIT)  sisei_output = OUTPUT_LIMIT;
    else if (sisei_output < -OUTPUT_LIMIT) sisei_output = -OUTPUT_LIMIT;

    switch (currentMode) {
        case MODE_READY:
          // 準備中：特に動作なし
          break;
        case MODE_NORMAL:
          // 試合モード
          break;
        case MODE_DEBUG:
          // 作成中
          send_to_ball(line_data);
          //16ビット目が立っているならボールあり
          //if (line_data & 0x8000) {
              // ボールあり
          //}
          //Mstop();
          break;
        case MODE_STOP:
          // 後で作る
          break;
    }

    pre_error = error;
    last_micros = current_micros;
    delay(1); 
}