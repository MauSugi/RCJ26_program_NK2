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
};

// 現在のモードを保持
RobotMode currentMode = MODE_DEBUG; 
DebugMode currentDebug = DEBUG_BNO;

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
const float target_angle = 0.0f;
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
HardwareSerial lineSerial(PA3, PA2); // RX: PA3, TX: PA2
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
    }
  }
}

HardwareSerial ballSerial(PC11, PC10); // RX: PC11, TX: PC10
// デバッグ用にボールマイコンにラインデータを送信する関数
void send_to_ball_linedata(uint16_t data) {
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

// 角度（float）を2バイトに圧縮してボールマイコンへ送信する関数
void send_to_ball_BNOdata(float angle) {
  // -180.00 ～ 180.00 を -18000 ～ 18000 の整数に変換
  // これにより小数点第2位まで保持したまま16bitに収まる
  int16_t flat_angle = (int16_t)(angle * 100.0f);
  
  // 送信用にuint16_tとして解釈
  uint16_t data = (uint16_t)flat_angle;

  uint8_t high = (data >> 8) & 0xFF; 
  uint8_t low  = data & 0xFF;
  uint8_t header = 0xAC; // 角度データ用ヘッダー

  // チェックサム計算
  uint8_t checksum = (uint8_t)(header + high + low);

  // まとめて送信
  ballSerial.write(header);
  ballSerial.write(high); 
  ballSerial.write(low);
  ballSerial.write(checksum);
}
// システム状態（全体モードとデバッグ項目）をまとめてボールマイコンへ送信する関数
void send_to_ball_system_status() {
  uint8_t header = 0xAE;
  uint8_t high = (uint8_t)currentMode;   // RobotModeを送信
  uint8_t low  = (uint8_t)currentDebug;  // DebugModeを送信
  uint8_t checksum = (uint8_t)(header + high + low);

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

//ボタンピンの定義
const int button_pins[4]{
  PB0, PB1, PB2, PB3
};
// ボタン管理用のグローバル変数
bool btn_now[4] = {false, false, false, false};      // 今押されているか（生データ）
bool btn_pressed[4] = {false, false, false, false};  // 押された瞬間だけtrue
void update_buttons() {
    static bool last_btn_state[4] = {false, false, false, false};
    static unsigned long last_debounce[4] = {0, 0, 0, 0};

    for (int i = 0; i < 4; i++) {
        bool raw = digitalRead(button_pins[i]); // 抵抗の接続に合わせてHIGH/LOW調整
        btn_pressed[i] = false; // 毎ループリセット

        if (millis() - last_debounce[i] > 20) { // 20msチャタリング防止
            if (raw != last_btn_state[i]) {
                if (raw == HIGH) { // 立ち上がり（押された瞬間）
                    btn_pressed[i] = true;
                }
                last_btn_state[i] = raw;
                last_debounce[i] = millis();
            }
        }
        btn_now[i] = last_btn_state[i];
    }
}
// モード切り替えロジック
void handle_mode_logic() {
  // ボタン0：デバッグ項目の切り替え (BALL -> LINE -> BNO)
  if (btn_pressed[0]) {
    currentDebug = (DebugMode)((currentDebug + 1) % 3);
    // モードが変わったことをすぐに通知
    send_to_ball_system_status();
  }

  // ボタン1：全体モードの切り替え (READY -> NORMAL -> DEBUG -> READY)
  /*
  if (btn_pressed[1]) {
    if (currentMode == MODE_READY) currentMode = MODE_NORMAL;
    else if (currentMode == MODE_NORMAL) currentMode = MODE_DEBUG;
    else currentMode = MODE_READY;
    
    // モードが変わったことをすぐに通知
    send_to_ball_system_status();
  }*/
}


// PCデバッグ用
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


// メインストリーム
void setup() {
  Serial.begin(115200);
  lineSerial.begin(115200);
  ballSerial.begin(115200);
  delay(3000);
  //Serial.println("starting...");
  for (int i = 0; i < 8; i++) {
    pinMode(motor_pins[i], OUTPUT);
  }
  for (int i = 0; i < 4; i++){
    pinMode(button_pins[i], INPUT);
  }

  I2C_BNO.begin();
  BNO_init();
  //Serial.println("BNO055 initialized.");
  Mstop();
}

void loop() {
    // センサー更新
    update_buttons();
    handle_mode_logic();
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
        switch (currentDebug) {
          case DEBUG_BALL:
            send_to_ball_system_status();
            break;
          case DEBUG_BNO:
            send_to_ball_BNOdata(current_yaw);
            break;
          case DEBUG_LINE:
            send_to_ball_linedata(line_data);
            break;
        }
        Mstop();
        break;
      case MODE_STOP:
        // 後で作る
        break;
    }

    pre_error = error;
    last_micros = current_micros;
    delay(1); 
}