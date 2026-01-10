#include <Arduino.h>
#include <math.h>

// --- 定数・列挙型の定義 ---
enum RobotMode { MODE_READY, MODE_NORMAL, MODE_DEBUG, MODE_STOP };
enum DebugMode { DEBUG_BALL, DEBUG_LINE, DEBUG_BNO, DEBUG_MOTOR };

RobotMode currentMode = MODE_READY; 
DebugMode currentDebug = DEBUG_BNO;

// キャリブレーション管理
bool is_line_calibrating = false;
float bno_offset = 0.0f; // BNOの0点オフセット用

// フィールド復帰用
float prev_field_angle = -999.0f;
bool is_field_out = false;

// --- BNO055関連 ---
#include <Wire.h>
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
  I2C_BNO.endTransmission(false);
  I2C_BNO.requestFrom((uint8_t)BNO055_ADDR, (uint8_t)1);
  return I2C_BNO.available() ? I2C_BNO.read() : 0;
}

void BNO_init() {
  write8(0x3D, 0x00); delay(25);
  write8(0x3F, 0x20); delay(700);
  write8(0x3D, 0x0C); delay(25);
}

float BNO_get_raw_yaw() {
  uint8_t l = read8(0x1A);
  uint8_t h = read8(0x1B);
  int16_t raw_heading = (int16_t)((h << 8) | l);
  float yaw = (float)raw_heading / 16.0f;
  if (yaw > 180.0f) yaw -= 360.0f;
  return yaw;
}

float normalize_angle(float angle) {
  while (angle > 180.0f)  angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float get_calibrated_yaw() {
  return normalize_angle(BNO_get_raw_yaw() - bno_offset);
}

// --- 通信関連 (4バイトパケット) ---
HardwareSerial lineSerial(PA3, PA2);
HardwareSerial ballSerial(PC11, PC10);

void send_packet(HardwareSerial &serial, uint8_t header, uint8_t high, uint8_t low) {
  uint8_t checksum = (uint8_t)(header + high + low);
  serial.write(header);
  serial.write(high);
  serial.write(low);
  serial.write(checksum);
}

void send_system_status() {
  for (int i = 0; i < 2; i++) {
    send_packet(lineSerial, 0xAE, (uint8_t)currentMode, (uint8_t)currentDebug);
    send_packet(ballSerial, 0xAE, (uint8_t)currentMode, (uint8_t)currentDebug);
    delay(2); 
  }
}

void send_calib_signal(bool active) {
  uint8_t val = active ? 1 : 0;
  for (int i = 0; i < 2; i++) {
    send_packet(lineSerial, 0xAF, val, 0);
    send_packet(ballSerial, 0xAF, val, 0);
    delay(2);
  }
}

// --- モーター制御 ---
#define M_MAX 250.0f
const int motor_pins[8] = { PA8, PC0, PA0, PC1, PA1, PC2, PA6, PC3 };

void M1move(float speed) {
  speed = round(constrain(speed, -M_MAX, M_MAX));
  if (speed > 0) { digitalWrite(motor_pins[1], LOW); analogWrite(motor_pins[0], (int)speed); }
  else if (speed < 0) { digitalWrite(motor_pins[1], HIGH); analogWrite(motor_pins[0], (int)-speed); }
  else analogWrite(motor_pins[0], 0);
}
void M2move(float speed) {
  speed = round(constrain(speed, -M_MAX, M_MAX));
  if (speed > 0) { digitalWrite(motor_pins[3], HIGH); analogWrite(motor_pins[2], (int)speed); }
  else if (speed < 0) { digitalWrite(motor_pins[3], LOW); analogWrite(motor_pins[2], (int)-speed); }
  else analogWrite(motor_pins[2], 0);
}
void M3move(float speed) {
  speed = round(constrain(speed, -M_MAX, M_MAX));
  if (speed > 0) { digitalWrite(motor_pins[5], LOW); analogWrite(motor_pins[4], (int)speed); }
  else if (speed < 0) { digitalWrite(motor_pins[5], HIGH); analogWrite(motor_pins[4], (int)-speed); }
  else analogWrite(motor_pins[4], 0);
}
void M4move(float speed) {
  speed = round(constrain(speed, -M_MAX, M_MAX));
  if (speed > 0) { digitalWrite(motor_pins[7], HIGH); analogWrite(motor_pins[6], (int)speed); }
  else if (speed < 0) { digitalWrite(motor_pins[7], LOW); analogWrite(motor_pins[6], (int)-speed); }
  else analogWrite(motor_pins[6], 0);
}

void Mstop() { M1move(0); M2move(0); M3move(0); M4move(0); }

void Mmove_with_spin(float angle, float spd_rate, float sisei_out) {
  float rad = (angle - 45.0f) * PI / 180.0f;
  float m1 = sin(rad) * spd_rate;
  float m2 = sin(rad - PI/2.0f) * spd_rate;
  float m3 = sin(rad - PI) * spd_rate;
  float m4 = sin(rad - 3.0f*PI/2.0f) * spd_rate;
  M1move((m1 + sisei_out) * M_MAX);
  M2move((m2 + sisei_out) * M_MAX);
  M3move((m3 + sisei_out) * M_MAX);
  M4move((m4 + sisei_out) * M_MAX);
}

void Mspin(float spn_rate) {
  M1move(spn_rate * M_MAX); M2move(spn_rate * M_MAX);
  M3move(spn_rate * M_MAX); M4move(spn_rate * M_MAX);
}

// --- ボタン管理 ---
const int button_pins[4] = { PB0, PB1, PB2, PB4 };
bool btn_pressed[4] = {false};
void update_buttons() {
  static bool last_state[4] = {false};
  for (int i = 0; i < 4; i++) {
    bool now = digitalRead(button_pins[i]);
    btn_pressed[i] = (now == HIGH && last_state[i] == LOW);
    last_state[i] = now;
  }
}

// --- 回り込み計算 ---
float get_orbit_angle(float ir_deg) {
  float abs_ir = abs(ir_deg);
  float orbit_abs;
  if (abs_ir <= 15.0f) orbit_abs = 0.0f;
  else if (abs_ir <= 40.0f) orbit_abs = (abs_ir - 15.0f) * 3.6f;
  else if (abs_ir <= 90.0f) orbit_abs = (abs_ir - 40.0f) * 1.6f + 90.0f;
  else orbit_abs = (abs_ir - 90.0f) * 0.666f + 170.0f;
  return normalize_angle((ir_deg >= 0) ? orbit_abs : -orbit_abs);
}


// --- メインループ変数 ---
float IR_angle = NAN;
uint16_t line_data = 0;
float current_yaw = 0.0f;
float pre_error = 0.0f, integral = 0.0f;
float sisei_output = 0.0f; // ★追加：姿勢制御の出力値を保存する変数
unsigned long last_micros = 0;
static const float K_P = 2.0f;     // 比例ゲイン
static const float K_I = 2.0f;     // 積分ゲイン
static const float K_D = 0.3f;     // 微分ゲイン
static const float I_LIMIT = 50.0f; // 積分項の蓄積上限 (ワインドアップ対策)

void reset_PID() {
  integral = 0.0f;
  pre_error = 0.0f;
  sisei_output = 0.0f; // 現在の出力値もクリア
}


void handle_mode_logic() {
  bool mode_changed = false;
  RobotMode nextMode = currentMode;

  // NORMALモードならどのボタンでも停止
  if (currentMode == MODE_NORMAL) {
    if (btn_pressed[0] || btn_pressed[1] || btn_pressed[2] || btn_pressed[3]) {
      nextMode = MODE_STOP; mode_changed = true;
    }
  }

  if (!(currentMode == MODE_DEBUG && currentDebug == DEBUG_MOTOR)) {
    // ボタン1: 調整 (BNOリセット時にもPIDリセット)
    if (btn_pressed[1] && currentMode == MODE_DEBUG) {
      if (currentDebug == DEBUG_LINE) {
        is_line_calibrating = !is_line_calibrating;
        send_calib_signal(is_line_calibrating);
      } else if (currentDebug == DEBUG_BNO) {
        send_calib_signal(true); delay(200);
        bno_offset = BNO_get_raw_yaw();
        reset_PID(); // キャリブレーション直後の跳ね上がりを防止
        send_calib_signal(false);
      }
    }
    // ボタン3: モード切替
    if (btn_pressed[3]) {
      if (currentMode == MODE_READY) nextMode = MODE_DEBUG;
      else if (currentMode == MODE_DEBUG) nextMode = MODE_STOP;
      else if (currentMode == MODE_STOP) nextMode = MODE_READY;
      mode_changed = true;
    }
    // ボタン2: 走行開始
    if (btn_pressed[2] && currentMode == MODE_READY) { nextMode = MODE_NORMAL; mode_changed = true; }
  }

  // ボタン0: デバッグ項目切替
  if (btn_pressed[0] && currentMode == MODE_DEBUG) {
    currentDebug = (DebugMode)((currentDebug + 1) % 4);
    mode_changed = true;
  }

  if (mode_changed) {
    // モードが変わる瞬間に一度だけPIDをリセット
    // 特にREADYやSTOPから他へ移る際、古い誤差を捨てる
    reset_PID();
    currentMode = nextMode;
    Mstop();
    send_system_status();
  }
}


// 12個のラインセンサーからフィールド中央方向を計算する（180度反転アルゴリズム対応）
float calculate_field_angle(uint16_t mask) {
  mask &= 0x0FFF; // 下位12ビット(0-11)を使用
  if (mask == 0) return NAN;

  int on_pins[12];
  int sum_pin = 0;

  // 1. 反応しているピンを抽出
  for (int i = 0; i < 12; i++) {
    if (mask & (1 << i)) {
      on_pins[sum_pin++] = i;
    }
  }

  // 2. 最大の隙間（暫定のフィールド中央）を探す
  int max_kankaku = 0;
  int idxmax = 0;
  for (int i = 0; i < sum_pin; i++) {
    int kankaku = (i < sum_pin - 1) ? (on_pins[i + 1] - on_pins[i]) : (on_pins[0] + 12 - on_pins[i]);
    if (kankaku > max_kankaku) {
      max_kankaku = kankaku;
      idxmax = i;
    }
  }

  // 3. 隙間の中央から計算した「今の方向」
  float target_pin = (float)on_pins[idxmax] + (float)max_kankaku * 0.5f;
  float current_field_angle = normalize_angle(target_pin * 30.0f);

  // 4. 180度反転（ライン突き抜け）の検知ロジック
  if (prev_field_angle > -360.0f) { // 前回のデータがある場合
    // 前回の方向と今回の方向の差を計算
    float diff = abs(normalize_angle(current_field_angle - prev_field_angle));

    // 差が180度付近（140〜220度）なら、ラインを跨いで外に出たと判定
    if (diff > 140.0f && diff < 220.0f) {
      is_field_out = true;
    } 
    // 逆に差が小さいなら、内側に戻ったと判定
    else if (diff < 70.0f) {
      is_field_out = false;
    }

    // 外にいる判定なら、向きを180度反転させてコート中央に向かせる
    if (is_field_out) {
      current_field_angle = normalize_angle(current_field_angle + 180.0f);
    }

    // 動きを滑らかにする（前回の値と70%:30%で混ぜる）
    current_field_angle = normalize_angle(prev_field_angle * 0.7f + current_field_angle * 0.3f);
  }

  prev_field_angle = current_field_angle;
  return current_field_angle;
}

// サブマイコンからの受信関数(既存の receive_from_line/ball を想定)
void receive_from_line() {
  while (lineSerial.available() >= 4) {
    if (lineSerial.read() != 0xAA) continue;
    uint8_t h = lineSerial.read(); uint8_t l = lineSerial.read(); uint8_t c = lineSerial.read();
    if ((uint8_t)(0xAA + h + l) == c) line_data = (uint16_t)((h << 8) | l);
  }
}

void receive_from_ball() {
  while (ballSerial.available() >= 4) {
    if (ballSerial.read() != 0xA1) continue;
    uint8_t ab = ballSerial.read(); uint8_t sb = ballSerial.read(); uint8_t c = ballSerial.read();
    if ((uint8_t)(0xA1 + ab + sb) == c) {
      if (!(sb & 0x80)) { IR_angle = NAN; }
      else {
        IR_angle = (float)ab * (360.0f / 255.0f) - 180.0f;
      }
    }
  }
}

void setup() {
  Serial.begin(115200); lineSerial.begin(115200); ballSerial.begin(115200);
  for (int i=0; i<8; i++) pinMode(motor_pins[i], OUTPUT);
  for (int i=0; i<4; i++) pinMode(button_pins[i], INPUT);
  I2C_BNO.begin(); BNO_init();
  Mstop();
  send_system_status();
  delay(2000);
  last_micros = micros();
}

void loop() {
  update_buttons();
  handle_mode_logic();
  receive_from_line();
  receive_from_ball();
  
  unsigned long cur = micros();
  float dt = (float)(cur - last_micros) / 1000000.0f;
  if (dt < 0.001f) return;
  last_micros = cur;

  current_yaw = get_calibrated_yaw(); // オフセット適用後の角度
  
  // PID計算 (定数を使用)
  float error = normalize_angle(0.0f - current_yaw);
  
  // 積分項：READYやSTOP、BNOキャリブ中以外のみ蓄積する
  if (currentMode == MODE_NORMAL || (currentMode == MODE_DEBUG && currentDebug == DEBUG_MOTOR)) {
    integral = constrain(integral + error * dt, -I_LIMIT, I_LIMIT);
  } else {
    integral = 0; // 走行中以外は常に0（ワインドアップ対策）
  }

  float derivative = (error - pre_error) / dt;
  sisei_output = (K_P * error + K_I * integral + K_D * derivative) / M_MAX;
  sisei_output = constrain(sisei_output, -1.0f, 1.0f);
  pre_error = error;

  switch (currentMode) {
    case MODE_READY:
      Mstop();
      break;
    case MODE_NORMAL: {

      // --- ラインセンサーの反応を確認 ---
      float avoid_angle = calculate_field_angle(line_data);

      if (line_data > 0) {
        Mmove_with_spin(avoid_angle, 1.0f, sisei_output);
      } 
      else {
        // --- 2. 通常走行（ボール追従） ---
        // ラインを踏んでいない場合のみ実行
        if (isnan(IR_angle)) {
          // ボールが見えない時はその場で回転して姿勢維持
          Mspin(sisei_output);
        } else {
          // ボールがある時は回り込み計算を行い、姿勢制御を混ぜて走行
          Mmove_with_spin(get_orbit_angle(IR_angle), 1.0f, sisei_output);
        }
      }
    } break;
    case MODE_DEBUG:
      switch (currentDebug) {
        case DEBUG_BALL:
          // 何もしない
          break;

        case DEBUG_LINE:
          static unsigned long lt_line = 0;
          if (millis() - lt_line > 30) {
            send_packet(ballSerial, 0xAA, (uint8_t)(line_data >> 8), (uint8_t)(line_data & 0xFF));
            lt_line = millis();
          }
          break;

        case DEBUG_BNO:
          static unsigned long lt_bno = 0;
          if (millis() - lt_bno > 30) {
            int16_t f = (int16_t)(current_yaw * 100.0f);
            send_packet(ballSerial, 0xAC, (uint8_t)(f >> 8), (uint8_t)(f & 0xFF));
            lt_bno = millis();
          }
          break;

        case DEBUG_MOTOR: {
          static int cw_state = 0;   // 0:停止, 1:M1, 2:M2, 3:M3, 4:M4 (時計回り用)
          static int ccw_state = 0;  // 0:停止, 1:M1, 2:M2, 3:M3, 4:M4 (反時計回り用)
          static bool pid_enabled = false;

          // --- ボタン処理 ---
          if (btn_pressed[1]) {
            cw_state = (cw_state + 1) % 5;
            ccw_state = 0; pid_enabled = false; Mstop();
          }
          if (btn_pressed[2]) {
            ccw_state = (ccw_state + 1) % 5;
            cw_state = 0; pid_enabled = false; Mstop();
          }
          if (btn_pressed[3]) {
            pid_enabled = !pid_enabled;
            cw_state = 0; ccw_state = 0; Mstop();
          }

          // --- 動作反映 ---
          if (pid_enabled) {
            Mspin(sisei_output); // 姿勢制御のテスト
          }
          else if (cw_state > 0) {
            float spd = 0.4f * M_MAX;
            if (cw_state == 1) M1move(spd);
            else if (cw_state == 2) M2move(spd);
            else if (cw_state == 3) M3move(spd);
            else if (cw_state == 4) M4move(spd);
          }
          else if (ccw_state > 0) {
            float spd = 0.4f * M_MAX;
            if (ccw_state == 1) M1move(-spd);
            else if (ccw_state == 2) M2move(-spd);
            else if (ccw_state == 3) M3move(-spd);
            else if (ccw_state == 4) M4move(-spd);
          } else {
            Mstop(); 
          }
        } break;
      }
      break;
    case MODE_STOP:
      Mstop();
      break;
  }
}