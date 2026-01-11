#include <Arduino.h>
#include <math.h>

// --- 定数・列挙型の定義 ---
enum RobotMode { MODE_READY, MODE_NORMAL, MODE_DEBUG, MODE_STOP };
enum DebugMode { DEBUG_BALL, DEBUG_LINE, DEBUG_BNO, DEBUG_MOTOR };

RobotMode currentMode = MODE_READY; 
DebugMode currentDebug = DEBUG_BNO;

// モードスロット管理
int normalModeIndex = 0; // 0~11

// キャリブレーション管理
bool is_line_calibrating = false;
float bno_offset = 0.0f; // BNOの0点オフセット用

// フィールド復帰用
float prev_field_angle = -999.0f;
bool is_field_out = false;

// ライン回避の補完用
float last_valid_line_dir = NAN; 
unsigned long last_line_time = 0; // ラインを最後に検知した時刻

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

// ボールマイコンへ現在のスロット番号(LED点滅用)を送信
void send_led_mode() {
  send_packet(ballSerial, 0xAD, (uint8_t)normalModeIndex, 0);
}

// --- モーター制御 ---
#define M_MAX 250.0f
const int motor_pins[8] = { PA8, PC0, PA0, PC1, PA1, PC2, PA6, PC3 };

// モーターの正転方向がm1とm2だけ機体間でちがう
// 機体黒　
void M1move(float speed) {
  speed = round(constrain(speed, -M_MAX, M_MAX));
  if (speed > 0) { digitalWrite(motor_pins[1], HIGH); analogWrite(motor_pins[0], (int)speed); }
  else if (speed < 0) { digitalWrite(motor_pins[1], LOW); analogWrite(motor_pins[0], (int)-speed); }
  else analogWrite(motor_pins[0], 0);
}
void M2move(float speed) {
  speed = round(constrain(speed, -M_MAX, M_MAX));
  if (speed > 0) { digitalWrite(motor_pins[3], LOW); analogWrite(motor_pins[2], (int)speed); }
  else if (speed < 0) { digitalWrite(motor_pins[3], HIGH); analogWrite(motor_pins[2], (int)-speed); }
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


//機体白
/*
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
*/

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

void Mmove(float angle, float spd_rate){
  // 移動方向（angle）をラジアンに変換
  // モーターの配置に合わせて -45度 オフセット
  float rad = (angle - 45.0f) * PI / 180.0f;

  // 各車輪の速度計算
  // 45度、135度、225度、315度に配置されたオムニホイールの基本式
  float m1 = sin(rad) * spd_rate;
  float m2 = sin(rad - PI/2.0f) * spd_rate;
  float m3 = sin(rad - PI) * spd_rate;
  float m4 = sin(rad - 3.0f*PI/2.0f) * spd_rate;

  // モーター出力（回転成分を加えない純粋な移動）
  M1move(m1 * M_MAX);
  M2move(m2 * M_MAX);
  M3move(m3 * M_MAX);
  M4move(m4 * M_MAX);
}

void Mspin(float spn_rate) {
  M1move(spn_rate * M_MAX); M2move(spn_rate * M_MAX);
  M3move(spn_rate * M_MAX); M4move(spn_rate * M_MAX);
}

// --- ボタン管理 ---
const int button_pins[4] = { PB0, PB1, PB2, PB4 };
bool btn_pressed[4] = {false};
void update_buttons() {
  static bool last_raw_state[4] = {false};     // 前回の生のピン状態
  static bool stable_state[4] = {false};       // 確定したボタンの状態
  static unsigned long last_debounce_time[4] = {0};
  const unsigned long debounce_delay = 20;     // 20ms安定していたら確定

  for (int i = 0; i < 4; i++) {
    bool current_raw = digitalRead(button_pins[i]);
    btn_pressed[i] = false; // 毎ループ初期化

    // ピンの状態が変化した（ノイズか押し始め）
    if (current_raw != last_raw_state[i]) {
      last_debounce_time[i] = millis();
    }

    // 最後に状態が変わってから debounce_delay 以上経過している場合
    if ((millis() - last_debounce_time[i]) > debounce_delay) {
      // 確定した状態が変わった場合
      if (current_raw != stable_state[i]) {
        stable_state[i] = current_raw;
        // 離れた状態から「押された状態」になった瞬間だけ true にする
        if (stable_state[i] == HIGH) {
          btn_pressed[i] = true;
        }
      }
    }
    last_raw_state[i] = current_raw;
  }
}

// --- 回り込み計算 (距離を考慮) ---
float get_orbit_angle(float ir_deg, int ir_dist) {// 角度と距離が引数
  //ir_dist が　定義：500以上でボールあり
  //ir_dist が　4000以上か以下かで回り込みの関数を変えたい
  float abs_ir = abs(ir_deg);
  float orbit_abs;

  // --- 1. 近距離（ir_dist >= 4000）の回り込みロジック ---
  // ボールが近いため、より早く・大きく回り込んでシュート体制を作る
  if (ir_dist >= 4000) {
    if (abs_ir <= 10.0f) orbit_abs = 0.0f; // 正面付近は直進
    else if (abs_ir <= 30.0f) orbit_abs = (abs_ir - 10.0f) * 4.5f; // 急激に回り込む
    else if (abs_ir <= 80.0f) orbit_abs = (abs_ir - 30.0f) * 1.0f + 90.0f; 
    else orbit_abs = (abs_ir - 80.0f) * 0.8f + 140.0f; // 真後ろ付近
  }
  // --- 2. 遠距離（ir_dist < 4000）の回り込みロジック (既存コードを維持) ---
  else {
    // --- 遠距離：近づくこと優先（より緩やかに変更） ---
    if (abs_ir <= 30.0f) {
      // 30度までは直進に近い形でボールへ向かう
      orbit_abs = abs_ir * 0.5f; 
    }
    else if (abs_ir <= 90.0f) {
      // 横方向でも「斜め前」程度に抑えて距離を詰める
      orbit_abs = (abs_ir - 30.0f) * 1.2f + 15.0f; 
    }
    else {
      // 真後ろにボールがあっても、大きく回り込まずにまずは引き付ける
      orbit_abs = (abs_ir - 90.0f) * 0.5f + 87.0f;
    }
  }

  return normalize_angle((ir_deg >= 0) ? orbit_abs : -orbit_abs);
}

// --- 回り込み計算 ---
float mawarikomi(float ir_deg) {// 角度が引数
  float abs_ir = abs(ir_deg);
  float orbit_abs;
  if (abs_ir <= 10.0f) orbit_abs = 0.0f; // 正面付近は直進
  else if (abs_ir <= 30.0f) orbit_abs = (abs_ir - 10.0f) * 4.5f; // 急激に回り込む
  else if (abs_ir <= 80.0f) orbit_abs = (abs_ir - 30.0f) * 1.0f + 90.0f; 
  else orbit_abs = (abs_ir - 80.0f) * 0.8f + 140.0f; // 真後ろ付近
  return normalize_angle((ir_deg >= 0) ? orbit_abs : -orbit_abs);
}

// --- メインループ変数 ---
float IR_angle = NAN;
float IR_distance = 0;
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

  update_buttons(); // 全ボタンの状態を一括更新

  // --- 1. NORMALモード中の処理 (最優先) ---
  if (currentMode == MODE_NORMAL) {
    // どのボタンを押しても即座に停止モードへ
    if (btn_pressed[0] || btn_pressed[1] || btn_pressed[2] || btn_pressed[3]) {
      nextMode = MODE_STOP;
      mode_changed = true;
    }
  } 
  
  // --- 2. READYモード中の処理 ---
  else if (currentMode == MODE_READY) {
    if (btn_pressed[0]) { // ボタン0: スロット切り替え
      normalModeIndex = (normalModeIndex + 1) % 12;
      send_led_mode(); // ボールマイコンへ 0xAD 送信
    }
    else if (btn_pressed[2]) { // ボタン2: 走行開始
      nextMode = MODE_NORMAL;
      mode_changed = true;
    }
    else if (btn_pressed[3]) { // ボタン3: デバッグモードへ
      nextMode = MODE_DEBUG;
      mode_changed = true;
    }
  }

  // --- 3. DEBUGモード中の処理 ---
  else if (currentMode == MODE_DEBUG) {
    // ボタン0: デバッグ項目の切り替え
    if (btn_pressed[0]) {
      currentDebug = (DebugMode)((currentDebug + 1) % 4);
      mode_changed = true;
    }
    // ボタン1: キャリブレーション実行
    else if (btn_pressed[1]) {
      if (currentDebug == DEBUG_LINE) {
        is_line_calibrating = !is_line_calibrating;
        send_calib_signal(is_line_calibrating);
      } else if (currentDebug == DEBUG_BNO) {
        send_calib_signal(true); delay(200);
        bno_offset = BNO_get_raw_yaw();
        reset_PID();
        send_calib_signal(false);
      }
    }
    // ボタン3: ストップへ戻る
    else if (btn_pressed[3]) {
      if (currentDebug == DEBUG_MOTOR) {
        // モーターデバッグ中なら、ここでは何もしない（loop内のDEBUG_MOTOR処理に任せる）
      } else {
        nextMode = MODE_STOP;
        mode_changed = true;
      }
    }
    // モーターデバッグ時の特殊操作は別途ここで行う（今回は省略）
  }

  // --- 4. STOPモード中の処理 ---
  else if (currentMode == MODE_STOP) {
    if (btn_pressed[3]) { // ボタン3: READYへ復帰
      nextMode = MODE_READY;
      mode_changed = true;
    }
  }

  // --- 5. モード変更の確定と事後処理 ---
  if (mode_changed) {
    reset_PID();
    currentMode = nextMode;
    Mstop();
    send_system_status(); // 0xAE 送信
    send_led_mode();      // 0xAD 送信（ボール側LEDを最新にする）
  }
}

/*
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
*/


// 12個のラインセンサーの反応から、ラインがある方向（重心）を計算する
// 戻り値: ラインがある方向(deg)。反応がない場合は NAN
float calculate_line_vector_angle(uint16_t mask) {
  mask &= 0x0FFF;
  if (mask == 0) return NAN;

  float vx = 0.0f;
  float vy = 0.0f;

  for (int i = 0; i < 12; i++) {
    if (mask & (1 << i)) {
      float angle_rad = (i * 30.0f) * PI / 180.0f;
      vx += cos(angle_rad);
      vy += sin(angle_rad);
    }
  }

  float final_mag = sqrt(vx * vx + vy * vy);

  // ベクトルがしっかり出ている（相殺されていない）場合
  if (final_mag > 0.4f) {
    float line_dir = atan2(vy, vx) * 180.0f / PI;
    last_valid_line_dir = normalize_angle(line_dir); // 有効な値を保存
    last_line_time = millis(); // 検知時刻を更新
    return last_valid_line_dir;
  } 
  
  // 相殺(3番+9番など)が起きたが、前回のデータが新しい(500ms以内)ならそれを使う
  if (!isnan(last_valid_line_dir) && (millis() - last_line_time < 500)) {
    return last_valid_line_dir;
  }

  return NAN;

  // 合成ベクトルの角度を計算（これが「ラインの塊がある方向」）
  float line_dir = atan2(vy, vx) * 180.0f / PI;
  return normalize_angle(line_dir);
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
  // バッファに4バイト以上ある間ループ
  while (ballSerial.available() >= 4) {
    // ヘッダーが 0xA1 か確認
    if (ballSerial.read() != 0xA1) continue;

    uint8_t ab = ballSerial.read(); // 角度バイト (angle_byte)
    uint8_t sb = ballSerial.read(); // ステータス/距離バイト (status_byte)
    uint8_t c  = ballSerial.read(); // チェックサム (checksum)

    // チェックサムの検証
    if ((uint8_t)(0xA1 + ab + sb) == c) {
      // ボールありフラグ（最上位ビット）の確認
      if (!(sb & 0x80)) { 
        IR_angle = NAN; 
        IR_distance = 0; // ボールがない場合は距離もリセット
      } 
      else {
        // 1. 角度の復元 (-180度 〜 180度)
        IR_angle = (float)ab * (360.0f / 255.0f) - 180.0f;

        // 2. 距離の復元
        // 下位7ビットを取り出し、送信時の逆算（* 55）を行う
        uint8_t dist_7bit = sb & 0x7F; 
        IR_distance = (int)dist_7bit * 55;
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

  // モード切替
  switch (currentMode) {
    case MODE_READY:
      Mstop();
      break;
    case MODE_NORMAL: {
      switch (normalModeIndex){
        
        /* --アタッカープログラム-- */
        case 0: {

    float vx = 0.0f;
    float vy = 0.0f;

    // 1. ボールへ向かうベクトル
    if (!isnan(IR_angle)) {
        float ball_rad = mawarikomi(IR_angle) * PI / 180.0f;
        vx += cos(ball_rad);
        vy += sin(ball_rad);
    }

    // 2. ラインから「離れる」ベクトル
    float line_dir = calculate_line_vector_angle(line_data);
    if (!isnan(line_dir)) {
        // ラインがある方向 (line_dir) の真逆 (+180度) が逃げる方向
        float escape_rad = (line_dir + 180.0f) * PI / 180.0f;
        
        // 回避の強さを設定。
        // ラインに近い（反応センサー数が多い）ほど強く反発させるなら、
        // 重みを固定にせず、ベクトル長を利用するのもアリです。
        float avoid_weight = 2.0f;
        vx += cos(escape_rad) * avoid_weight;
        vy += sin(escape_rad) * avoid_weight;
    }

    // 3. 移動実行
    float final_spd = sqrt(vx * vx + vy * vy);
    
    float max_val = 1.0f;
    if (final_spd > max_val) {
    vx = (vx / final_spd) * max_val;
    vy = (vy / final_spd) * max_val;
    final_spd = max_val;

    }
    if (final_spd > 0.1f) {
        float final_angle = atan2(vy, vx) * 180.0f / PI;
        float move_spd = (final_spd > 1.0f) ? 1.0f : final_spd;
        Mmove_with_spin(final_angle, move_spd, sisei_output);
    } else {
        Mspin(sisei_output);
    }
      
        } break;

        /* --キーパープログラム-- */
        case 1: {

    float vx = 0.0f;
    float vy = 0.0f;
    float keeper_spd = 0.8f;

    // 1. ボールの左右位置に応じた横移動 (vx)
    if (!isnan(IR_angle)) {
        if (IR_angle > 10.0f) vx = cos(90.0f * PI / 180.0f);
        else if (IR_angle < -10.0f) vx = cos(-90.0f * PI / 180.0f);
    }

    // 2. ラインベクトルを使った位置保持 (vy)
    float line_dir = calculate_line_vector_angle(line_data);

    if (!isnan(line_dir)) {
        // 検知しているラインの方向へ吸着する
        float line_rad = line_dir * PI / 180.0f;
        vx += cos(line_rad) * 1.0f; 
        vy += sin(line_rad) * 1.0f;
    } 
    else if (!isnan(last_valid_line_dir) && (millis() - last_line_time < 3000)) {
        // 最後に反応があったラインの方向へ向かって復帰を試みる
        float last_line_rad = last_valid_line_dir * PI / 180.0f;
        vx += cos(last_line_rad) * 1.0f;
        vy += sin(last_line_rad) * 1.0f;
    }
    else {
        // 完全に迷子なので、ラインに触れるまでゆっくり後退する
        vy = -0.8f; 
        vx = 0.0f; // 横移動を止めて後退を優先
    }

    // 3. 移動実行
    float final_spd = sqrt(vx * vx + vy * vy);
    if (final_spd > 0.1f) {
        float final_angle = atan2(vy, vx) * 180.0f / PI;
        Mmove_with_spin(final_angle, keeper_spd, sisei_output);
    } else {
        Mspin(sisei_output);
    }

        } break;
        
        /* --ラインセンサーなしの回り込み-- */
        case 2: {

    float vx = 0.0f;
    float vy = 0.0f;

    // 1. ボールへ向かうベクトル
    if (!isnan(IR_angle)) {
        float ball_rad = get_orbit_angle(IR_angle, IR_distance) * PI / 180.0f;
        vx += cos(ball_rad);
        vy += sin(ball_rad);
    }

    // 3. 移動実行
    float final_spd = sqrt(vx * vx + vy * vy);
    float max_val = 1.0f;
    if (final_spd > max_val) {
    vx = (vx / final_spd) * max_val;
    vy = (vy / final_spd) * max_val;
    final_spd = max_val;
    }
    if (final_spd > 0.1f) {
        float final_angle = atan2(vy, vx) * 180.0f / PI;
        float move_spd = (final_spd > 1.0f) ? 1.0f : final_spd;
        Mmove_with_spin(final_angle, move_spd, sisei_output);
    } else {
        Mspin(sisei_output);
    }

        } break;

        default:
          Mstop();
          break;
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
  delay(1);
}