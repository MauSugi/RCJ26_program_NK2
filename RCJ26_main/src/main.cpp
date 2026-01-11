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

// --- モーター安全制御用 ---
struct MotorSafety {
  int8_t last_dir;              // 1: 正転, -1: 逆転, 0: 停止
  unsigned long dead_time_end;  // デッドタイム終了時刻(ms)
};

MotorSafety m_safety[4] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
const unsigned long DEAD_TIME_MS = 3; // ★1msに設定

// 共通の反転防止ロジック
float check_motor_safety(int id, float speed) {
  int8_t next_dir = (speed > 5.0f) ? 1 : (speed < -5.0f) ? -1 : 0;
  unsigned long now = millis();

  // 方向が「正」から「負」、あるいは「負」から「正」へ直接切り替わった場合
  if (next_dir != 0 && m_safety[id].last_dir != 0 && next_dir != m_safety[id].last_dir) {
    m_safety[id].dead_time_end = now + DEAD_TIME_MS;
  }
  
  m_safety[id].last_dir = next_dir;

  // デッドタイム中、または極端に小さい値は0を返す
  if (now < m_safety[id].dead_time_end) return 0.0f;
  return speed;
}

// モーターの正転方向がm1とm2だけ機体間でちがう
// 機体黒　
void M1move(float speed) {
  speed = check_motor_safety(0, speed); // 安全装置
  speed = round(constrain(speed, -M_MAX, M_MAX));
  if (speed > 0) { digitalWrite(motor_pins[1], HIGH); analogWrite(motor_pins[0], (int)speed); }
  else if (speed < 0) { digitalWrite(motor_pins[1], LOW); analogWrite(motor_pins[0], (int)-speed); }
  else analogWrite(motor_pins[0], 0);
}
void M2move(float speed) {
  speed = check_motor_safety(1, speed); // 安全装置
  speed = round(constrain(speed, -M_MAX, M_MAX));
  if (speed > 0) { digitalWrite(motor_pins[3], LOW); analogWrite(motor_pins[2], (int)speed); }
  else if (speed < 0) { digitalWrite(motor_pins[3], HIGH); analogWrite(motor_pins[2], (int)-speed); }
  else analogWrite(motor_pins[2], 0);
}
void M3move(float speed) {
  speed = check_motor_safety(2, speed); // 安全装置
  speed = round(constrain(speed, -M_MAX, M_MAX));
  if (speed > 0) { digitalWrite(motor_pins[5], LOW); analogWrite(motor_pins[4], (int)speed); }
  else if (speed < 0) { digitalWrite(motor_pins[5], HIGH); analogWrite(motor_pins[4], (int)-speed); }
  else analogWrite(motor_pins[4], 0);
}
void M4move(float speed) {
  speed = check_motor_safety(3, speed); // 安全装置
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
  // 1. 各モーターの理想出力を計算
  float m1 = sin((angle-45.0f) * PI / 180.0f) * spd_rate + sisei_out;
  float m2 = sin((angle-135.0f) * PI / 180.0f) * spd_rate + sisei_out;
  float m3 = sin((angle-225.0f) * PI / 180.0f) * spd_rate + sisei_out;
  float m4 = sin((angle-315.0f) * PI / 180.0f) * spd_rate + sisei_out;

  // 2. 4つのモーターの中で最大絶対値を求める
  float max_val = fabs(m1);
  if (fabs(m2) > max_val) max_val = fabs(m2);
  if (fabs(m3) > max_val) max_val = fabs(m3);
  if (fabs(m4) > max_val) max_val = fabs(m4);

  // 3. もし最大値が 1.0 を超えていたら、全体を max_val で割ってスケーリングする
  if (max_val > 1.0f) {
    m1 /= max_val;
    m2 /= max_val;
    m3 /= max_val;
    m4 /= max_val;
  }

  // 4. 最終的な出力
  M1move(m1 * M_MAX);
  M2move(m2 * M_MAX);
  M3move(m3 * M_MAX);
  M4move(m4 * M_MAX);
}

/*
void Mmove(float angle, float spd_rate){
  // angle: 前0, 右90, 左-90（atan2(vx, vy)から直接来る角度）
  float rad = angle * PI / 180.0f;

  // 右前から時計回りにM1, M2, M3, M4配置
  // 前(0度)を入力した時に前進する、最もシンプルな符号の組み合わせです
  float m1 =  sin(rad + PI/4.0f) * spd_rate; // 右前 (45度)
  float m2 = -cos(rad + PI/4.0f) * spd_rate; // 右後 (135度)
  float m3 = -sin(rad + PI/4.0f) * spd_rate; // 左後 (-135度)
  float m4 =  cos(rad + PI/4.0f) * spd_rate; // 左前 (-45度)

  // モーター出力（回転成分を加えない純粋な移動）
  M1move(m1 * M_MAX);
  M2move(m2 * M_MAX);
  M3move(m3 * M_MAX);
  M4move(m4 * M_MAX);
}
*/

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

float mawarikomi(float ir_deg) {
  float abs_ir = fabs(ir_deg);
  float orbit_abs;

  // 10度までは「0(直進)」ではなく、あえて少しだけ角度をつける(1.0倍)
  // これにより、ボールが正面付近で左右に振れた時の過剰な反応を抑えます
  if (abs_ir <= 15.0f) {
    orbit_abs = abs_ir * 1.5f; 
  } 
  else if (abs_ir <= 90.0f) {
    // 15度(22.5) -> 90度(135) : 係数1.5
    // 滑らかに接続しつつ、横に逃げるボールをしっかり追う
    orbit_abs = (abs_ir - 15.0f) * 1.5f + 22.5f;
  } 
  else {
    // 90度(135) -> 180度(180) : 係数0.5
    // 後ろにあるボールに対しては、回り込みすぎず「引き付ける」
    orbit_abs = (abs_ir - 90.0f) * 0.5f + 135.0f;
  }

  // 元の符号（左右）を戻す
  float result = (ir_deg >= 0) ? orbit_abs : -orbit_abs;
  return normalize_angle(result);
}

// フィルター用のグローバル変数
float filted_IR_angle = 0;

// --- メインループ変数 ---
float IR_angle = NAN;
float IR_distance = 0;
uint16_t line_data = 0;
float current_yaw = 0.0f;
float pre_error = 0.0f, integral = 0.0f;
float sisei_output = 0.0f; // ★追加：姿勢制御の出力値を保存する変数
unsigned long last_micros = 0;
static const float K_P = 1.5f;     // 比例ゲイン
static const float K_I = 2.0f;     // 積分ゲイン
static const float K_D = 0.01f;     // 微分ゲイン
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

// 12個のラインセンサーの反応から、ラインがある方向（重心）を計算する
// 戻り値: ラインがある方向(deg)。反応がない、または反応が1つ以下の場合は NAN
float calculate_line_vector_angle(uint16_t mask) {
  mask &= 0x0FFF;

  // --- 1. センサー反応数をカウント ---
  int active_count = __builtin_popcount(mask);

  // --- 2. 1個以下の時は「無効」として即座にNANを返す ---
  // 保持データ(last_valid_line_dir)の更新もしないため、
  // 呼び出し側(Case 1など)の保持タイマー判定(100ms)に委ねられる
  if (active_count < 2) {
    return NAN;
  }

  float vx = 0.0f;
  float vy = 0.0f;

  for (int i = 0; i < 12; i++) {
    if (mask & (1 << i)) {
      float angle_rad = (i * 30.0f) * PI / 180.0f;
      vx += sin(angle_rad);
      vy += cos(angle_rad);
    }
  }

  float final_mag = sqrt(vx * vx + vy * vy);

  // --- 3. 2個以上反応し、かつベクトルが相殺されていない場合のみ有効 ---
  if (final_mag > 0.2f) {
    float line_dir = atan2(vx, vy) * 180.0f / PI; 
    last_valid_line_dir = normalize_angle(line_dir);
    last_line_time = millis(); // 「有効な方向」が得られた最終時刻を更新
    return last_valid_line_dir;
  } 
  
  return NAN;
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

  if (!isnan(IR_angle)) {
    if (isnan(filted_IR_angle)) {
      filted_IR_angle = IR_angle;
    } else {//またぎの処理
      float diff = normalize_angle(IR_angle - filted_IR_angle);
      filted_IR_angle = normalize_angle(filted_IR_angle + diff * 0.5f);
    }
  } else {
    // ボールが見えない時はNANにする
    filted_IR_angle = NAN;
  }
  
  unsigned long cur = micros();
  float dt = (float)(cur - last_micros) / 1000000.0f;
  if (dt < 0.0005f) return;
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

            static unsigned long last_ball_time = 0; // ボール消失タイマー用
            float vx = 0.0f;
            float vy = 0.0f;

            // 1. ボールへ向かうベクトル
            if (!isnan(filted_IR_angle)) {
                last_ball_time = millis(); // ボールが見えている間は常に時刻を更新
                // 回り込み計算
                float ball_rad = mawarikomi(filted_IR_angle) * PI / 180.0f;
                vx += sin(ball_rad) * 1.0f;
                vy += cos(ball_rad) * 1.0f;
            } 
            else {
                // ボールが1000ms（1秒）以上見えない場合は自陣方向へ後退
                if (millis() - last_ball_time > 1000) {
                    vx += 0.0f;
                    vy += -0.5f; // 後退ベクトル
                }
            }

            // 2. ライン回避ベクトル (重心計算)
            float line_dir = calculate_line_vector_angle(line_data);
            
            // ラインを検知している、または検知から100ms以内（チャタリング・単発反応対策）
            if (!isnan(line_dir)) {
                // ラインがある方向の真逆へ逃げる
                float escape_rad = (line_dir + 180.0f) * PI / 180.0f;
                vx += sin(escape_rad) * 2.0f;
                vy += cos(escape_rad) * 2.0f;
            } 
            else if (!isnan(last_valid_line_dir) && (millis() - last_line_time < 100)) {
                // 消失直後100msは、最後に検知したライン方向から逃げ続ける
                float escape_rad = (last_valid_line_dir + 180.0f) * PI / 180.0f;
                vx += sin(escape_rad) * 2.0f;
                vy += cos(escape_rad) * 2.0f;
            }

            // 3. 合成ベクトルの正規化と移動実行
            float final_spd = sqrt(vx * vx + vy * vy);
            float max_val = 1.0f;
            
            // ベクトルの大きさが最大値を超えたらスケーリング
            if (final_spd > max_val) {
                vx = (vx / final_spd) * max_val;
                vy = (vy / final_spd) * max_val;
                final_spd = max_val;
            }

            // 最終的な移動
            if (final_spd > 0.1f) {
                float final_angle = atan2(vx, vy) * 180.0f / PI;
                // 姿勢制御(sisei_output)を掛け合わせてモーター出力
                Mmove_with_spin(final_angle, final_spd, sisei_output);
            } else {
                // 移動ベクトルが小さいときはその場で姿勢維持のみ
                Mspin(sisei_output);
            }
      
        } break;

        /* --キーパープログラム-- */
        case 1: {
           float vx = 0.0f;
            float vy = 0.0f;

            // 1. ライン方向の取得
            float line_dir = calculate_line_vector_angle(line_data);
            
            // 2. ライン検知時の処理
            if (!isnan(line_dir)) {
                // --- A. ライン復帰ベクトル ---
                float line_weight = 0.5f;
                vx += sin(line_dir * PI / 180.0f) * line_weight;
                vy += cos(line_dir * PI / 180.0f) * line_weight;

                // --- B. ライン並行移動ベクトル ---
                if (!isnan(filted_IR_angle)) {
                    // ライン方向を基準にしたボールの相対角度
                    float relative_ball_angle = normalize_angle(filted_IR_angle - line_dir);
                    
                    // 【追加】不感帯：ボールが正面(ライン垂直線上)付近なら横移動しない
                    float side_weight = 0.0f;
                    if (fabs(relative_ball_angle) > 15.0f) { // 15度以上のズレで反応
                        side_weight = 1.0f;
                    }

                    float side_dir = (relative_ball_angle > 0) ? (line_dir + 90.0f) : (line_dir - 90.0f);
                    
                    vx += sin(side_dir * PI / 180.0f) * side_weight;
                    vy += cos(side_dir * PI / 180.0f) * side_weight;
                }
            } 
            // 3. ライン消失時：直近100ms以内なら、その場に留まる or 前回の慣性で動く
            else if (!isnan(last_valid_line_dir) && (millis() - last_line_time < 100)) {
                // ライン復帰ベクトルのみ、最後に見たラインの方向へ弱く出し続ける
                float hold_weight = 0.4f;
                vx = sin(last_valid_line_dir * PI / 180.0f) * hold_weight;
                vy = cos(last_valid_line_dir * PI / 180.0f) * hold_weight;
            }
            // 4. 完全にラインを見失った時
            else {
                vx = 0.0f;
                vy = -0.5f; // ゆっくり後退してラインを探す
            }

            // --- 移動実行 ---
            float final_spd = sqrt(vx * vx + vy * vy);
            if (final_spd > 0.05f) {
                float final_angle = atan2(vx, vy) * 180.0f / PI;
                
                // 合成ベクトルの大きさに応じて速度を制限
                // final_spdが小さいときは低速、大きいときは最大0.8f
                float move_spd = constrain(final_spd, 0.0f, 1.0f);
                Mmove_with_spin(final_angle, move_spd, sisei_output);
            } else {
                Mspin(sisei_output);
            }

        } break;
        
        /* --ラインセンサーなしの回り込み-- */
        case 2: {

    float vx = 0.0f;
    float vy = 0.0f;

    // 1. ボールへ向かうベクトル
    if (!isnan(filted_IR_angle)) {
        float ball_rad = mawarikomi(filted_IR_angle) * PI / 180.0f;
        vx += sin(ball_rad);
        vy += cos(ball_rad);
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
        float final_angle = atan2(vx, vy) * 180.0f / PI;
        float move_spd = (final_spd > 1.0f) ? 1.0f : final_spd;
        Mmove_with_spin(final_angle, move_spd, sisei_output);
    } else {
        Mspin(sisei_output);
    }

        } break;

        // 平行移動のテスト
        case 3: {
          static unsigned long last_move_time = 0;
          static int move_step = 0;
          unsigned long now = millis();

          // 1000ms (1秒) ごとにステップを進める
          if (now - last_move_time > 1000) {
            move_step = (move_step + 1) % 4; // 0, 1, 2, 3 を繰り返す
            last_move_time = now;
          }

          float test_angle = 0;
          if (move_step == 0) test_angle = 0.0f;    // 前
          if (move_step == 1) test_angle = 90.0f;   // 右
          if (move_step == 2) test_angle = 180.0f;  // 後
          if (move_step == 3) test_angle = -90.0f;  // 左

          // 姿勢制御を効かせながら平行移動
          Mmove_with_spin(test_angle, 1.0f, sisei_output);
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