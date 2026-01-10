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
  MODE_READY,     // 準備完了
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
RobotMode currentMode = MODE_READY; 
DebugMode currentDebug = DEBUG_BNO;

// フィールド復帰用の管理変数
float prev_field_angle = -999.0f; // 前回のフィールド方向（初期値は無効値）
bool is_field_out = false; // 半分以上外側に出ているか否か

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
#define M_MAX 250.0f
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
// 指定した角度と速度で全モーターを動かす関数(angle:-180 ~ 180, spd_rate:0 ~ 1)
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

// 姿勢制御込みの移動関数
void Mmove_with_spin(float angle, float spd_rate, float sisei_out) {
  // 基本の移動スピードを計算
  float m1 = sin(degtorad(angle - 45.0f))  * spd_rate;
  float m2 = sin(degtorad(angle - 135.0f)) * spd_rate;
  float m3 = sin(degtorad(angle - 225.0f)) * spd_rate;
  float m4 = sin(degtorad(angle - 315.0f)) * spd_rate;

  // 全モーターに姿勢制御の回転成分(sisei_out)を加算
  // sisei_outが正なら時計回り、負なら反時計回りの力が加わる
  M1move((m1 + sisei_out) * M_MAX);
  M2move((m2 + sisei_out) * M_MAX);
  M3move((m3 + sisei_out) * M_MAX);
  M4move((m4 + sisei_out) * M_MAX);
}

//姿勢(PID)制御関連
#define P_GAIN 2.0f
#define I_GAIN 2.0f
#define D_GAIN 0.3f
#define I_LIMIT 50.0f       // アンチワインドアップ（積分の最大値）
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

    // 差が180度付近（110〜250度）なら、ラインを跨いで外に出たと判定
    if (diff > 110.0f && diff < 250.0f) {
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
// 同様のラインマイコンへ送信する関数
void send_to_line_system_status() {
  uint8_t header = 0xAE;
  uint8_t high = (uint8_t)currentMode;   // RobotModeを送信
  uint8_t low  = (uint8_t)currentDebug;  // DebugModeを送信
  uint8_t checksum = (uint8_t)(header + high + low);

  lineSerial.write(header);
  lineSerial.write(high);
  lineSerial.write(low);
  lineSerial.write(checksum);
}

float IR_angle = NAN;//いったんこれだけ使う予定
int IR_distance = 0;

// ボールマイコンからボールデータを受信する関数
void receive_from_ball(){
  // 4バイト以上溜まっている間、パケットを探し続ける
  while (ballSerial.available() >= 4) {
    // 1. ヘッダー(0xA1)を探す
    if (ballSerial.read() != 0xA1) {
      continue; 
    }

    // 2. 残りのデータを読み込む
    uint8_t angle_byte  = ballSerial.read();
    uint8_t status_byte = ballSerial.read();
    uint8_t checksum    = ballSerial.read();

    // 3. チェックサム確認
    uint8_t header = 0xA1;
    uint8_t calc_sum = (uint8_t)(header + angle_byte + status_byte);

    if (calc_sum == checksum) {
      // 4. ステータスバイトの最上位ビット(Bit7)でボールの有無を確認
      bool ball_exists = (status_byte & 0x80) != 0;

      if (!ball_exists) {
        IR_angle = NAN;
        IR_distance = 0;
      } else {
        // 5. データの復元
        // 角度: 0~255 -> -180~180
        IR_angle = (float)angle_byte * (360.0f / 255.0f) - 180.0f;
        
        // 距離: 下位7bitを取り出して復元 0~127 -> 0~7000
        IR_distance = (int)(status_byte & 0x7F) * 55;
      }
    }
  }
}

// ボールの角度(-180~180)を入れると、大回りの回り込み角度(-180~180)を返す
float get_orbit_angle(float ir_deg) {
    float abs_ir = abs(ir_deg);
    float orbit_abs;

    // 回り込みの計算（大回りセッティング）
    if (abs_ir <= 15.0f) {
        // 15度以内：真正面(0度)に進む
        orbit_abs = 0.0f;
    } 
    else if (abs_ir <= 40.0f) {
        // 15度〜40度を 0度〜90度にマッピング
        // 式：(abs_ir - 15) * (90 - 0) / (40 - 15) + 0
        orbit_abs = (abs_ir - 15.0f) * 3.6f; 
    } 
    else if (abs_ir <= 90.0f) {
        // 40度〜90度を 90度〜170度にマッピング
        // 式：(abs_ir - 40) * (170 - 90) / (90 - 40) + 90
        orbit_abs = (abs_ir - 40.0f) * 1.6f + 90.0f;
    } 
    else {
        // 90度〜180度を 170度〜230度にマッピング
        // 式：(abs_ir - 90) * (230 - 170) / (180 - 90) + 170
        orbit_abs = (abs_ir - 90.0f) * 0.666f + 170.0f;
    }

    // 元の符号（左右）に戻す
    float result = (ir_deg >= 0) ? orbit_abs : -orbit_abs;
    
    // -180~180の範囲に収める（230度などはここで適切なマイナス角に変換される）
    return normalize_angle(result);
}

//ボタンピンの定義
const int button_pins[4]{
  PB0, PB1, PB2, PB4
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

        if (millis() - last_debounce[i] > 10) { // 10msチャタリング防止
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
  bool mode_changed = false;

  // --- 追加：NORMALモード中、どのボタンを押してもSTOPへ ---
  if (currentMode == MODE_NORMAL) {
    if (btn_pressed[0] || btn_pressed[1] || btn_pressed[2] || btn_pressed[3]) {
      currentMode = MODE_STOP;
      Mstop();
      is_field_out = false;
      prev_field_angle = -999.0f;
      mode_changed = true;
      
      // 以降のボタン処理をスキップ
      goto sync_and_exit; 
    }
  }

  // ボタン3: メインモード切替
  if (btn_pressed[3]) {
    // デバッグモード、かつモーターデバッグ中の時は反応させない
    if (currentMode == MODE_DEBUG && currentDebug == DEBUG_MOTOR) {
      // 無効
    } 
    else {
      if (currentMode == MODE_READY)      currentMode = MODE_DEBUG;
      else if (currentMode == MODE_DEBUG) currentMode = MODE_STOP;
      else if (currentMode == MODE_STOP)  currentMode = MODE_READY;
      // (MODE_NORMALからのSTOP移行は冒頭の処理でカバーされるため削除)
      
      is_field_out = false;
      prev_field_angle = -999.0f;
      Mstop();
      mode_changed = true;
    }
  }

  // ボタン2: 通常走行へ移行
  if (btn_pressed[2]) {
    if (currentMode == MODE_READY) {
      currentMode = MODE_NORMAL;
      is_field_out = false;
      prev_field_angle = -999.0f;
      mode_changed = true;
    }
  }
  
  // ボタン0: デバッグ項目の切り替え
  if (btn_pressed[0]) {
    if (currentMode == MODE_DEBUG) {
      currentDebug = (DebugMode)((currentDebug + 1) % 4);
      Mstop();
      mode_changed = true;
    }
  }

sync_and_exit: // 強制停止時などのジャンプ先

  if (mode_changed) {
    for(int i = 0; i < 3; i++) {
      send_to_line_system_status();
      send_to_ball_system_status();
      delay(2); 
    }
  }
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

  // サブマイコンへ現在のモードを強制同期（念のため3回送信）
  for(int i = 0; i < 3; i++) {
    send_to_line_system_status();
    send_to_ball_system_status();
    delay(5); // 確実に受信させるための短い待ち時間
  }
  
  last_micros = micros(); // PID用の時間管理をここから開始
}

void loop() {
    // センサー更新
    update_buttons();
    handle_mode_logic();
    receive_from_line(); // ライン情報の受信
    receive_from_ball(); // ボール情報の受信

    // 姿勢制御用の時間計測
    unsigned long current_micros = micros();
    float dt = (float)(current_micros - last_micros) / 1000000.0f;
    if (dt <= 0.0001f) { delay(1); return; }

    // 角度・PID制御計算
    current_yaw = BNO_get_yaw();
    float error = normalize_angle(target_angle - current_yaw);
    float p_out = P_GAIN * error;
    integral += error * dt;
    if (integral > I_LIMIT)  integral = I_LIMIT;
    else if (integral < -I_LIMIT) integral = -I_LIMIT;
    float i_out = I_GAIN * integral;
    float d_out = D_GAIN * (error - pre_error) / dt;
    float sisei_output = (p_out + i_out + d_out) / M_MAX;
    if (abs(error) > 0.5f) { // 0.5度以上のズレがあるなら
        float min_pwm = 0.05f; // モーターが動き出す最小の力（15%など。実機で調整）
        if (sisei_output > 0) sisei_output += min_pwm;
        else if (sisei_output < 0) sisei_output -= min_pwm;
    }
    if (sisei_output > OUTPUT_LIMIT)  sisei_output = OUTPUT_LIMIT;
    else if (sisei_output < -OUTPUT_LIMIT) sisei_output = -OUTPUT_LIMIT;
    
    switch (currentMode) {
      case MODE_READY:
        Mstop();
        break;
      case MODE_NORMAL: {
        /*
        float field_angle = calculate_field_angle(line_data);

        if (line_data > 0) {
          // 【最優先】ライン踏み中：計算されたフィールド中央へ戻る
          Mmove_with_spin(field_angle, 0.7f, sisei_output);
        } 
        else if (is_field_out) {
          // 【緊急】ラインは踏んでいないが、外に飛び出している判定の時
          // フィールド中央へ向かって復帰走行（少し遅めの0.5f）
          Mmove_with_spin(field_angle, 0.8f, sisei_output);
        }
        else if (isnan(IR_angle)) {
          // ボールがない時は停止
          Mspin(sisei_output);
        } 
        else {
          // 通常の回り込み追従
          float move_angle = get_orbit_angle(IR_angle);
          Mmove_with_spin(move_angle, 0.6f, sisei_output);
        }*/

        // フィールド角度の計算自体は内部状態（is_field_outなど）更新のために実行だけしておく
        float field_angle = calculate_field_angle(line_data);

        if (isnan(IR_angle)) {
          // ボールが見えない時はその場で姿勢を維持（回転のみ）
          Mspin(sisei_output);
        } 
        else {
          // ラインの状態に関わらず、常にボールへの回り込み角度を計算して移動
          float move_angle = get_orbit_angle(IR_angle);
          Mmove_with_spin(move_angle, 1.0f, sisei_output);
        }
      } break;
      case MODE_DEBUG:
        switch (currentDebug) {
          case DEBUG_BALL:
            break;
          case DEBUG_BNO:
            // BNOの送信頻度だけを30msに1回（約33Hz）に制限
            static unsigned long last_bno_send = 0;
            if (millis() - last_bno_send > 30) {
              send_to_ball_BNOdata(current_yaw);
              last_bno_send = millis();
            }
            break;
          case DEBUG_LINE:
            send_to_ball_linedata(line_data);
            break;
          case DEBUG_MOTOR:
            static int cw_state = 0;  // 0:停止, 1:M1, 2:M2, 3:M3, 4:M4 (時計回り用)
            static int ccw_state = 0; // 0:停止, 1:M1, 2:M2, 3:M3, 4:M4 (反時計回り用)
            static bool pid_enabled = false;

            // --- ボタン処理 ---
            if (btn_pressed[1]) {
              cw_state = (cw_state + 1) % 5;
              ccw_state = 0; pid_enabled = false; Mstop(); // モード変更時に一旦止める
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
              // 姿勢制御（不感帯補正が必要ならここに追加）
              Mspin(sisei_output); 
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
              Mstop(); // 何も選択されていない時は停止
            }

            break;
        }
        if(currentDebug != DEBUG_MOTOR) Mstop();
        break;
      case MODE_STOP:{
        Mstop();
      } break;
    }

    pre_error = error;
    last_micros = current_micros;
    delay(1); 

    static unsigned long last_backup_sync = 0;
    if (millis() - last_backup_sync > 2000) { // 2秒に1回だけ「念のため」同期
      send_to_line_system_status();
      send_to_ball_system_status();
      last_backup_sync = millis();
    }
}