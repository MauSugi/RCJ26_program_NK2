#include <Arduino.h>
#include <math.h>
float radtodeg(float rad) {
  return rad * 180.0f / PI;
}
float degtorad(float deg) {
  return deg * PI / 180.0f;
}

/*
// BNO関連
#include <Wire.h>
byte ADDRESS = 0x28; 
byte EULER_REGISTER = 0x1A;
int merge(byte low, byte high){
    int result = low | (high << 8);
    if (result > 32767) { 
        result -= 65536;
    }
    return result; 
}
void writeToBNO(byte reg, byte val, int dly){
    Wire.beginTransmission(ADDRESS);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission(false);
    delay(dly); 
}
void initBNO(){
    Wire.beginTransmission(ADDRESS); 
    Wire.write(0x00);
    Wire.endTransmission(false); 
    Wire.requestFrom(ADDRESS, 1);
    if (Wire.read() == 0xa0) {
        Serial.println("BNO055 found.");
        writeToBNO(0x3d, 0x00, 80);// operating mode = config mode     
        writeToBNO(0x3f, 0x20, 1000);// sys_trigger = rst_sys     
        writeToBNO(0x3e, 0x00, 80);// pwr_mode = normal mode     
        writeToBNO(0x3f, 0x80, 1000);// sys trigger = clk_sel ex_osc     
        writeToBNO(0x3d, 0x0c, 80);// operating mode = ndof   
        Serial.println("BNO055 initialized.");
    } else {
        while (1) {
            Serial.println("BNO055 not found..");
            delay(5000);
        }
    } 
}
float get_compass_angle(){
    int euler[6];

    Wire.beginTransmission(ADDRESS);
    Wire.write(EULER_REGISTER);
    Wire.endTransmission(false);

    Wire.requestFrom(ADDRESS, 6);
    byte buffer[6];
    Wire.readBytes(buffer, 6);

    euler[0] = merge(buffer[0], buffer[1]);
    euler[1] = merge(buffer[2], buffer[3]);
    euler[2] = merge(buffer[4], buffer[5]);
    
    float yaw = float(euler[0]) / 16.0;
    if (yaw > 180.0f) {
      yaw -= 360.0f; // 181度なら-179度、360度なら0度になる
    }
    return yaw;
}
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#define BNO055_ADDR 0x28

// I2Cインスタンスの作成 (PB7: SDA, PB6: SCL)
TwoWire I2C_BNO(PB7, PB6);

// 低レベル書き込み関数
void write8(uint8_t reg, uint8_t val) {
  I2C_BNO.beginTransmission(BNO055_ADDR);
  I2C_BNO.write(reg);
  I2C_BNO.write(val);
  I2C_BNO.endTransmission();
}

// 低レベル読み込み関数
uint8_t read8(uint8_t reg) {
  I2C_BNO.beginTransmission(BNO055_ADDR);
  I2C_BNO.write(reg);
  I2C_BNO.endTransmission(false); // falseにして接続を維持
  I2C_BNO.requestFrom((uint8_t)BNO055_ADDR, (uint8_t)1);
  return I2C_BNO.available() ? I2C_BNO.read() : 0;
}

// 角度取得関数（yawを-180〜180で返す）
float get_yaw() {
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

/*
#define I2C_SDA PB9
#define I2C_SCL PB8
// 定数はまとめて定義
const uint8_t BNO055_ADDR = 0x28;
const uint8_t BNO055_CHIP_ID = 0xA0;
// レジスタアドレス
enum BNO_REG {
  REG_CHIP_ID = 0x00,
  REG_OPR_MODE = 0x3D,
  REG_PWR_MODE = 0x3E,
  REG_SYS_TRIGGER = 0x3F,
  REG_EULER_H_LSB = 0x1A
};
// 符号付き16bit整数への合成を最適化
int16_t merge(byte low, byte high) {
  return (int16_t)(low | (high << 8));
}
// 通信処理の共通化
void writeToBNO(uint8_t reg, uint8_t val, int dly) {
  Wire.beginTransmission(BNO055_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
  if (dly > 0) delay(dly);
}
void initBNO() {
  // 1. 接続確認
  Wire.beginTransmission(BNO055_ADDR);
  Wire.write(REG_CHIP_ID);
  Wire.endTransmission(false);
  Wire.requestFrom(BNO055_ADDR, (uint8_t)1);

  if (Wire.read() != BNO055_CHIP_ID) {
    while (1) {
      Serial.println("BNO055 not found. Check wiring (SDA:PB9, SCL:PB8)");
      delay(2000);
    }
  }
  Serial.println("BNO055 connected.");

  // 2. 初期化シーケンス（config mode -> reset -> normal power -> NDOF）
  writeToBNO(REG_OPR_MODE, 0x00, 20);   // CONFIG_MODE
  writeToBNO(REG_SYS_TRIGGER, 0x20, 700); // Reset
  writeToBNO(REG_PWR_MODE, 0x00, 10);   // NORMAL_Power
  writeToBNO(REG_SYS_TRIGGER, 0x80, 20);  // External Oscillator
  writeToBNO(REG_OPR_MODE, 0x0C, 80);   // NDOF Mode (9軸フュージョン)

  Serial.println("BNO055 ready.");
}
float get_compass_angle() {
  Wire.beginTransmission(BNO055_ADDR);
  Wire.write(REG_EULER_H_LSB);
  Wire.endTransmission(false);

  if (Wire.requestFrom(BNO055_ADDR, (uint8_t)6) != 6) return 0.0f;

  int16_t raw_yaw   = merge(Wire.read(), Wire.read());
  int16_t raw_roll  = merge(Wire.read(), Wire.read()); // 読み飛ばし防止
  int16_t raw_pitch = merge(Wire.read(), Wire.read());

  // 1度 = 16 LSB
  float yaw = (float)raw_yaw / 16.0f;

  // 0-360 を -180〜180 に変換
  if (yaw > 180.0f) yaw -= 360.0f;

  return yaw;
}
*/

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
float target_angle = 0.0f;
float compass_angle = 0.0f;
float pretime = 0.0f;
float dt = 0.0f;
const float Kp = 6.0;
const float Ki = 3.0;
const float Kd = 0.4;
float P = 0.0f;
float I = 0.0f; 
float D = 0.0f;
float preP = 0.0f;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("starting...");
  for (int i = 0; i < 8; i++) {
    pinMode(motor_pins[i], OUTPUT);
  }

  I2C_BNO.begin();
  // 初期化シーケンス
  write8(0x3D, 0x00); // Config mode
  delay(25);
  write8(0x3F, 0x20); // Reset
  delay(700);
  write8(0x3D, 0x0C); // NDOF mode
  delay(25);

  Serial.println("BNO055 initialized.");
  Mstop();
}

void loop() {
  /*
  compass_angle = get_compass_angle();
  Serial.println(compass_angle);
  delay(100);*/
  
  float current_yaw = get_yaw();
  
  Serial.print("Yaw: ");
  Serial.println(current_yaw);

  delay(100);
}
