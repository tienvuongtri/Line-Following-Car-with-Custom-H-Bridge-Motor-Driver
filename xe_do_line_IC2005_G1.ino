#include "BluetoothSerial.h"

// ==================== KHAI BÁO PIN ĐỘNG CƠ (GIỮ NGUYÊN) ====================
BluetoothSerial SerialBT;

#define ENA  14
#define IN1  27
#define IN2  26
#define IN3  25
#define IN4  33
#define ENB  32

// ==================== KHAI BÁO PIN CẢM BIẾN ====================
#define S1 17   // Trái ngoài cùng
#define S2 19   // Trái trong
#define S3 4    // Giữa
#define S4 16   // Phải trong
#define S5 23   // Phải ngoài cùng

#define TRIG   5
#define ECHO   18
#define BUZZER 15
#define LED    2

// ==================== PWM ESP32 ====================
#define PWM_CHANNEL_A 0
#define PWM_CHANNEL_B 1
#define PWM_FREQ 1000
#define PWM_RES 8

// ==================== BIẾN TOÀN CỤC & PID ====================
int systemMode = 0;        
int SPEED_BASE = 120; // Lấy theo initial_motor_speed của code mẫu
int manualSpeed = 180;
char lastCmd = 'S';
int stepCounter = 1; 
unsigned long lastCrossTime = 0; 


// Các thông số PID từ code mẫu (Bạn có thể cần tinh chỉnh lại Kd trên sân)
float Kp = 20.0;
float Ki = 0.3;
float Kd = 50.0;

int error = 0;
int previous_error = 0;
float P = 0, I = 0, D = 0;
float PID_value = 0;

unsigned long lastPingTime = 0;
unsigned long lastTimePID = 0; 
float currentDist = 999;
float dt = 0;

// ==================== HÀM HỖ TRỢ ====================
void setAlarm(bool state) {
  digitalWrite(BUZZER, state ? HIGH : LOW);
  digitalWrite(LED, state ? HIGH : LOW);
}

void beep(int ms) {
  setAlarm(true); delay(ms); setAlarm(false);
}

// Hàm motor đã được tối ưu để dùng cho ESP32 và hỗ trợ quay ngược (âm)
void motor(int left, int right) {
  digitalWrite(IN1, left >= 0 ? HIGH : LOW);
  digitalWrite(IN2, left >= 0 ? LOW  : HIGH);
  ledcWrite(PWM_CHANNEL_A, abs(left));

  digitalWrite(IN3, right >= 0 ? HIGH : LOW);
  digitalWrite(IN4, right >= 0 ? LOW  : HIGH);
  ledcWrite(PWM_CHANNEL_B, abs(right));
}

void stopM() {
  motor(0, 0);
}

float getDistance() {
  digitalWrite(TRIG, LOW); delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long t = pulseIn(ECHO, HIGH, 4000); 
  return (t == 0) ? 999 : t * 0.034 / 2.0;
}


// ==================== DÒ LINE SA HÌNH FPT ====================
void lineFollow() {
  int l1 = digitalRead(S1); 
  int l2 = digitalRead(S2); 
  int l3 = digitalRead(S3); 
  int l4 = digitalRead(S4); 
  int l5 = digitalRead(S5); 

  // 1. NHẬN DIỆN GIAO CẮT & ĐẾM BƯỚC
  // Nếu mắt ngoài cùng chạm vạch VÀ đã qua ít nhất 0.8s kể từ ngã rẽ trước (chống đếm đúp)
  if ((l1 == 0 || l5 == 0) && (millis() - lastCrossTime > 800)) {
    lastCrossTime = millis(); // Reset đồng hồ

    // Thực hiện lệnh rẽ tương ứng với từng chốt trên chữ FPT
    switch(stepCounter) {
      
      case 1: // CHỐT 1: Ngã 3 giữa chữ F -> Phải rẽ Phải
        motor(180, -180); delay(150); 
        while(digitalRead(S3) == 1) { motor(180, -180); }
        break;

      case 2: // CHỐT 2: Góc gập 45 độ lên chéo -> Phải rẽ Trái
        motor(-180, 180); delay(100); // Delay ngắn hơn vì góc 45 độ
        while(digitalRead(S3) == 1) { motor(-180, 180); }
        break;

      case 3: // CHỐT 3 (BẪY): Đâm chéo vào chữ P -> Ép rẽ Trái (Đi lên)
        motor(-180, 180); delay(150); 
        while(digitalRead(S3) == 1) { motor(-180, 180); }
        break;

      case 4: // CHỐT 4: Đỉnh chữ P -> Rẽ Phải
        motor(180, -180); delay(150); 
        while(digitalRead(S3) == 1) { motor(180, -180); }
        break;

      case 5: // CHỐT 5: Cắt ngang thân chữ T -> ÉP ĐI THẲNG QUA
        motor(SPEED_BASE, SPEED_BASE);
        delay(200); // Lướt qua vạch ngang mà không bẻ lái
        break;

      case 6: // CHỐT 6: Đỉnh nhọn chữ T -> Rẽ Phải góc gắt 135 độ
        motor(180, -180); delay(200); // Delay lâu hơn để quay sâu vào trong
        while(digitalRead(S3) == 1) { motor(180, -180); }
        break;

      case 7: // CHỐT 7 (BẪY): Đâm chéo vào thân chữ T -> Ép rẽ Trái (Đi xuống Finish)
        motor(-180, 180); delay(150); 
        while(digitalRead(S3) == 1) { motor(-180, 180); }
        break;
    }

    // Tăng bộ đếm cho ngã rẽ tiếp theo, đặt lại lỗi PID để xe chạy thẳng mượt
    stepCounter++;
    previous_error = 0;
    lastTimePID = micros(); 
    return; // Rẽ xong thì thoát luôn, nhường vòng lặp tiếp theo cho PID
  }

  // 2. NẾU MẤT VẠCH TRÊN ĐƯỜNG ĐI (Khắc phục sự cố)
  if (l5 == 1 && l4 == 1 && l3 == 1 && l2 == 1 && l1 == 1) {
    if (previous_error > 0) { error = 5; } 
    else if (previous_error < 0) { error = -5; }
  }

  // 3. TÍNH TOÁN LỖI CHO ĐOẠN ĐƯỜNG THẲNG/CHÉO
  else {
    if      (l5 == 1 && l4 == 1 && l3 == 1 && l2 == 1 && l1 == 0) { error = 4; }
    else if (l5 == 1 && l4 == 1 && l3 == 1 && l2 == 0 && l1 == 0) { error = 3; }
    else if (l5 == 1 && l4 == 1 && l3 == 1 && l2 == 0 && l1 == 1) { error = 2; }
    else if (l5 == 1 && l4 == 1 && l3 == 0 && l2 == 0 && l1 == 1) { error = 1; }
    else if (l5 == 1 && l4 == 1 && l3 == 0 && l2 == 1 && l1 == 1) { error = 0; }
    else if (l5 == 1 && l4 == 0 && l3 == 0 && l2 == 1 && l1 == 1) { error = -1; }
    else if (l5 == 1 && l4 == 0 && l3 == 1 && l2 == 1 && l1 == 1) { error = -2; }
    else if (l5 == 0 && l4 == 0 && l3 == 1 && l2 == 1 && l1 == 1) { error = -3; }
    else if (l5 == 0 && l4 == 1 && l3 == 1 && l2 == 1 && l1 == 1) { error = -4; }
  }

  // 4. CHẠY PID ĐỂ GIỮ THĂNG BẰNG
  unsigned long now = micros();
  dt = (now - lastTimePID) / 1000000.0;
  if (dt <= 0.0 || dt > 0.1) dt = 0.005; 
  lastTimePID = now;

  P = error;
  I = I + error * dt;
  D = (error - previous_error) / dt; 
  I = constrain(I, -50, 50); 
  
  PID_value = Kp * P + Ki * I + Kd * D;
  previous_error = error;

  PID_value = constrain(PID_value, -180, 180);

  int leftSpeed  = SPEED_BASE - PID_value;
  int rightSpeed = SPEED_BASE + PID_value;
  leftSpeed  = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  motor(leftSpeed, rightSpeed);
}

// ==================== SETUP & LOOP ====================
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_PRO_CAR");

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, PWM_CHANNEL_A);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENB, PWM_CHANNEL_B);

  pinMode(S1, INPUT); pinMode(S2, INPUT); pinMode(S3, INPUT);
  pinMode(S4, INPUT); pinMode(S5, INPUT);

  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  pinMode(BUZZER, OUTPUT); pinMode(LED, OUTPUT);

  stopM();
  beep(300);
  lastTimePID = micros(); 
}

void loop() {
  if (SerialBT.available()) {
    char cmd = SerialBT.read();
    if (cmd == '1') { systemMode = 1; beep(100); }
    else if (cmd == '0') { systemMode = 0; stopM(); beep(500); }
    else if (cmd=='F'||cmd=='B'||cmd=='L'||cmd=='R'||cmd=='S') {
      systemMode = 2; lastCmd = cmd;
      switch(cmd) {
        case 'F': motor(manualSpeed, manualSpeed); break;
        case 'B': motor(-manualSpeed, -manualSpeed); break;
        case 'L': motor(-manualSpeed, manualSpeed); break;
        case 'R': motor(manualSpeed, -manualSpeed); break;
        case 'S': stopM(); break;
      }
    }
  }

  if (systemMode == 1) {
    if (millis() - lastPingTime > 60) {
      currentDist = getDistance();
      lastPingTime = millis();
    }
    if (currentDist < 15.0) {
      stopM(); beep(200);
      motor(-100, -100); delay(300);
      motor(-130, 130); delay(450);
      currentDist = 999;
      lastTimePID = micros(); 
    } else {
      lineFollow();
    }
  }
  else if (systemMode == 2) {
    float dist = getDistance();
    if (dist < 8.0 && lastCmd == 'F') {
      setAlarm(true); stopM();
    } else {
      setAlarm(false);
    }
  }
  delay(4);
}