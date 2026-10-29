#include <Arduino.h>

#define SDA_PIN 3
#define SCL_PIN 2
#define STC8_ADDR 0x30 // Địa chỉ gốc của chip STC8

void i2c_start() {
  digitalWrite(SDA_PIN, HIGH); delayMicroseconds(1);
  digitalWrite(SCL_PIN, HIGH); delayMicroseconds(1);
  digitalWrite(SDA_PIN, LOW);  delayMicroseconds(1);
  digitalWrite(SCL_PIN, LOW);  delayMicroseconds(1);
}

void i2c_stop() {
  digitalWrite(SCL_PIN, LOW);  delayMicroseconds(1);
  digitalWrite(SDA_PIN, LOW);  delayMicroseconds(1);
  digitalWrite(SCL_PIN, HIGH); delayMicroseconds(1);
  digitalWrite(SDA_PIN, HIGH); delayMicroseconds(1);
}

bool i2c_recvACK() {
  bool ack;
  digitalWrite(SCL_PIN, LOW);
  pinMode(SDA_PIN, INPUT); 
  digitalWrite(SCL_PIN, HIGH); delayMicroseconds(1);
  ack = digitalRead(SDA_PIN);
  digitalWrite(SCL_PIN, LOW);
  pinMode(SDA_PIN, OUTPUT);
  return ack;
}

void i2c_sendByte(uint8_t dat) {
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(SCL_PIN, LOW); delayMicroseconds(1);
    digitalWrite(SDA_PIN, (dat & 0x80) ? HIGH : LOW);
    dat <<= 1;
    delayMicroseconds(1);
    digitalWrite(SCL_PIN, HIGH); delayMicroseconds(1);
  }
}

void writeRegister(uint8_t regAddr, uint8_t value) {
  i2c_start();
  i2c_sendByte(STC8_ADDR << 1); 
  i2c_recvACK();
  i2c_sendByte(regAddr);        
  i2c_recvACK();
  i2c_sendByte(value);          
  i2c_recvACK();
  i2c_stop();
}

// motor: 1 (Trước Phải), 2 (Trước Trái), 3 (Sau Phải), 4 (Sau Trái)
// dir: 1 (Tiến), 0 (Lùi)
// pwm: 0 -> 255
void setMotor(uint8_t motor, bool dir, uint8_t pwm) {
  uint8_t regIN1, regIN2;
  
  if (motor == 1)      { regIN1 = 0x01; regIN2 = 0x02; }
  else if (motor == 2) { regIN1 = 0x03; regIN2 = 0x04; }
  else if (motor == 3) { regIN1 = 0x05; regIN2 = 0x06; }
  else if (motor == 4) { regIN1 = 0x07; regIN2 = 0x08; }
  else return;

  if (dir) {
    writeRegister(regIN1, 0);
    writeRegister(regIN2, pwm);
  } else {
    writeRegister(regIN1, pwm);
    writeRegister(regIN2, 0);
  }
}

void setup() {
  pinMode(SDA_PIN, OUTPUT);
  pinMode(SCL_PIN, OUTPUT);
  
  // Tắt tất cả động cơ khi khởi động
  for(int i=1; i<=4; i++) {
    setMotor(i, 1, 0);
  }
}

void loop() {
  // Ví dụ: Cho xe đi thẳng với tốc độ 100 trong 2 giây
  setMotor(1, 1, 100); // Trước Phải Tiến
  setMotor(2, 1, 100); // Trước Trái Tiến
  setMotor(3, 1, 100); // Sau Phải Tiến
  setMotor(4, 1, 100); // Sau Trái Tiến
  delay(2000);
  
  // Dừng xe 2 giây
  for(int i=1; i<=4; i++) setMotor(i, 1, 0);
  delay(2000);
}