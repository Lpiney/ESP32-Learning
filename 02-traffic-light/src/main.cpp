
#include <Arduino.h>

const int PIN_RED = 4;
const int PIN_GREEN = 5;
const int PIN_BLUE = 6;
void setup() {
  pinMode(PIN_RED, OUTPUT);     // 只做一次：把 GPIO4 设成输出
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BLUE, OUTPUT);
}

void loop() {
  digitalWrite(PIN_RED, HIGH);  // 亮
  digitalWrite(PIN_GREEN, HIGH);
  digitalWrite(PIN_BLUE, HIGH);
  delay(100);                  // 等 1000 毫秒（1 秒）
  digitalWrite(PIN_RED, LOW);   // 灭
  digitalWrite(PIN_GREEN, LOW);
  digitalWrite(PIN_BLUE, LOW);
  delay(100);
  
}
