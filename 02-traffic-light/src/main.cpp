/*
 * 第 2 课：三色 LED —— 交通灯 & 流水灯
 * =====================================
 * 第一个自己接线的课。代码很短，重点是手上的活：面包板怎么用、
 * LED 怎么分正负极、为什么每个 LED 都必须串一个电阻。
 *
 * 【接线】LED 长脚是正极(+)，短脚是负极(-)。短脚那侧灯体有个平边。
 *
 *   GPIO4 ──[220Ω]──► 红灯长脚 ──► 短脚 ──┐
 *   GPIO5 ──[220Ω]──► 绿灯长脚 ──► 短脚 ──┤──► 面包板负极轨 ──► 板上 GND
 *   GPIO6 ──[100Ω]──► 蓝灯长脚 ──► 短脚 ──┘
 *
 * 【为什么必须串电阻】
 * LED 导通后内阻极小，直接接 3.3V 会流过上百 mA，烧 LED 或烧芯片引脚。
 * 串上电阻，电流就被限制住：  电流 = (3.3V - LED压降) / 电阻
 *   红/绿 LED 压降约 2V  → (3.3-2)/220 ≈ 6mA
 *   蓝 LED   压降约 3V  → (3.3-3)/100 ≈ 3mA（压降高，电阻要用小一点）
 *
 * 【改这里切换演示】1 = 交通灯，2 = 流水灯
 */
#include <Arduino.h>  // pinMode / digitalWrite / delay / Serial 的声明都在这里。
                      // 注: Arduino IDE 会自动帮你加这一行, PlatformIO 不会, 必须自己写。

const int MODE = 1;

const int PIN_RED   = 4;
const int PIN_GREEN = 5;
const int PIN_BLUE  = 6;

const int LEDS[3] = {PIN_RED, PIN_GREEN, PIN_BLUE};

// 三个灯全灭
void allOff() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(LEDS[i], LOW);
  }
}

// 演示 1：交通灯
void trafficLight() {
  digitalWrite(PIN_RED, HIGH);    // 红灯：停
  delay(3000);
  digitalWrite(PIN_RED, LOW);

  digitalWrite(PIN_GREEN, HIGH);  // 绿灯：行
  delay(3000);
  digitalWrite(PIN_GREEN, LOW);

  // 蓝灯闪 3 下，对应真实红绿灯里的黄灯（手头没有黄色 LED）
  for (int i = 0; i < 3; i++) {
    digitalWrite(PIN_BLUE, HIGH);
    delay(300);
    digitalWrite(PIN_BLUE, LOW);
    delay(300);
  }
}

// 演示 2：流水灯
void runningLight() {
  for (int i = 0; i < 3; i++) {
    allOff();
    digitalWrite(LEDS[i], HIGH);
    delay(200);
  }
}

void setup() {
  // 串口：调试的第一工具。上传后打开串口监视器就能看到这一行。
  Serial.begin(115200);
  Serial.println("Lesson 02: traffic light / running light");

  // 告诉芯片：这三个脚要用来"输出"电压
  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BLUE, OUTPUT);

  allOff();  // 上电先全灭，避免"随机亮"
}

// 注意 delay() 是"阻塞"的：等待期间芯片什么都不做。
// 本课程序只干这一件事，没问题；以后做多任务时会换成 millis() 的写法。
void loop() {
  if (MODE == 1) {
    trafficLight();
  } else {
    runningLight();
  }
}
