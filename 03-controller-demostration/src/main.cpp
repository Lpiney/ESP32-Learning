/* ============================================================================
 * 03 · 摇杆双电机遥控器 (Joystick Dual-Motor Controller)
 * ----------------------------------------------------------------------------
 * 一块 ESP32-S3 独立完成三件事，不依赖电脑、不依赖别的板子：
 *
 *      PS2 摇杆 ──> 算左右电机脉宽 ──> 两路 50Hz PWM ──> 两个电调/电机
 *                    │
 *                    └──> 同一帧画到 OLED 上
 *
 * 画面上看到什么，电机就输出什么 —— 用的是同一份数据。
 *
 * ┌──────────────────────────────────────────────────────────┐
 * │ READY L=2000 R=1000     X +0.42                          │ ← 预热状态 + 两轴数值
 * │                         Y -0.17                          │
 * │            · · · · · · · ·                               │ ← 外圈：行程边界 ±1.0
 * │          ·                 ·                             │
 * │         ·     · · · ·       ·                            │ ← 内圈：死区 ±0.15
 * │        ·     ·       ·       ·     OUT                   │
 * │        ·     ·   ✛   ·       ·                           │
 * │         ·     · · · ·       ·                            │
 * │          ·        ●        ·                             │
 * │            · · · · · · · ·                               │
 * │  X ─────────────┼───────●────────                        │ ← 两轴一维展开
 * │  Y ─────────────┼────────────●───                        │
 * └──────────────────────────────────────────────────────────┘
 *
 * 知识点
 *   1. ADC 读模拟电压（12 位 + 11dB 衰减）、必须用 ADC1
 *   2. 开机自动校准中点（中位数采样）
 *   3. 死区(deadzone) 与「线性重标定」为什么比硬截断手感好
 *   4. 差速转向：两轮同速前进、反向原地转
 *   5. LEDC 产生 50Hz 舵机/电调 PWM
 *   6. 安全两道闸：开机预热窗口、油门变化限速
 *   7. 一阶低通滤波、双缓冲刷屏
 *
 * 接线表见同目录 README.md
 * ==========================================================================*/

#include <Arduino.h>
#include <Wire.h>            // I2C（OLED 用）
#include <math.h>            // sinf / cosf / sqrtf / lroundf / fabsf
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* ==========================  1. 引脚定义（改这里就能改接线）  ============== */

// ---- 两个电调 ----
// 注意：这块 N16R8 板子的 PSRAM 走 Octal 模式，GPIO33~37 被 PSRAM 占了，
//       绝对不能接任何东西，所以下面选的脚全部避开了 33~37。
//
// 电调的三根细线：信号(白/橙) -> 下面的 GPIO，地(黑/棕) -> GND，
// 电源(红) 一般留空不接（电调自己从动力电取电，BEC 5V 更不要接进 3.3V）。
#define PIN_ESC_L       4    // 左电机信号
#define PIN_ESC_R       5    // 右电机信号

// LEDC 通道号（ESP32-S3 一共 8 个，挑两个没用到的）
#define ESC_CH_L        0
#define ESC_CH_R        1

// ---- PS2 摇杆 ----
// URX / URY 必须接 ADC1 的通道 (GPIO1~GPIO10)，原因见 README。
#define PIN_JOY_X       1    // 模块丝印 URX
#define PIN_JOY_Y       2    // 模块丝印 URY
// GPIO3 是 strapping 脚，但上电要求为高电平，内部上拉正好满足；
// 我们只把它当按键输入用，不影响启动。
#define PIN_JOY_SW      3    // 模块丝印 SW (按下 = 低电平)

// ---- OLED (4 脚 I2C 版 SSD1306, 128x64) ----
#define PIN_OLED_SDA    8    // 模块丝印 SDA
#define PIN_OLED_SCL    9    // 模块丝印 SCL

/* ==========================  2. 可调参数  ================================ */

#define SCREEN_W      128
#define SCREEN_H       64
#define I2C_ADDR     0x3C    // SSD1306 地址，常见 0x3C / 0x3D
#define OLED_I2C_HZ  400000UL

#define ADC_MAX       4095   // analogReadResolution(12) -> 0~4095

/* ---- 电调 PWM ---- */
// 行业标准：50Hz 刷新，脉宽 1000~2000us 表示油门行程，1500us 是中立。
//
// ★★★ 首次给电调通电之前必须先看这一条 ★★★
//   双向电调（能倒车，ROV 一般用这种）：中立 = 1500us  ← 本课用这个
//   单向电调（只能往前转）            ：中立 = 1000us
//   猜错的后果：上电瞬间螺旋桨直接疯转。所以首次通电一定要先脱桨！
//   改这里：把 PULSE_NEUTRAL 改成 1000。
//
// 关于 3000us：本课的固件是**直接**输出 PWM 的，所以要改就能改。但 3000us
// 超出常规电调 1000~2000 的行程，发出去多半被当成无效信号（电调可能不动、
// 报警、或进入失控保护），而不是"更大的油门"。要改请先量出你那台电调的实际
// 行程（只接一路、脱桨、脉宽从 1000 每 10us 往上加到它开始转/到顶）。
#define ESC_PWM_FREQ_HZ   50
#define ESC_PWM_RES_BITS  14        // S3 的 LEDC 最大 14 位；50Hz 下一格 ≈1.22us
#define ESC_PERIOD_US     20000     // 1/50Hz = 20ms

#define PULSE_MIN       1000
#define PULSE_MAX       2000
#define PULSE_NEUTRAL   1500

// 油门变化限速：每毫秒最多变化多少 us。
// 1000us 的满行程 -> 1000/2 = 500ms 才走完，避免"啪"一下满油门。
// 想让手感更直接可以调大；调到 10000 基本等于不限速。
#define SLEW_US_PER_MS  2.0f

/* ---- 安全参数 ---- */
// 上电后先忽略摇杆这么久，然后**自动开始响应**。
//
// 为什么需要它：电调上电要自检/响铃，这时候如果你手正搭在摇杆上，
// 它会直接读到油门就开始转。所以开机先给一个窗口期，把摇杆当没动。
//
// 这里没有"解锁/上锁"状态机：那玩意儿每次都要按一下开关，台面上调试很烦。
// 代价是板子通电就随时可能转 —— 所以测试时请脱桨，手边留着电源开关。
#define BOOT_IGNORE_MS  3000

// 长按 SW 多久算「重新校准」（中点漂了才需要，平时用不到）
#define LONG_PRESS_MS   800
// 按键消抖
#define DEBOUNCE_MS     30

/* ---- 摇杆参数 ---- */
// 死区：中点为圆心、半径 15% 的圆内算「没动」。真摇杆回弹不可能每次都精确
// 回到同一个电压，加上 ADC 噪声，不给死区的话松手后输出会一直抖。
#define JOY_DEADZONE    0.15f

// ADC 抖动滤波系数：新值占 0.6，旧值占 0.4
#define SMOOTHING       0.6f

// 开机校准采样次数
#define CAL_SAMPLES     64

// X 轴的符号：+1 表示"推右 = X 正数"，-1 表示取反。
//
// 约定是「把摇杆往右推，X 应该是正数」（屏幕上的点也往右走，和电脑上的
// 模拟器一致）。2026-10-03 用 tools/joy_check.py 实测这颗模块：推右时读到
// 的是**负**数，所以取反。
//
// 这不是线接错了：电位器哪一头对应低电平由模块自己决定，两种都正常，
// **只能测出来，从代码推不出来**。换模块或重新接线后再反，就把它改回 +1.0f
// （或者跑一遍 tools/joy_check.py，它会直接告诉你要不要改）。
#define JOY_X_SIGN      (-1.0f)

/* ---- 单电机测试模式（第一次给电调通电时用）---- */
// 1 = 只测一个电机：摇杆**前后**直接控制输出，**左右摇杆完全不起作用**。
//     两路输出给的是同一个值，所以电调插 GPIO4 还是 GPIO5 都能测，
//     不用管接线 —— 变量减到最少：只验证"推上 = 电机往前进方向转"。
// 0 = 正常差速控制（左右分开 + 转向）。那套代码一行都没动，就在下面
//     joyToMotorTargets() 的 #else 里，把这里改回 0 就完全恢复。
//
// 注意：这个开关只改固件，tools/motor_sim.py 不必跟着动 —— 模拟器读的是
// 串口里的 L/R，测试模式下两路相等，画面上就是"直行"，本来就该这样。
#define SINGLE_MOTOR_TEST  0

// 测试模式下的油门系数：0.35 = 推到底也只有 35% 油门。
// 第一次转电机用小的更安全（限速 SLEW_US_PER_MS 只是让它变柔和，不限制幅度）。
// 方向确认好之后改成 1.0f 就能全油门。
#define TEST_THROTTLE      0.95f

/* ---- 输入来源：板载摇杆 还是 电脑上的手柄 ---- */
// 1 = **电脑手柄模式**：手柄插在电脑上，电脑每 20ms 通过串口发一行指令
//      CMD nx=+0.42 ny=-0.17 en=1
//     (nx/ny 是 -1~+1；en=1 表示"已解锁"，en=0 表示锁着 -> 强制中立。
//      安全键是**开关式**的：默认锁住，按一下解锁、再按一下锁上，
//      这一层的逻辑在电脑端 pad_bridge.py 的 Latch 里。)
//     板载摇杆那套代码完整保留在下面各个 #else 里，改回 0 就完全恢复。
// 0 = 板载 PS2 摇杆（第 3 课原样）
//
// 电脑端跑：python3 tools/pad_bridge.py
#define PC_CONTROL  1

// PC 模式下多久没收到指令就算**断流**，强制回中立。
// 必须要有这一条：电脑/脚本/线一挂，电机不能带着最后一条指令一直转。
// 200ms ≈ 10 个发送周期，正常抖动不会误触发。
#define PC_TIMEOUT_MS  200

/* ---- 时序 ---- */
#define CONTROL_MS      8       // 电机控制周期 ≈125Hz（要跟手，所以比刷屏快）
#define DRAW_INTERVAL_MS 25     // 刷屏 ≈40fps（I2C 屏刷一整屏要 ~25ms）
#define TELEMETRY_MS    50      // 遥测 ≈20Hz

#define SERIAL_BAUD     115200

/* ==========================  3. 全局对象  =============================== */

// I2C 版构造函数：(宽, 高, Wire指针, 复位脚, 时钟频率)
// 4 脚模块没有独立的 RES 脚（复位走 I2C 命令），所以复位脚传 -1。
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1, OLED_I2C_HZ);

// 上一帧的画面，用来做「有什么变化吗」的比较
static uint8_t prevFrame[SCREEN_W * SCREEN_H / 8];

// 摇杆校准结果：中点 + 两个方向各自的满量程
struct AxisCal {
  int center;   // 松手时的 ADC 读数（理想 2048，实际每个模块都不同）
  int spanLo;   // center - 打到最左边时的读数
  int spanHi;   // 打到最右边时的读数 - center
};
static AxisCal calX, calY;

// 平滑后的 ADC 值
static float filtX = 0.0f, filtY = 0.0f;

// ---- 电机状态 ----
// 注意这里**没有** locked/armed 之类的状态：摇杆永远直接控制电机，
// 唯一的门是开机后的 BOOT_IGNORE_MS 预热窗口（见 motor.readyAt）。
struct MotorState {
  uint32_t readyAt   = 0;                 // 到这个时刻之后才开始响应摇杆
  bool     ready     = false;             // 预热窗口过了吗（只用于显示/上报）
  int      targetL   = PULSE_NEUTRAL;     // 摇杆算出来的目标脉宽
  int      targetR   = PULSE_NEUTRAL;
  float    outL      = PULSE_NEUTRAL;     // 限速之后的实际输出（浮点，平滑用）
  float    outR      = PULSE_NEUTRAL;
  int      writtenL  = -1;                // 已经写进 LEDC 的值，避免重复写
  int      writtenR  = -1;
};
static MotorState motor;

static uint32_t lastControl = 0;
static uint32_t lastDraw    = 0;
static uint32_t lastTelemetry = 0;

#if PC_CONTROL
// 给**显示**用的两个状态位（loop() 每圈更新，renderFrame() 只读它们）：
//   pcLinkOk   电脑链路在线吗 -> OLED 显示 LOCK/UNLOCK 还是 NOLINK
//   pcUnlocked 安全键解锁了吗 -> OLED 显示 UNLOCK 还是 LOCK
// 绘图函数比"输入解析"那段靠前，而且它本来也不该去掏解析器的内部结构，
// 所以这里只放这两个位，别的都不暴露。
static bool pcLinkOk = false;
static bool pcUnlocked = false;
#endif

/* ==========================  4. 摇杆校准与读取  ========================= */

// 采 n 次取中位数：中位数不怕偶发的毛刺（静电、电源波动）
static int sampleMedian(uint8_t pin) {
  int buf[CAL_SAMPLES];
  for (int i = 0; i < CAL_SAMPLES; i++) {
    buf[i] = analogRead(pin);
    delay(2);
  }
  // 插入排序：数组很小，够用
  for (int i = 1; i < CAL_SAMPLES; i++) {
    int key = buf[i], j = i - 1;
    while (j >= 0 && buf[j] > key) {
      buf[j + 1] = buf[j];
      j--;
    }
    buf[j + 1] = key;
  }
  return buf[CAL_SAMPLES / 2];
}

// 开机校准：此时**手不要碰摇杆**。
//
// 为什么要校准：PS2 摇杆是两个电位器，机械零点有误差，ADC 也有误差，所以
// 「松手时的读数」几乎不可能是理想的 2048。不校准的话，松手时程序就会以为
// 你一直在往上推，电机就自己转起来了。
//
// spanLo/spanHi 取满量程的 0.75 倍（留余量），保证推到底能到 ±1.0 而不溢出。
static void calibrateJoystick() {
  Serial.println();
  Serial.println(F("=== 摇杆校准 ==="));
  Serial.println(F("请松开摇杆，不要用手碰它..."));
  delay(1200);   // 留时间让手离开

  calX.center = sampleMedian(PIN_JOY_X);
  calY.center = sampleMedian(PIN_JOY_Y);

  const int nominal = ADC_MAX / 2;         // 2047
  const int margin  = (nominal * 3) / 4;   // 1535 ≈ 满量程 75%
  calX.spanLo = calX.spanHi = margin;
  calY.spanLo = calY.spanHi = margin;

  Serial.printf("X 中点 = %4d   Y 中点 = %4d   (理想值 2048)\n",
                calX.center, calY.center);
  Serial.printf("满量程 ±%d  (= %.2f V 摆幅)\n", margin,
                margin * 3.3f / 4096.0f);
  Serial.println(F("校准完成。"));
  Serial.println();

  // 播报给电脑上的模拟器，它就不需要自己猜中点了
  Serial.printf("CAL XC=%d XSL=%d XSH=%d YC=%d YSL=%d YSH=%d\n",
                calX.center, calX.spanLo, calX.spanHi,
                calY.center, calY.spanLo, calY.spanHi);
}

// ADC 原始值 -> 归一化坐标 (-1.0 ~ +1.0)
//
// 中点是 0，推到两端分别是 -1 和 +1。中点到两端的实际距离不一定相等
// （电位器不一定对称），所以两边各用一个 span，这样两个方向推到底都能
// 刚好打到 ±1.0。
static float mapAxis(int raw, const AxisCal &c) {
  float v;
  if (raw < c.center) {
    v = -(float)(c.center - raw) / (float)c.spanLo;
  } else {
    v = (float)(raw - c.center) / (float)c.spanHi;
  }
  if (v > 1.0f) v = 1.0f;
  if (v < -1.0f) v = -1.0f;
  return v;
}

static bool readButtonRaw() {
  return digitalRead(PIN_JOY_SW) == LOW;   // 按下接地 -> LOW
}

/* ==========================  5. 摇杆 -> 双电机  ========================= */

// 死区 + 线性重标定。
//
// 重标定是必要的：直接硬截断的话，摇杆刚离开死区那一刻输出会从 0 突跳到
// deadzone(0.15)，手感是"一顿"。重标定把它拉回 0 连续起步。
static float applyDeadzone(float v) {
  float a = fabsf(v);
  if (a <= JOY_DEADZONE) return 0.0f;
  float n = (a - JOY_DEADZONE) / (1.0f - JOY_DEADZONE);
  return (v < 0.0f) ? -n : n;
}

static int unitToPulse(float v) {
  if (v > 1.0f) v = 1.0f;
  if (v < -1.0f) v = -1.0f;
  return (int)lroundf(PULSE_NEUTRAL + v * (PULSE_MAX - PULSE_NEUTRAL));
}

// 摇杆位置 -> 左右目标脉宽（差速转向）
//
//   fwd  = y        前后
//   turn = x        左右（**后退时取反**，见下面 dir）
//   左 = fwd + turn
//   右 = fwd - turn
//   最后线性映射到 [PULSE_MIN, PULSE_MAX]，中立 1500
//
//   Y=+1, X= 0 -> L=2000 R=2000   两路同速前进（前提：两路电调都把 2000 当"前进"）
//   Y=-1, X= 0 -> L=1000 R=1000   两路同速后退
//   Y= 0, X=+1 -> L=2000 R=1000   原地：左路出力多 → 车头往右转
//   Y= 0, X=-1 -> L=1000 R=2000   原地：右路出力多 → 车头往左转
//   Y=+1, X=+1 -> L=2000 R=1500   前进中推右 → 车头往右偏
//   Y=-1, X=+1 -> L=1000 R=1500   后退中推右 → **车尾往右走**（和前进时相反）
//   Y=-1, X=-1 -> L=1500 R=1000   后退中推左 → 车尾往左走
//
// ★ 为什么"推右"要让**左**路出力多：这是普通差速车的运动学（电脑上的
//   模拟器用的就是这套）—— 左边快、右边慢，车头就往右偏。
//
// ★ 这一步和上面的 JOY_X_SIGN 是**一对**，必须一起定：
//     推右 → X=+1 → 左路出力多 → 右转
//   只改其中一个会把方向弄反。2026-10-03 前面两次改不对，就是因为在
//   两个符号之间来回动单个（改完一处，另一处就把它抵消掉了）。
//
// ★ 真船上还要再验一次：两个推进器若是对称反装的（或某一路相线反了），
//   实际转向会和这里相反。接上电机跑 python3 tools/joy_check.py 第 6 步，
//   它会问你看现象、直接告诉你这两行要不要对调。要改的话，
//   tools/motor_sim.py 的 motor_pulses() 和 tools/test_firmware_logic.py
//   里同样的两行必须一起改，否则自检会报错。
static void joyToMotorTargets(float nx, float ny, int &targetL, int &targetR) {
  float y = applyDeadzone(ny);

#if SINGLE_MOTOR_TEST
  /* ---- 单电机测试模式（SINGLE_MOTOR_TEST 见文件上方）----
   * 只让摇杆的前后说了算：两路输出同一个值、左右摇杆被完全忽略。
   * 目的就是"一次只验证一件事"：推上，电机是不是往前进方向转。 */
  targetL = unitToPulse(y * TEST_THROTTLE);
  targetR = unitToPulse(y * TEST_THROTTLE);
#else
  float x = applyDeadzone(nx);

  // 后退时把左右取反 —— 方向盘惯例（也是真车倒车的行为）：
  //   往右打方向，车**尾**往右走。不取反就是"坦克惯例"：不管前进后退，
  //   车头总是顺时针转。两者都自洽，选一个即可；这里选方向盘惯例。
  //
  // 代价要说清楚：摇杆前后经过中点时（y 穿 0），转向会**翻一次**。真车也是
  // 这样 —— 停着不动的时候方向盘怎么打都不会转弯。摇杆 y 有死区
  // (JOY_DEADZONE)，所以这个翻转发生在"后退 → 死区 → 前进"之间，
  // 那一段本来就是"原地自转"，不是"边走边转"。
  const float dir = (y < 0.0f) ? -1.0f : 1.0f;

  targetL = unitToPulse(y + dir * x);
  targetR = unitToPulse(y - dir * x);
#endif
}

/* ==========================  6. 电机输出 (LEDC)  ======================== */

// 脉宽(us) -> LEDC 占空比。14 位下整个周期 20000us 对应 16384 格。
static uint32_t usToDuty(int us) {
  return (uint32_t)us * (1u << ESC_PWM_RES_BITS) / ESC_PERIOD_US;
}

static void writePWM(int l, int r) {
  if (l != motor.writtenL) { ledcWrite(ESC_CH_L, usToDuty(l)); motor.writtenL = l; }
  if (r != motor.writtenR) { ledcWrite(ESC_CH_R, usToDuty(r)); motor.writtenR = r; }
}

// 开机配置 LEDC 并**立刻**输出中立脉宽。
//
// 为什么要立刻？电调上电时会看信号线：看到合法的中立脉宽，它就完成自检然后
// 安静待命；线路悬空则会一直报警或干脆不工作。
static void motorsBegin() {
  ledcSetup(ESC_CH_L, ESC_PWM_FREQ_HZ, ESC_PWM_RES_BITS);
  ledcSetup(ESC_CH_R, ESC_PWM_FREQ_HZ, ESC_PWM_RES_BITS);
  ledcAttachPin(PIN_ESC_L, ESC_CH_L);
  ledcAttachPin(PIN_ESC_R, ESC_CH_R);

  motor.outL = motor.outR = (float)PULSE_NEUTRAL;
  motor.targetL = motor.targetR = PULSE_NEUTRAL;
  motor.readyAt = millis() + BOOT_IGNORE_MS;   // 预热窗口
  motor.ready = false;
  writePWM(PULSE_NEUTRAL, PULSE_NEUTRAL);
}

// 高频调用：限速 + 写 PWM。dtMs 是距上次调用的毫秒数。
//
// 这里就是全部逻辑：预热窗口内当作摇杆没动，之后摇杆直接说了算。
// 没有解锁、没有上锁、没有状态机。
static void motorsTick(float dtMs) {
  uint32_t now = millis();

  if (!motor.ready && (int32_t)(now - motor.readyAt) >= 0) {
    motor.ready = true;
    Serial.println(F("READY: 预热结束，摇杆开始生效"));
  }

  // 预热窗口内：把目标压成中立（摇杆动了也不响应）
  if (!motor.ready) {
    motor.targetL = motor.targetR = PULSE_NEUTRAL;
  }

  // 油门限速：朝目标每次最多走 SLEW_US_PER_MS * dtMs
  float maxStep = SLEW_US_PER_MS * (dtMs > 0.0f ? dtMs : 1.0f);
  float dL = (float)motor.targetL - motor.outL;
  float dR = (float)motor.targetR - motor.outR;
  if (dL >  maxStep) dL =  maxStep;
  if (dL < -maxStep) dL = -maxStep;
  if (dR >  maxStep) dR =  maxStep;
  if (dR < -maxStep) dR = -maxStep;
  motor.outL += dL;
  motor.outR += dR;

  writePWM((int)lroundf(motor.outL), (int)lroundf(motor.outR));
}

/* ==========================  7. 绘图小工具  ============================= */

// 画一段「虚线椭圆弧」。用像素长度决定虚实，这样弧长一点短一点看起来一样。
// 分开传 rx/ry 是为了把圈纵向压扁：屏幕只有 64 行高，压扁后圈占 y≈11~33，
// 上面留给状态行、下面留给两条横线。
static void dashedArc(int cx, int cy, int rx, int ry, float aStart, float aEnd,
                      int dashPx, int gapPx) {
  int period = dashPx + gapPx;
  if (period <= 0) period = 1;
  int px = 0;
  bool prev = false;
  for (float a = aStart; a <= aEnd; a += 0.02f) {
    bool on = (px % period) < dashPx;
    if (on || prev) {   // 每小段首尾都画，避免空隙
      display.drawPixel(cx + (int)lroundf(rx * cosf(a)),
                        cy + (int)lroundf(ry * sinf(a)), SSD1306_WHITE);
    }
    prev = on;
    px++;
  }
}

// 画一条中间留缺口的横线，用来当中点标记。
//
// 为什么不用一根竖线做中点刻度？因为竖线有高度，会顶到上面那一行文字。
// 留缺口只需要一行，物理上不可能和文字冲突。
static void hLineWithGap(int x, int y, int w, int gap, uint16_t color) {
  int mid = x + w / 2;
  int half = gap / 2;
  int x0 = mid - half;
  int x1 = mid + half;
  if (x0 > x)      display.drawFastHLine(x, y, x0 - x, color);
  if (x1 < x + w)  display.drawFastHLine(x1, y, x + w - x1, color);
}

/* ==========================  8. 画面一帧  =============================== */

static void renderFrame(float nx, float ny, bool inDead, int outL, int outR) {
  display.clearDisplay();   // 只清内存缓冲，还没动屏幕

  /* ---- 画面分区（这几条线是算出来的，保证互不重叠）----
   *
   *    x=0~41              x=17~47            x=56~127
   *   ┌──────────────────┬────────────────┬─────────────┐ y=0
   *   │ READY / LOCK     │                │ L=1500      │ ← 第一行：摇杆模式显示
   *   │ DEAD             │                │ R=1500      │ y=9  预热状态；电脑手柄
   *   └──────────────────┴────────────────┴─────────────┘      模式显示锁的状态
   *   ├──────────────────┤   ● 行程圈      │ X +0.42     │ y=20
   *   │                  │   圆心(32,30)   │ Y -0.17     │ y=31
   *   │                  │   rx=13 ry=10  │             │
   *   ├──────────────────┴────────────────┴─────────────┤ y=44 以下留白
   *   │ X ────────────────┄┄┄───────●────               │ y=50  缺口 = 中点
   *   │ Y ────────────────┄┄┄─────●──────               │ y=58
   *   └─────────────────────────────────────────────────┘ y=63
   *
   * 三条硬约束（拿椭圆方程逐行算出来的，不是估的）：
   *   a) 摇杆点（含半径 2）只在 x=17~47、y=18~42 之间跑
   *      （所以左上角第一行最多 6 个字符：2 + 6×6 = 38 ≤ 41。
   *        要加更长的字，先跑 tools/verify_layout.py）
   *   b) 左栏文字限制在 x≤41，右栏从 x=56 起；因为点永远出不了 x=47，
   *      所以左右两栏都不会被点压到
   *   c) y≥44 整条留白，底部两行横线才能从 x=18 一直画到 x=105
   *
   * 右栏只有 y=0~38 可用（下面让给横线），塞不下"L/R + X/Y + DEAD"五行，
   * 所以 DEAD 挪到左上角第二行。
   */
  const int cx = 32, cy = 30;
  const int rx = 13, ry = 10;

  // 8.1 行程圈（虚线）+ 死区圈（点线）
  dashedArc(cx, cy, rx, ry, 0.0f, 6.2832f, 2, 3);
  int rdx = (int)lroundf(JOY_DEADZONE * rx);
  int rdy = (int)lroundf(JOY_DEADZONE * ry);
  if (rdx < 3) rdx = 3;
  if (rdy < 3) rdy = 3;
  dashedArc(cx, cy, rdx, rdy, 0.0f, 6.2832f, 1, 3);

  // 十字准星
  display.drawFastHLine(cx - 5, cy, 11, SSD1306_WHITE);
  display.drawFastVLine(cx, cy - 5, 11, SSD1306_WHITE);

  // 8.2 当前摇杆位置：从圆心连线到点
  int dotX = cx + (int)lroundf(nx * rx);
  // 减号：屏幕的 y 是**向下**增大的，而约定是"推上 = ny 正数"，
  // 所以这里要翻过来 —— 推上时点画在上面。
  //
  // 2026-10-06 实测确认过（两条输入路径的 Y 符号都是"推上为正"：
  // 板载摇杆那边测电机时"推上 = 前进"，手柄这边 --show-input 直接看到
  // 轴 1 推上为正）。所以这里用减号对两条路径都对。
  // ⚠ 别因为"看着像反的"就改成加号 —— 改之前先拿手推一下看屏幕。
  int dotY = cy - (int)lroundf(ny * ry);
  display.drawLine(cx, cy, dotX, dotY, SSD1306_WHITE);
  display.fillCircle(dotX, dotY, 2, SSD1306_WHITE);

  // 8.3 文字
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setTextWrap(false);   // 关键：不让长行折到下一行糊成一片

  // 左上角：状态 + 死区状态（两行，x≤41 都在圆的通道之外）
  // 第二行统一是"现在有没有在出力"；第一行两种模式不一样：
#if PC_CONTROL
  // 电脑手柄模式：显示**锁**的状态。比赛现场这个比预热状态重要得多：
  //   LOCK   = 锁着（安全键没按，推摇杆也不出力）
  //   UNLOCK = 已解锁，摇杆说了算
  //   NOLINK = 电脑那边没在发指令（断流 / 脚本没跑），此时强制中立
  display.setCursor(2, 0);
  display.print(!pcLinkOk ? F("NOLINK")
                          : (pcUnlocked ? F("UNLOCK") : F("LOCK")));
#else
  display.setCursor(2, 0);
  display.print(motor.ready ? F("READY") : F("WAIT"));
#endif

  display.setCursor(2, 9);
  display.print(inDead ? F("DEAD") : F("OUT "));

  // 右栏：实际输出的左右脉宽（屏幕上的数和电机收到的完全一致）
  display.setCursor(56, 0);
  display.print(F("L="));
  display.print(outL);

  display.setCursor(56, 9);
  display.print(F("R="));
  display.print(outR);

  // 右栏：归一化后的摇杆位置
  display.setCursor(56, 20);
  display.print(F("X "));
  display.print(nx, 2);
  display.setCursor(56, 31);
  display.print(F("Y "));
  display.print(ny, 2);

  /* ---- 8.4 底部：两轴一维展开（线中间的缺口就是中点标记）----
   * 横线从 x=18 画到 x=105，所以 x≥56 的区域必须留空，不能放文字。 */
  const int barX = 18, barW = 88, barY = 48, barH = 8;
  int barCy = barY + 2;

  display.setCursor(4, 50);
  display.print('X');
  hLineWithGap(barX, barCy, barW, 6, SSD1306_WHITE);
  display.fillCircle(barX + (int)lroundf((nx + 1.0f) * 0.5f * barW), barCy, 2,
                     SSD1306_WHITE);

  display.setCursor(4, 57);
  display.print('Y');
  hLineWithGap(barX, barCy + barH, barW, 6, SSD1306_WHITE);
  display.fillCircle(barX + (int)lroundf((ny + 1.0f) * 0.5f * barW),
                     barCy + barH, 2, SSD1306_WHITE);

  // 8.5 刷屏：只有画面真的变了才发数据（I2C 刷整屏要 ~25ms，省下来很值）
  uint8_t *cur = display.getBuffer();
  if (memcmp(cur, prevFrame, sizeof(prevFrame)) != 0) {
    display.display();
    memcpy(prevFrame, cur, sizeof(prevFrame));
  }
}

/* ==========================  8.5 电脑手柄输入  ========================== */
#if PC_CONTROL
// 电脑发来的一行：CMD nx=+0.42 ny=-0.17 en=1
//
// 解析规矩和遥测那边一样：**逐 token 找 key=value**，不许用
// s.split(" X=") 那种切法（行里同时有 nx= 和 X=，硬切迟早切错）。
// 不认识的 key 直接跳过 —— 以后加字段不会把这里弄坏。
struct PcInput {
  float    nx = 0.0f;      // 电脑算好的归一化摇杆值
  float    ny = 0.0f;
  bool     enable = false; // 安全键是否按住
  uint32_t lastCmdMs = 0;  // 上一条**合法**指令的到达时刻
};

static PcInput  pc;
static char     cmdBuf[96];
static uint8_t  cmdLen = 0;
static uint32_t pcCmds = 0, pcBadLines = 0;

static bool parseCmdLine(const char *s) {
  if (strncmp(s, "CMD", 3) != 0) return false;

  bool gotNx = false, gotNy = false, gotEn = false;
  float nx = pc.nx, ny = pc.ny;
  bool en = pc.enable;

  const char *p = s + 3;
  while (*p) {
    while (*p == ' ') p++;
    if (!*p) break;
    const char *eq = strchr(p, '=');
    if (eq == nullptr) return false;         // 有半个 token，丢掉整行
    const size_t klen = (size_t)(eq - p);
    const char  *val  = eq + 1;

    if      (klen == 2 && !strncmp(p, "nx", 2)) { nx = strtof(val, nullptr); gotNx = true; }
    else if (klen == 2 && !strncmp(p, "ny", 2)) { ny = strtof(val, nullptr); gotNy = true; }
    else if (klen == 2 && !strncmp(p, "en", 2)) { en = (atoi(val) != 0);     gotEn = true; }

    p = val;
    while (*p && *p != ' ') p++;
  }

  // 缺字段的行不算数 —— 宁可不动，也不要拿半个指令去驱动电机
  if (!(gotNx && gotNy && gotEn)) return false;

  pc.nx = nx;
  pc.ny = ny;
  pc.enable = en;
  pc.lastCmdMs = millis();
  pcCmds++;
  return true;
}

// 把串口里已经收到的字节吃掉，按行喂给 parseCmdLine()
static void pollPcInput() {
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      cmdBuf[cmdLen] = '\0';
      if (cmdLen > 0 && !parseCmdLine(cmdBuf)) pcBadLines++;
      cmdLen = 0;
      continue;
    }
    if (cmdLen < sizeof(cmdBuf) - 1) cmdBuf[cmdLen++] = ch;
    else cmdLen = 0;        // 太长：丢掉整行，别把缓冲区撑爆
  }
}
#endif  // PC_CONTROL

/* ==========================  9. setup / loop  =========================== */

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);   // 等串口稳定，否则开机前几行会丢
  Serial.println(F("\n\n=== 03 摇杆双电机遥控器 ==="));

  // --- 电调：第一件事就是把中立脉宽挂上去 ---
  motorsBegin();

  // --- 摇杆按键 ---
#if !PC_CONTROL
  pinMode(PIN_JOY_SW, INPUT_PULLUP);   // 内部上拉：松开 HIGH，按下接地 LOW

  // --- ADC ---
  // 12 位分辨率 + 11dB 衰减：11dB 量程约 0~3.1V，摇杆 0~3.3V 的摆动正好落在里面。
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_JOY_X, ADC_11db);
  analogSetPinAttenuation(PIN_JOY_Y, ADC_11db);
  for (int i = 0; i < 8; i++) {        // 丢几次，让 ADC 稳定
    analogRead(PIN_JOY_X);
    analogRead(PIN_JOY_Y);
  }

  calibrateJoystick();

  filtX = (float)analogRead(PIN_JOY_X);   // 用中点当滤波初值
  filtY = (float)analogRead(PIN_JOY_Y);
#endif

  // --- OLED ---
  // 必须先 setPins 再 begin：库内部会调 Wire.begin()，begin 过再设就不生效了。
  Wire.setPins(PIN_OLED_SDA, PIN_OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, I2C_ADDR, true, true)) {
    Serial.println(F("!! OLED 初始化失败：检查 SDA/SCL 有没有接反、VCC 是不是 3.3V"));
    Serial.printf("   (当前试的地址 0x%02X，有些模块是 0x3D)\n", I2C_ADDR);
    // 屏坏了也让遥控器能工作：不卡死，只是没有显示
    Serial.println(F("!! 继续运行（电机控制不依赖屏幕）"));
  } else {
    display.setRotation(0);
    display.cp437(true);
    display.clearDisplay();
    display.display();
    memset(prevFrame, 0, sizeof(prevFrame));
    Serial.printf("OLED OK: %dx%d @ %lu Hz I2C (addr 0x%02X)\n",
                  SCREEN_W, SCREEN_H, (unsigned long)OLED_I2C_HZ, I2C_ADDR);
  }

  Serial.printf("开机预热 %d ms。\n", BOOT_IGNORE_MS);
#if PC_CONTROL
  Serial.println(F("输入来源：**电脑手柄**。等 tools/pad_bridge.py 发指令；"));
  Serial.printf("断流超过 %d ms、或安全键没按住 -> 强制中立。\n", PC_TIMEOUT_MS);
  Serial.println(F("屏幕：WAIT/READY = 开机预热而已；DEAD = 当前没在出力"));
  Serial.println(F("      （断流 / 安全键松开 / 摇杆在中点），OUT = 正在出力。"));
#else
  Serial.println(F("输入来源：板载摇杆。预热结束摇杆自动生效；长按 SW (>0.8s) 重新校准中点。"));
#endif
  Serial.println(F("★ 首次给电调通电前请脱桨，并确认 PULSE_NEUTRAL 与你的电调匹配。"));
}

void loop() {
  const uint32_t now = millis();

  // ---- 9.1 输入：板载摇杆 或 电脑手柄 ----
  int   rawX = 0, rawY = 0;    // PC 模式没有 ADC 读数；只为遥测行格式统一而保留
  float nx, ny;
  bool  inDead;
#if PC_CONTROL
  pollPcInput();

  // 断流判定：从来没收到过（lastCmdMs==0）也算没连上
  pcLinkOk = (pc.lastCmdMs != 0) &&
             ((int32_t)(now - pc.lastCmdMs) < PC_TIMEOUT_MS);
  pcUnlocked = pc.enable;          // 给 OLED 显示用
  nx = pcLinkOk ? pc.nx : 0.0f;
  ny = pcLinkOk ? pc.ny : 0.0f;
  const float mag = sqrtf(nx * nx + ny * ny);

  // 屏幕上 DEAD 的含义：**现在没有在出力** —— 没连上、还锁着、预热没过、或摇杆在中点。
  // 不用改绘图代码：renderFrame() 本来就只是照着 inDead 显示 DEAD/OUT。
  inDead = !motor.ready || !pcLinkOk || !pc.enable || (mag < JOY_DEADZONE);
#else
  rawX = analogRead(PIN_JOY_X);
  rawY = analogRead(PIN_JOY_Y);
  filtX += SMOOTHING * ((float)rawX - filtX);
  filtY += SMOOTHING * ((float)rawY - filtY);

  nx = JOY_X_SIGN * mapAxis((int)lroundf(filtX), calX);
  ny = mapAxis((int)lroundf(filtY), calY);
  const float mag = sqrtf(nx * nx + ny * ny);
  inDead = (mag < JOY_DEADZONE);

  // ---- 9.2 按键：只用来重新校准（长按 >0.8s）----
  // 摇杆不需要任何按键就能用：上电预热完就一直有效。
  static bool     rawPrev    = false;   // 上一帧的原始电平（消抖用）
  static bool     stable     = false;   // 消抖后的状态
  static uint32_t changedAt  = 0;
  static uint32_t pressedAt  = 0;

  const bool raw = readButtonRaw();
  if (raw != rawPrev) { rawPrev = raw; changedAt = now; }
  if ((int32_t)(now - changedAt) >= DEBOUNCE_MS && stable != raw) {
    stable = raw;
    if (stable) {
      pressedAt = now;                     // 刚按下
    } else if ((int32_t)(now - pressedAt) >= LONG_PRESS_MS) {
      // 刚松开且按了很久 -> 重新校准中点
      calibrateJoystick();
      filtX = (float)analogRead(PIN_JOY_X);
      filtY = (float)analogRead(PIN_JOY_Y);
    }
  }
#endif

  // ---- 9.3 输入 -> 目标脉宽 ----
#if PC_CONTROL
  // 安全两道闸：断流、或安全键没按住 -> 强制中立。
  // 这里不看摇杆值 —— 电脑算好什么就是什么。
  if (pcLinkOk && pc.enable) {
    joyToMotorTargets(nx, ny, motor.targetL, motor.targetR);
  } else {
    motor.targetL = motor.targetR = PULSE_NEUTRAL;
  }
#else
  joyToMotorTargets(nx, ny, motor.targetL, motor.targetR);
#endif

  // ---- 9.4 电机控制（限速 + 写 PWM），固定 8ms 节拍 ----
  if ((int32_t)(now - lastControl) >= CONTROL_MS) {
    float dtMs = (float)(now - lastControl);
    lastControl = now;
    motorsTick(dtMs);
  }

  // ---- 9.5 刷屏，25ms 节拍 ----
  if ((int32_t)(now - lastDraw) >= DRAW_INTERVAL_MS) {
    lastDraw = now;
    renderFrame(nx, ny, inDead, (int)lroundf(motor.outL), (int)lroundf(motor.outR));
  }

  // ---- 9.6 遥测，50ms 一行，给电脑上的 Motor-Sim / pad_bridge 监视 & 调试 ----
  if ((int32_t)(now - lastTelemetry) >= TELEMETRY_MS) {
    lastTelemetry = now;
    // PC=1 表示输入来自电脑手柄；EN=1 允许出力（安全键按住）；LINK=1 输入在线。
    // 摇杆模式下 EN/LINK 恒为 1（有线摇杆没什么可断的）。
    // 新加的字段都**追加在行尾**，老解析器忽略不认识的 token，不会把旧工具弄坏。
    const int pcFlag = (PC_CONTROL) ? 1 : 0;
#if PC_CONTROL
    const int enFlag = pc.enable ? 1 : 0;
    const int linkFlag = pcLinkOk ? 1 : 0;
#else
    const int enFlag = 1;
    const int linkFlag = 1;
#endif
    Serial.printf(
        "JOY JX=%d JY=%d X=%+.3f Y=%+.3f MAG=%.3f %s READY=%d "
        "L=%d R=%d TL=%d TR=%d PC=%d EN=%d LINK=%d\n",
        rawX, rawY, nx, ny, mag, inDead ? "DEAD" : "OUT",
        motor.ready ? 1 : 0,
        (int)lroundf(motor.outL), (int)lroundf(motor.outR),
        motor.targetL, motor.targetR,
        pcFlag, enFlag, linkFlag);
  }

  // 让出 CPU 给别的任务（含看门狗）
  vTaskDelay(pdMS_TO_TICKS(1));
}
