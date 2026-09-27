/*
 * Step 9 —— 加固: WiFi 自动重连 + 错误状态上屏
 *
 * 【这个程序干什么】
 * 前八步做出来的是一个"演示品": 只要一切顺利它就跑得很好, 但**出一点意外
 * 它就永久罢工**。这一步把它变成一个真能摆在桌上、拔掉电脑自己转的东西。
 *
 * ============================================================
 * 【Step 8 留下的两个致命问题】
 *
 * 问题一: 开机时如果路由器还没开, setup() 里 connectWiFi() 失败就 return 了。
 *         设备从此卡死, 路由器后来开了也没用 —— 必须重启。
 *
 * 问题二: 运行中 WiFi 掉线(路由器重启、信号抖动), 程序完全不知道,
 *         还在那儿每秒重画时间, 天气却永远是旧的。
 *
 * 【根因: 阻塞式写法】
 * Step 8 以前, 连 WiFi / 对时 / 拿天气 全都是"发起 -> 原地死等 -> 成功或超时"。
 * 这种写法在开机时没问题 (反正没别的事可做), 但**在运行中绝对不能用**:
 * 原地等 20 秒, 时钟就停 20 秒, 屏幕上秒针不动、分钟不跳, 看起来就是死机。
 *
 * 【解法: 服务函数 + 状态变量】
 *
 * 把"等"这件事从代码里删掉, 改成:
 *
 *     - 每个任务拆成一个 service 函数, 调用一次只做"现在该做的那一点点"
 *     - 该不该做, 由两个东西决定: 一个**状态变量** + 一个**下次执行的时刻**
 *     - loop() 飞快地转圈 (每 200ms), 谁到点了就叫谁干活
 *
 *     void loop() {
 *         uint32_t now = millis();
 *         serviceWiFi(now);       // 要不要发起连接 / 重连?
 *         serviceNtp(now);        // 要不要对时?
 *         serviceWeather(now);    // 要不要刷新天气?
 *         serviceDisplay(now);    // 画面要不要重画?
 *         delay(200);
 *     }
 *
 * 这种结构叫"协作式多任务"。它没有操作系统, 也不需要 —— 只要每个服务函数
 * 都**立刻返回**, 不霸占 CPU, 四件事就能像并行一样各自按自己的节奏走。
 *
 * 判断标准很简单: **任何一次 loop() 的耗时可预测且有上限**。
 *
 * 【状态变量 —— 让不同任务之间能"看见"彼此】
 *     wifiOk         WiFi 现在连着吗
 *     timeReady      时间对上了吗
 *     weatherFailed  上次请求天气成功了吗
 *     g_weather      最后一次拿到的天气数据
 *     displayDirty   画面需要重画吗
 *
 * 注意 displayDirty 这个变量。屏幕刷新很慢 (要往 I2C 上推 1KB), 不可能每圈
 * 都刷。所以改成"谁改动了状态, 谁把 displayDirty 立起来", 画完再放倒。
 * 这叫**脏标记 (dirty flag)**, 是界面编程里最常用的优化手段之一。
 *
 * ============================================================
 * 【屏幕上会出现的几种状态 —— 作品要能自己说话】
 *
 *   一切正常          → 正常显示
 *   WiFi 掉线         → 湿度那行变成反白的 "NO WIFI"
 *   天气请求失败      → 那行变成 "retrying"
 *   一次都没拿到天气  → 天气区域显示 "No network / check router"
 *   时间还没对上      → 时间显示 "--:--", 右上角显示 "SYNC"
 *
 * 【概念: 反白 (inverse video)】
 * 想让一行字扎眼, 最有效的手段不是加感叹号, 而是**黑底白字** ——
 * 在一个满是白底黑字的屏幕上, 唯一一块反色的区域会自动抓住视线。
 * OLED 是单色的, 所以"反白"就是用 fillRect 涂一块白底, 再把文字颜色
 * 设成 SSD1306_BLACK 画上去。
 *
 * ★ 有个坑必须记住: 画完一定要把文字颜色**改回白色**。
 *   不然之后所有的绘制都是黑的, 屏幕上什么都看不见, 而且极难排查。
 * ============================================================
 *
 * 【Step 6~8 的内容保持不变, 说明见下】
 * gzip: 和风天气无视 Accept-Encoding 永远返回压缩数据, 用 ROM 自带的
 *       miniz 手工解压 (gunzip)。判据: 头两字节 1f 8b。
 * JSON: 所有数值字段都是**字符串** ("temp":"26"), 要 atoi 转, 不能用 as<int>()。
 * 中文: 字体画不出中文, 用 icon 编号查表换成英文 + 32x32 位图。
 */

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <time.h>

// ESP32-S3 芯片 ROM 里自带的 miniz 解压库 (不占 flash)
#include "rom/miniz.h"

// 32x32 天气图标位图, 由 tools/make_icons.py 生成
#include "weather_icons.h"

// ==================== 私人配置 ====================
// WiFi 密码 / 天气 API Key 都放在 src/secrets.h, 不随代码上传 GitHub。
// 换电脑或克隆代码后: 复制 src/secrets.h.example 为 src/secrets.h, 填入自己的值。
#include "secrets.h"
// =================================================

// ==================== 屏幕 ====================
static const int PIN_SDA = 8;
static const int PIN_SCL = 9;
static const int SCREEN_W = 128;
static const int SCREEN_H = 64;
static const uint8_t OLED_ADDR = 0x3C;

static Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);
// ==============================================

// ==================== 时区 / NTP ====================
static const long TIMEZONE_OFFSET_SEC = 8 * 3600;
static const int DST_OFFSET_SEC = 0;
static const char *NTP_SERVER_1 = "ntp.aliyun.com";
static const char *NTP_SERVER_2 = "cn.pool.ntp.org";
static const char *NTP_SERVER_3 = "pool.ntp.org";
// ==================================================

static const uint32_t HTTP_TIMEOUT_MS = 10000;

// ---- 各种"多久以后再说"的间隔 ----
//
// 重连间隔不能太短: 路由器对短时间内反复连接的设备会开启防抖
// (表现是串口刷 "Association refused temporarily"), 越急越连不上。
static const uint32_t WIFI_RETRY_INTERVAL_MS = 15UL * 1000UL;
static const uint32_t NTP_RETRY_INTERVAL_MS = 10UL * 1000UL;

// 天气: 成功就 10 分钟后再来; 失败 30 秒后重试。
// 不区分这两种情况的话, 一次网络抖动就要黑屏 10 分钟。
static const uint32_t WEATHER_OK_INTERVAL_MS = 10UL * 60UL * 1000UL;
static const uint32_t WEATHER_RETRY_INTERVAL_MS = 30UL * 1000UL;

// 主循环转一圈的间隔
static const uint32_t LOOP_INTERVAL_MS = 200;

// 星期缩写 (3 个字母, 省地方)
static const char *WEEKDAY_ABBR[] = {"Sun", "Mon", "Tue", "Wed",
                                     "Thu", "Fri", "Sat"};

// ===============================================================
// 全局状态 —— 各个服务函数靠这些变量互相"看见"
// ===============================================================

static bool wifiOk = false;         // WiFi 现在连着吗
static bool timeReady = false;      // 时间对上了吗
static bool ntpStarted = false;     // SNTP 客户端启动过吗
static bool weatherFailed = true;   // 上次天气请求成功了吗
static bool displayDirty = true;    // 画面需要重画吗

// 各任务"下次该干活"的时刻 (millis() 时间戳)
static uint32_t wifiNextMs = 0;
static uint32_t ntpNextMs = 0;
static uint32_t weatherNextMs = 0;

// ---------------------------------------------------------------
// 屏幕辅助
// ---------------------------------------------------------------

static void drawCentered(const char *text, int16_t y, uint8_t size) {
    display.setTextSize(size);
    display.setTextColor(SSD1306_WHITE);

    int16_t bx, by;
    uint16_t w, h;
    display.getTextBounds(text, 0, 0, &bx, &by, &w, &h);

    display.setCursor((SCREEN_W - (int16_t)w) / 2, y);
    display.print(text);
}

static void showMessage(const char *line1, const char *line2) {
    display.clearDisplay();
    drawCentered(line1, 22, 1);
    if (line2) {
        drawCentered(line2, 38, 1);
    }
    display.display();
}

// ===============================================================
// 天气状况对照表: icon 编号 -> {英文描述, 图标位图}
// ===============================================================

struct WeatherKind {
    const char *code;     // 和风天气的 icon 编号
    const char *text;     // 英文描述 (屏幕上显示这行)
    const uint8_t *icon;  // 32x32 位图
};

/*
 * 编号规律: 第一位数字是大类
 *     1xx 晴/多云/阴   3xx 雨   4xx 雪   5xx 雾霾沙尘   9xx 特殊
 * 后面两位是细分。表里没列到的编号, 由下面的 fallback 按大类兜底。
 *
 * 注意 100 和 150 都是"晴", 区别是白天版 (太阳) 和夜间版 (月亮)。
 */
static const WeatherKind WEATHER_TABLE[] = {
    // ---- 1xx: 晴 / 多云 / 阴 ----
    {"100", "Sunny", ICON_SUN},
    {"101", "Cloudy", ICON_CLOUD},
    {"102", "Few Clouds", ICON_PARTLY},
    {"103", "Partly Cloudy", ICON_PARTLY},
    {"104", "Overcast", ICON_CLOUD},
    {"150", "Clear", ICON_MOON},  // 晴 (夜间)
    {"151", "Cloudy", ICON_CLOUD},
    {"152", "Few Clouds", ICON_PARTLY},
    {"153", "Partly Cloudy", ICON_PARTLY},
    {"154", "Overcast", ICON_CLOUD},

    // ---- 3xx: 雨 (302/303/304 是雷, 单独给个闪电图标) ----
    {"300", "Shower", ICON_RAIN},
    {"301", "Heavy Shower", ICON_RAIN},
    {"302", "Thunderstorm", ICON_STORM},
    {"303", "Heavy Storm", ICON_STORM},
    {"304", "Thunder Hail", ICON_STORM},
    {"305", "Light Rain", ICON_RAIN},
    {"306", "Moderate Rain", ICON_RAIN},
    {"307", "Heavy Rain", ICON_RAIN},
    {"308", "Extreme Rain", ICON_RAIN},
    {"309", "Drizzle", ICON_RAIN},
    {"310", "Rainstorm", ICON_RAIN},
    {"311", "Heavy Rainstorm", ICON_RAIN},
    {"312", "Severe Rainstorm", ICON_RAIN},
    {"313", "Freezing Rain", ICON_RAIN},
    {"399", "Rain", ICON_RAIN},

    // ---- 4xx: 雪 ----
    {"400", "Light Snow", ICON_SNOW},
    {"401", "Moderate Snow", ICON_SNOW},
    {"402", "Heavy Snow", ICON_SNOW},
    {"403", "Blizzard", ICON_SNOW},
    {"404", "Sleet", ICON_SNOW},
    {"405", "Rain and Snow", ICON_SNOW},
    {"406", "Snow Shower", ICON_SNOW},
    {"407", "Snow Shower", ICON_SNOW},
    {"408", "Light to Mod Snow", ICON_SNOW},
    {"409", "Mod to Heavy Snow", ICON_SNOW},
    {"410", "Heavy Snow to Blizzard", ICON_SNOW},
    {"499", "Snow", ICON_SNOW},

    // ---- 5xx: 雾 / 霾 / 沙尘 ----
    {"500", "Mist", ICON_FOG},
    {"501", "Fog", ICON_FOG},
    {"502", "Haze", ICON_FOG},
    {"503", "Sand", ICON_FOG},
    {"504", "Dust", ICON_FOG},
    {"507", "Duststorm", ICON_FOG},
    {"508", "Sandstorm", ICON_FOG},
    {"509", "Dense Fog", ICON_FOG},
    {"510", "Strong Fog", ICON_FOG},
    {"511", "Moderate Haze", ICON_FOG},
    {"512", "Severe Haze", ICON_FOG},
    {"513", "Heavy Haze", ICON_FOG},
    {"514", "Heavy Fog", ICON_FOG},
    {"515", "Extra Heavy Fog", ICON_FOG},

    // ---- 9xx: 特殊 ----
    {"900", "Hot", ICON_SUN},
    {"901", "Cold", ICON_SNOW},
};

// 表里没列到的编号, 按大类兜底。
// 单独命名而不是写 WEATHER_TABLE[10] 这种下标 —— 下标会随表的增删悄悄错位。
static const WeatherKind FALLBACK_1 = {"1xx", "Cloudy", ICON_CLOUD};
static const WeatherKind FALLBACK_3 = {"3xx", "Rain", ICON_RAIN};
static const WeatherKind FALLBACK_4 = {"4xx", "Snow", ICON_SNOW};
static const WeatherKind FALLBACK_5 = {"5xx", "Fog", ICON_FOG};
static const WeatherKind FALLBACK_X = {"???", "Unknown", ICON_UNKNOWN};

/*
 * 查表。三层保险, 保证永远返回一条有效记录, 调用方不用判空:
 *   1) 编号精确匹配
 *   2) 没命中就按第一位数字 (大类) 兜底
 *   3) 连大类都认不出来, 用问号图标
 */
static const WeatherKind *lookupWeather(const char *icon) {
    if (icon == nullptr || icon[0] == '\0') {
        return &FALLBACK_X;
    }

    for (const WeatherKind &k : WEATHER_TABLE) {
        if (strcmp(k.code, icon) == 0) {
            return &k;
        }
    }

    switch (icon[0]) {
        case '1':
            return &FALLBACK_1;
        case '3':
            return &FALLBACK_3;
        case '4':
            return &FALLBACK_4;
        case '5':
            return &FALLBACK_5;
        default:
            return &FALLBACK_X;
    }
}

// ===============================================================
// 网络部分
// ===============================================================

/*
 * 解压 gzip。头部 10 字节 + 尾部 8 字节要剥掉, 中间的 deflate 数据
 * 才能喂给 ROM 里的 tinfl。详见 Step 6 的说明。
 */
static size_t gunzip(const uint8_t *src, size_t srcLen, char *out,
                     size_t outCap) {
    const size_t GZIP_HEADER_LEN = 10;
    const size_t GZIP_TRAILER_LEN = 8;

    if (srcLen <= GZIP_HEADER_LEN + GZIP_TRAILER_LEN) {
        return 0;
    }

    size_t outLen = tinfl_decompress_mem_to_mem(
        out, outCap, src + GZIP_HEADER_LEN,
        srcLen - GZIP_HEADER_LEN - GZIP_TRAILER_LEN, 0);

    if (outLen == 0 || outLen > outCap) {
        return 0;
    }
    return outLen;
}

struct WeatherData {
    bool valid;           // 拿到过有效数据吗
    int temp;             // 温度 °C
    int feelsLike;        // 体感温度 °C
    int humidity;         // 相对湿度 %
    int windScale;        // 风力等级
    const char *text;     // 英文描述
    const uint8_t *icon;  // 32x32 位图
};

static WeatherData g_weather = {false, 0, 0, 0, 0, "--", ICON_UNKNOWN};

/*
 * 请求 + 解压 + 解析。
 * 注意这个函数**会阻塞**几秒 (HTTPS 握手 + 下载 + 解压), 这是没办法的 ——
 * 一次网络交互本来就是需要时间的。我们能控制的是: 多久才做一次。
 * 所以它由 serviceWeather 按 10 分钟的节奏调用, 而不是每圈都调。
 *
 * 成功返回 true。
 */
static bool fetchWeather(WeatherData *out) {
    String url = String("https://") + QWEATHER_HOST + "/v7/weather/now" +
                 "?location=" + QWEATHER_LOCATION + "&key=" + QWEATHER_KEY;

    // 不打印完整 URL —— 里面有 API Key
    Serial.println("---- 刷新天气 ----");

    WiFiClientSecure client;
    client.setInsecure();  // 不校验服务器证书 (个人作品可接受)

    HTTPClient http;
    http.setTimeout(HTTP_TIMEOUT_MS);

    if (!http.begin(client, url)) {
        Serial.println("!! http.begin 失败");
        return false;
    }

    int status = http.GET();
    if (status != HTTP_CODE_OK) {
        Serial.printf("!! HTTP 状态码 %d\n", status);
        http.end();
        return false;
    }

    String body = http.getString();
    http.end();

    // 确认是 gzip —— 头两字节 1f 8b
    const uint8_t *raw = (const uint8_t *)body.c_str();
    if (body.length() < 2 || raw[0] != 0x1F || raw[1] != 0x8B) {
        Serial.println("!! 返回的不是 gzip 数据");
        return false;
    }

    // static: 4KB 的缓冲区放栈上有溢出风险, 而且只需要一份
    static char json[4096];
    size_t n = gunzip(raw, body.length(), json, sizeof(json) - 1);
    if (n == 0) {
        Serial.println("!! 解压失败");
        return false;
    }
    json[n] = '\0';

    // ---- 解析 ----
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, json);
    if (err) {
        Serial.printf("!! JSON 解析失败: %s\n", err.c_str());
        return false;
    }

    const char *code = doc["code"] | "???";
    if (strcmp(code, "200") != 0) {
        Serial.printf("!! 业务失败, code = %s\n", code);
        return false;
    }

    // 所有字段都是**字符串**, 所以先取 const char* 再 atoi。
    // `| "0"` 是 ArduinoJson 的默认值语法: 字段不存在时用右边的值。
    const char *iconStr = doc["now"]["icon"] | "";
    out->temp = atoi(doc["now"]["temp"] | "0");
    out->feelsLike = atoi(doc["now"]["feelsLike"] | "0");
    out->humidity = atoi(doc["now"]["humidity"] | "0");
    out->windScale = atoi(doc["now"]["windScale"] | "0");

    // 中文的 "text" 用不了 (字体画不出中文), 改用 icon 编号查英文和位图
    const WeatherKind *kind = lookupWeather(iconStr);
    out->text = kind->text;
    out->icon = kind->icon;
    out->valid = true;

    Serial.printf("   %d C  %s  (体感 %d, 湿度 %d%%, 风力 %d 级, icon %s)\n",
                  out->temp, out->text, out->feelsLike, out->humidity,
                  out->windScale, iconStr);
    return true;
}

// ===============================================================
// 屏幕主界面
// ===============================================================

/*
 * 画一帧完整的画面。
 *
 * t 为 nullptr 表示时间还没对上 (NTP 还没成功), 这时画占位符。
 *
 * 【为什么要传 t 进来而不是函数内部自己取?】
 * 因为调用它的 serviceDisplay 已经取过一次了。时间在同一个函数里取两次,
 * 有可能一次是 59 分 59 秒、另一次是 00 分 00 秒 —— 画面就会自相矛盾。
 * 取一次, 传下去, 保证一帧画面里所有东西都是同一时刻的。
 */
static void drawScreen(const struct tm *t) {
    char buf[24];

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    // ---------------- 上半屏: 时间 ----------------
    display.setTextSize(2);
    if (t) {
        snprintf(buf, sizeof(buf), "%02d:%02d", t->tm_hour, t->tm_min);
    } else {
        snprintf(buf, sizeof(buf), "--:--");
    }
    display.setCursor(2, 0);
    display.print(buf);

    // 右上角: 正常情况下是星期缩写; 时间还没对上时提示 SYNC。
    // getTextBounds 量出宽度再反推起点, 这样改字号也不会错位。
    display.setTextSize(1);
    const char *corner = t ? WEEKDAY_ABBR[t->tm_wday] : "SYNC";
    int16_t bx, by;
    uint16_t w, h;
    display.getTextBounds(corner, 0, 0, &bx, &by, &w, &h);
    display.setCursor(SCREEN_W - (int16_t)w - 4, 4);
    display.print(corner);

    // 日期: 月/日
    if (t) {
        snprintf(buf, sizeof(buf), "%02d/%02d", t->tm_mon + 1, t->tm_mday);
    } else {
        snprintf(buf, sizeof(buf), "--/--");
    }
    display.setCursor(2, 19);
    display.print(buf);

    // 分隔线
    display.drawLine(0, 29, SCREEN_W - 1, 29, SSD1306_WHITE);

    // ---------------- 下半屏: 天气 ----------------
    if (g_weather.valid) {
        display.drawBitmap(2, 30, g_weather.icon, 32, 32, SSD1306_WHITE);

        // 温度
        snprintf(buf, sizeof(buf), "%d", g_weather.temp);
        display.setTextSize(2);
        display.setCursor(40, 30);
        display.print(buf);

        // 度数符号自己画 —— 字体里没有 '°'。
        // 字号 2 时每个字符宽 12 像素, 所以温度数字的宽度 = 字符数 x 12。
        int16_t tempW = (int16_t)strlen(buf) * 12;
        display.drawCircle(40 + tempW + 4, 34, 2, SSD1306_WHITE);
        display.setCursor(40 + tempW + 8, 30);
        display.print("C");

        // 英文描述
        display.setTextSize(1);
        display.setCursor(40, 46);
        display.print(g_weather.text);

        // ---- 第三行: 正常是湿度, 出问题就变成警告 ----
        // 挑这一行是因为它最不重要, 而且位置正好在右下角, 够显眼。
        if (!wifiOk) {
            // 反白显示。见文件开头说明 —— 画完必须把颜色改回白色,
            // 否则后面所有绘制都是黑的, 屏幕上什么都看不见。
            int16_t wx, wy;
            uint16_t ww, wh;
            display.getTextBounds("NO WIFI", 0, 0, &wx, &wy, &ww, &wh);
            display.fillRect(40 - 2, 53, (int16_t)ww + 4, 10, SSD1306_WHITE);
            display.setTextColor(SSD1306_BLACK);
            display.setCursor(40, 54);
            display.print("NO WIFI");
            display.setTextColor(SSD1306_WHITE);  // ★ 别忘了改回来
        } else if (weatherFailed) {
            display.setCursor(40, 54);
            display.print("retrying");
        } else {
            snprintf(buf, sizeof(buf), "Hum %d%%", g_weather.humidity);
            display.setCursor(40, 54);
            display.print(buf);
        }
    } else {
        // 一次都没成功拿到过天气 —— 告诉人该去查什么, 而不是干等着
        display.setTextSize(1);
        display.setCursor(4, 34);
        if (!wifiOk) {
            display.print("No network");
            display.setCursor(4, 46);
            display.print("check router");
        } else {
            display.print("Weather");
            display.setCursor(4, 46);
            display.print("fetching...");
        }
    }

    // 一帧只推一次。所有绘制都完成后才 display(),
    // 这样屏幕上永远不会出现"画了一半"的中间状态。
    display.display();
}

// ===============================================================
// 四个服务函数 —— 每个都只做"现在该做的一点点", 立刻返回
// ===============================================================

/*
 * WiFi 连接 / 重连。
 *
 * 关键: WiFi.begin() 本身**不阻塞** —— 它只是告诉驱动"去连", 然后立刻返回。
 * 连没连上, 之后用 WiFi.status() 查。所以重连可以完全非阻塞。
 *
 * 真正会阻塞的是"原地等 status 变成已连接", 我们没有那么做。
 */
static void serviceWiFi(uint32_t now) {
    // ---- 情况一: 连着 ----
    if (WiFi.status() == WL_CONNECTED) {
        if (!wifiOk) {
            // 刚连上 (开机首次 或 掉线后恢复)
            wifiOk = true;
            Serial.printf("WiFi 已连接, IP %s, 信号 %d dBm\n",
                          WiFi.localIP().toString().c_str(), WiFi.RSSI());

            // 补上因为没网而欠下的活: 立刻对时、立刻刷天气。
            // 用 now 而不是 0 —— 0 在系统跑了 24 天以后会失效 (见下)。
            ntpNextMs = now;
            weatherNextMs = now;
            displayDirty = true;
        }
        return;
    }

    // ---- 情况二: 没连着 ----
    if (wifiOk) {
        // 从"连着"变成"没连着" —— 刚刚掉线
        wifiOk = false;
        Serial.println("!! WiFi 掉线");
        displayDirty = true;  // 屏幕要立刻把 NO WIFI 显示出来
    }

    // 到重试时间了吗? 没到就什么都不做, 下一圈再问。
    // 这个"看表"的判断是整套非阻塞写法的核心。
    if ((int32_t)(now - wifiNextMs) < 0) {
        return;
    }

    Serial.println("WiFi 连接中...");
    // 先断开再连: 如果上一次连接卡在半路, 不清掉状态会一直失败。
    // 注意这里**没有**原地等结果, 连上了下一圈自然会知道。
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    wifiNextMs = now + WIFI_RETRY_INTERVAL_MS;
}

/*
 * NTP 对时。
 *
 * 只在"WiFi 通了, 但时间还没对上"的时候做事。
 * 一旦对上就永久放手 —— ESP32 的 SNTP 客户端会自己在后台定期校准,
 * 不需要我们操心。断网期间它靠内部晶振继续走, 几小时也就差几十毫秒。
 */
static void serviceNtp(uint32_t now) {
    if (timeReady || !wifiOk) {
        return;
    }

    // 第一次: 启动 SNTP 客户端。这只需要做一次。
    if (!ntpStarted) {
        Serial.println("启动 SNTP 对时");
        configTime(TIMEZONE_OFFSET_SEC, DST_OFFSET_SEC, NTP_SERVER_1,
                   NTP_SERVER_2, NTP_SERVER_3);
        ntpStarted = true;
        ntpNextMs = now + NTP_RETRY_INTERVAL_MS;
        return;
    }

    if ((int32_t)(now - ntpNextMs) < 0) {
        return;
    }

    // getLocalTime 超时给 0 = 不等待, 只查"现在有结果了吗"
    struct tm t;
    if (getLocalTime(&t, 0)) {
        timeReady = true;
        Serial.printf("对时成功: %04d-%02d-%02d %02d:%02d:%02d\n",
                      t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour,
                      t.tm_min, t.tm_sec);
        displayDirty = true;
    } else {
        ntpNextMs = now + NTP_RETRY_INTERVAL_MS;
    }
}

/*
 * 天气刷新。
 *
 * 没网的时候直接跳过 —— 不然要白等 10 秒 HTTP 超时, 时钟会明显卡一下。
 */
static void serviceWeather(uint32_t now) {
    if (!wifiOk) {
        return;
    }
    if ((int32_t)(now - weatherNextMs) < 0) {
        return;
    }

    weatherFailed = !fetchWeather(&g_weather);
    weatherNextMs =
        now + (weatherFailed ? WEATHER_RETRY_INTERVAL_MS : WEATHER_OK_INTERVAL_MS);
    displayDirty = true;
}

/*
 * 画面刷新。
 *
 * 平时什么都不做, 只在两种情况下重画:
 *   a) 分钟变了  —— 时间要更新
 *   b) displayDirty —— 别的服务改了状态 (WiFi 掉了、天气更新了...)
 *
 * 屏幕刷新要走 I2C 推 1KB 数据, 是有成本的; 一分钟一次 +
 * 偶尔的状态变化, 这个频率刚刚好。
 */
// 注意它不用传 now 进来 —— 它自己从系统时间拿。
static void serviceDisplay() {
    static int lastMinute = -1;

    struct tm t;
    bool haveTime = getLocalTime(&t, 0);

    // 分钟变了 (或第一次拿到时间) 就要重画
    if (haveTime && t.tm_min != lastMinute) {
        lastMinute = t.tm_min;
        displayDirty = true;
    }

    if (!displayDirty) {
        return;
    }
    displayDirty = false;

    drawScreen(haveTime ? &t : nullptr);

    // 串口日志: 每次重画打一行, 方便调试
    if (haveTime && timeReady) {
        if (g_weather.valid) {
            Serial.printf("%02d:%02d  %d C  %s  湿度 %d%%%s\n", t.tm_hour,
                          t.tm_min, g_weather.temp, g_weather.text,
                          g_weather.humidity, wifiOk ? "" : "   [无网络]");
        } else {
            Serial.printf("%02d:%02d  (天气数据还没拿到)\n", t.tm_hour,
                          t.tm_min);
        }
    }
}

// ===============================================================

void setup() {
    Serial.begin(115200);
    delay(1500);

    Serial.println();
    Serial.println("================================");
    Serial.println("  Step 9: 自动重连 + 错误上屏");
    Serial.println("================================");

    Wire.begin(PIN_SDA, PIN_SCL);
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("!! SSD1306 初始化失败");
        return;
    }
    showMessage("Booting...", nullptr);

    // ---- 这一步最重要的改动: setup 里不再死等任何东西 ----
    //
    // WiFi 只是"发起"连接, 连没连上交给 serviceWiFi 判断。
    // wifiNextMs = 0 表示"立刻就该做", 所以主循环第一圈就会发起连接。
    //
    // 好处: 就算开机时路由器还没开, setup 也能正常跑完,
    //       设备会一直重试, 路由器一开就自己活过来。
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    wifiNextMs = 0;

    Serial.println("启动完成, 交给主循环。");
    Serial.println("(屏幕会自己从 --:-- 变成正确时间, 不用管)");
}

void loop() {
    uint32_t now = millis();

    // 四件事各自判断"现在该不该做"。都立刻返回, 所以四者互不阻塞。
    serviceWiFi(now);
    serviceNtp(now);
    serviceWeather(now);
    serviceDisplay();

    // 200ms 转一圈。这个数字决定了状态变化的响应速度 ——
    // WiFi 掉线到屏幕显示 NO WIFI 最多延迟 200ms, 人眼感觉是即时的。
    //
    // 注意这里**不能**改成 delay(60000) 之类的长间隔:
    // 那样这四件事就只能一分钟做一次, 分钟跳变会延迟, 重连也会变慢。
    delay(LOOP_INTERVAL_MS);
}
