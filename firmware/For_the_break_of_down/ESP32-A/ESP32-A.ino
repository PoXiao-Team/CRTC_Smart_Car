// ================= 头文件 =================
#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>
// ================= ===== [改动] 全局配置区，拿到板子只改这里 ===== =================
// --- 1. 网络配置 ---
const char* ssid     = "beltpoint";      // 手机热点名称
const char* password = "15514933656";      // 手机热点密码
const char* host_ip  = "10.168.121.64";       // 电脑的 IP
const int   udp_port = 8888;                   // 电脑监听的端口
// --- 2. 图像参数 ---
const framesize_t   FRAME_SIZE_SETTING = FRAMESIZE_HD; // 分辨率：HD 是 1280x720
const int   JPEG_QUALITY       = 12;           // 画质：0-63，越小越清晰，12 兼顾清晰度和体积
const int   MAX_PAYLOAD        = 1400;         // 单包最大数据，勿改
const int   HEADER_SIZE        = 8;            // 协议包头长度，勿改
// --- 3. 性能与调试参数 ---
const int   TARGET_FPS         = 15;           // 目标帧率，太高会导致 WiFi 丢包
const int   LOG_PRINT_INTERVAL = 10;           // 每多少帧打印一次串口日志
// ================= ===== [改动] 引脚定义（避开了 PSRAM 冲突的 S3 引脚） ===== =================
#define PWDN_GPIO_NUM     -1 // 悬空
#define RESET_GPIO_NUM    -1 // 悬空
#define XCLK_GPIO_NUM     -1
#define SIOD_GPIO_NUM      4 // I2C SDA
#define SIOC_GPIO_NUM      5 // I2C SCL
#define Y9_GPIO_NUM       16
#define Y8_GPIO_NUM       17
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       12
#define Y5_GPIO_NUM       10
#define Y4_GPIO_NUM        8
#define Y3_GPIO_NUM        9
#define Y2_GPIO_NUM       11
#define VSYNC_GPIO_NUM     6
#define HREF_GPIO_NUM      7
#define PCLK_GPIO_NUM     13
WiFiUDP udp;
uint16_t frame_counter = 0;
unsigned long last_frame_time = 0;
// ================= 大端写入辅助 =================
inline void write_u16_be(uint8_t* buf, uint16_t v) {
  buf[0] = (uint8_t)((v >> 8) & 0xFF);
  buf[1] = (uint8_t)(v & 0xFF);
}
// ================= 初始化 =================
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n--- CRTC2026 Vision System Booting ---");
  // ---------- 1. 摄像头初始化 ----------
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
#else
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
#endif
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 12000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAME_SIZE_SETTING;  // [改动] 使用全局变量
  config.jpeg_quality = JPEG_QUALITY;        // [改动] 使用全局变量
  config.fb_count     = 2;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_LATEST;  // 丢弃旧帧，获取最新画面
  // 在 esp_camera_init(&config) 之前加上：
sensor_t * s = esp_camera_sensor_get();
if (s != NULL) {
    s->set_whitebal(s, 1);       // 开启自动白平衡
    s->set_awb_gain(s, 1);       // 开启自动白平衡增益
    s->set_exposure_ctrl(s, 1);  // 开启自动曝光
    s->set_aec2(s, 1);           // 开启 DSP 自动曝光
    // 下面几行是防偏色的关键
    s->set_saturation(s, 0);     // 饱和度归零，防止颜色过艳
    s->set_contrast(s, 1);       // 稍微提高一点对比度，让边缘更清晰
}
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    while (true) delay(1000);
  }
  Serial.println("Camera init OK");
  IPAddress local_IP(10, 168, 121, 100);
  IPAddress gateway(10, 168, 121, 114); // 你的默认网关
  IPAddress subnet(255, 255, 255, 0);
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure static IP");
  }
  // ---------- 2. WiFi 连接 ----------
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected! IP: ");
  Serial.println(WiFi.localIP());
  udp.begin(udp_port);
  Serial.println("UDP ready. Target IP: ");
  Serial.println(host_ip);
  last_frame_time = millis();
}
// ================= 主循环 =================
void loop() {
  // ===== [改动] 1. WiFi 断线自动重连机制 =====
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected! Reconnecting...");
    WiFi.reconnect();
    delay(500);
    return; // 跳过本次，等待重连
  }
  // ===== [改动] 2. 帧率控制，防止 WiFi 拥塞 =====
  unsigned long current_time = millis();
  int frame_interval_ms = 1000 / TARGET_FPS;
  if (current_time - last_frame_time < frame_interval_ms) {
    return; // 还没到下一帧的时间，直接跳过
  }
  last_frame_time = current_time;
  // 3. 抓取一帧
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    delay(50);
    return;
  }
  // 4. 计算分片数
  int total_chunks = (fb->len + MAX_PAYLOAD - 1) / MAX_PAYLOAD;
  // 5. 依次发送每个分片
  static uint8_t buffer[HEADER_SIZE + MAX_PAYLOAD];
  for (int i = 0; i < total_chunks; i++) {
    int offset = i * MAX_PAYLOAD;
    int remaining = (int)fb->len - offset;
    uint16_t payload_len = (remaining > MAX_PAYLOAD) ? MAX_PAYLOAD : (uint16_t)remaining;
    // 填充 8 字节头（大端）
    write_u16_be(buffer + 0, frame_counter);
    write_u16_be(buffer + 2, (uint16_t)total_chunks);
    write_u16_be(buffer + 4, (uint16_t)i);
    write_u16_be(buffer + 6, payload_len);
    // 拷贝数据
    memcpy(buffer + HEADER_SIZE, fb->buf + offset, payload_len);
    udp.beginPacket(host_ip, udp_port);
    udp.write(buffer, HEADER_SIZE + payload_len);
    udp.endPacket();
    // ===== [改动] 3. 发送延迟优化 =====
    // 原来的 delay(1) 每发一个包就休眠 1ms，70个包就是 70ms，严重拖慢帧率。
    // 现在改成每发 10 个包才让出 1ms 的 CPU 时间，既防止缓冲区溢出，又能保持高帧率。
    if (i % 10 == 0) {
      delay(1);
    }
  }
  // ===== [改动] 4. 串口日志节流 =====
  // 每 LOG_PRINT_INTERVAL 帧才打印一次，避免串口打印阻塞网络发送
  if (frame_counter % LOG_PRINT_INTERVAL == 0) {
    Serial.printf("Frame %u | %u bytes | %d chunks | FPS: %d\n",
                  frame_counter, (unsigned)fb->len, total_chunks, TARGET_FPS);
  }
  // 6. 释放 + 帧号递增
  esp_camera_fb_return(fb);
  frame_counter++;
}

