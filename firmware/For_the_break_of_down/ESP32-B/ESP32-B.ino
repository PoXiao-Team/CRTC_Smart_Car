#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <Esp.h> // [补充] 确保 ESP.restart() 可编译

// 从 ESP32 视角命名引脚，接 STM32 时务必交叉接线
#define ESP32_RX_PIN 18 // ESP32 接收端，接 STM32 的 TX
#define ESP32_TX_PIN 17 // ESP32 发送端，接 STM32 的 RX

// ================= 2. 全局变量与对象 =================
rcl_subscription_t subscriber;
std_msgs__msg__Int32MultiArray target_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

HardwareSerial SerialToSTM(2); 

// ================= [补充] 3. 函数前置声明（防止编译报错）=================
void sendSerialToSTM(int16_t x, int16_t y, int16_t angle);

// ================= 4. 回调函数与串口协议 =================
void target_callback(const void * msgin) {
  const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)msgin;
  
  if (msg->data.size >= 3) {
    int32_t x32 = msg->data.data[0];
    int32_t y32 = msg->data.data[1];
    int32_t a32 = msg->data.data[2];
    
    // [补充] 坐标范围钳位，防止 int32 转 int16 时溢出导致负数反转
    x32 = constrain(x32, -32768, 32767);
    y32 = constrain(y32, -32768, 32767);
    a32 = constrain(a32, -32768, 32767);
    
    sendSerialToSTM((int16_t)x32, (int16_t)y32, (int16_t)a32);
  }
}

void sendSerialToSTM(int16_t x, int16_t y, int16_t angle) {
  uint8_t packet[10];
  packet[0] = 0xAA;
  packet[1] = 0xBB;
  packet[2] = (x >> 8) & 0xFF; packet[3] = x & 0xFF;
  packet[4] = (y >> 8) & 0xFF; packet[5] = y & 0xFF;
  packet[6] = (angle >> 8) & 0xFF; packet[7] = angle & 0xFF;
  
  uint8_t checksum = 0;
  for (int i = 0; i < 8; i++) checksum += packet[i];
  packet[8] = checksum;
  packet[9] = 0xCC;
  
  SerialToSTM.write(packet, 10);
}

// ================= 5. 初始化 setup() =================
void setup() {
  Serial.begin(115200);
  // 注意：参数顺序为 (波特率, 配置, RX, TX)
  SerialToSTM.begin(115200, SERIAL_8N1, ESP32_RX_PIN, ESP32_TX_PIN);
  IPAddress local_IP(10, 168, 121, 101);
  IPAddress gateway(10, 168, 121, 114); // 你的默认网关
  IPAddress subnet(255, 255, 255, 0);
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure static IP");
  }

  // 配置 micro-ROS WiFi 传输层
  // AGENT_IP 是虚拟机的 IP
  set_microros_wifi_transports("beltpoint", "15514933656", "10.168.121.35", 8888);
  
  // WiFi 连接超时保护
  Serial.print("Connecting WiFi...");
  unsigned long start_time = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - start_time > 10000) { // 10秒连不上就重启
      Serial.println("\nWiFi connect timeout! Restarting...");
      ESP.restart();
    }
  }
  Serial.println("\nWiFi Connected!");

  // [补充] 检查 MicroROS Agent 是否在线，增加 30 秒超时保护
  Serial.print("Waiting for MicroROS Agent...");
  unsigned long agent_start = millis();
  while (rmw_uros_ping_agent(1000, 3) != RMW_RET_OK) {
    Serial.print(".");
    if (millis() - agent_start > 30000) { // 30秒连不上 Agent 直接重启
      Serial.println("\nAgent connect timeout! Restarting...");
      ESP.restart();
    }
  }
  Serial.println("\nAgent Connected!");

  // [补充] 必须为 MicroROS 消息预分配内存
  target_msg.data.data = (int32_t*)malloc(3 * sizeof(int32_t)); 
  if (target_msg.data.data == NULL) {
    Serial.println("malloc failed! Restarting...");
    ESP.restart();
  }
  target_msg.data.size = 0;
  target_msg.data.capacity = 3;

  // 初始化 MicroROS 节点和订阅者
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_control_node", "", &support);

  rclc_subscription_init_default(
      &subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
      "/target_coord");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &target_msg, &target_callback, ON_NEW_DATA);
}

// ================= 6. 主循环 loop() =================
void loop() {
  // [优化] 单次循环约 11ms，能跟上 50Hz 左右的发布频率
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  delay(1);
}
