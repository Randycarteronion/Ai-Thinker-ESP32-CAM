#include "esp_camera.h"
//#include <esp_now.h> //ESP-NOW 不能与普通 WiFi STA 模式（连接路由器）同时使用，只能在 WiFi AP 模式或 WiFi 关闭 的情况下稳定工作。
#include <WiFi.h>
#include <stdio.h>
//降低主时钟频率为10 
//需要增加一个远程复位功能 掉线提示 需要维持视频图传
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//
//            You must select partition scheme from the board menu that has at least 3MB APP space.
//            Face Recognition is DISABLED for ESP32 and ESP32-S2, because it takes up from 15
//            seconds to process single frame. Face Detection is ENABLED if PSRAM is enabled as well
//警告!!! UXGA 分辨率和高质量 JPEG 需要 PSRAM IC  
//            请确保选择了 ESP32 Wrover 模块或其他带有 PSRAM 的开发板  
//            如果图像超过缓冲区大小，将仅传输部分图像  
//
//            你必须在开发板菜单中选择 **至少** 具有 3MB APP 空间的分区方案。  
//            由于单帧处理时间长达 15 秒，**ESP32 和 ESP32-S2 上已禁用人脸识别**。  
//            如果启用了 PSRAM，则人脸检测功能可用。  

// ===================
// Select camera model
// ===================
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE  // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE
#define CAMERA_MODEL_AI_THINKER  // Has PSRAM //具有 PSRAM（外部 SRAM） camera_pins.h 会根据 CAMERA_MODEL_AI_THINKER 来选择合适的 GPIO 配置 该模块正在使用 AI-Thinker 版本的 ESP32 摄像头模块，并且该模块带有 PSRAM（外部静态随机存取内存）
//#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_CAMS3_UNIT  // Has PSRAM
//#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
//#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
// ** Espressif Internal Boards **
//#define CAMERA_MODEL_ESP32_CAM_BOARD
//#define CAMERA_MODEL_ESP32S2_CAM_BOARD
//#define CAMERA_MODEL_ESP32S3_CAM_LCD
//#define CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3 // Has PSRAM
//#define CAMERA_MODEL_DFRobot_Romeo_ESP32S3 // Has PSRAM
#include "camera_pins.h" //引脚和硬件配置定义

// ===========================
// Enter your WiFi credentials
// ===========================
const char *ssid = "震惊长安第一拳";
const char *password = "18353611063";

void startCameraServer();//启动摄像头服务器
void setupLedFlash(int pin); //闪光灯控制 控制引脚为pin

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);//设置串口通信的参数并且输出一行空白的调试信息 设置串口通信波特率为 115200，开启与计算机的串口通信 启用调试输出使后续的串口输出（Serial.print()）用于调试
  Serial.println();

  camera_config_t config;//引脚配置
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  //config.frame_size = FRAMESIZE_UXGA; //uxga-->vga降低分辨率
  config.frame_size = FRAMESIZE_VGA; 
  config.pixel_format = PIXFORMAT_JPEG;  // 用于流式传输
  // 配置像素格式为 PIXFORMAT_RGB565；用于人脸检测/识别
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  //config.jpeg_quality = 12；
  config.jpeg_quality = 8;  // 提高图像质量（0-63，数值越小质量越高） 
  //config.fb_count = 1;
  config.fb_count = 2;  // 增加帧缓冲，提高帧率

 // 如果存在 PSRAM 集成电路，则以 UXGA 分辨率和更高的 JPEG 质量进行初始化
 // 以便为更大的预分配帧缓冲区提供支持。
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;//检查像素格式-->PSRAM可用config.jpeg_quality = 10：设置JPEG图像的质量为10config.fb_count = 2：设置帧缓存的数量为2config.grab_mode = CAMERA_GRAB_LATEST：设置抓取模式为“最新如果没有PSRAM可用，则将帧大小限制为SVGA（800x600），以减少内存需求。config.fb_location = CAMERA_FB_IN_DRAM：将帧缓存位置设置为DRAM（动态随机存取存储器）根据PSRAM的可用性来调整相机配置，以确保在不同的硬件资源情况下，图像捕获能平稳运行。
    } else {
      //当psram不可用限制帧大小
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // 人脸检测最佳选项
    config.frame_size = FRAMESIZE_240X240;//帧大小设置为240x240
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;//为ESP32S3设置双缓冲，增强图像抓取和处理的效率。
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);//将GPIO13配置为输入并启用内部上拉电阻用于连接按键或者外部设备，设置为上拉输入可以确保引脚在没有信号输入时保持为高电平
  pinMode(14, INPUT_PULLUP);//同上
#endif

  // 相机初始化
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);//如果返回的错误码不是 ESP_OK（即不为零），说明初始化失败 如果初始化失败，使用 Serial.printf 输出错误信息
    return;
  }

  sensor_t *s = esp_camera_sensor_get();//获取当前相机模块的传感器对象，返回一个指向 sensor_t 结构体的指针
  // initial sensors are flipped vertically and colors are a bit saturated 修复反转和饱和
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // 开启垂直翻转
    s->set_brightness(s, 1);   // 适当增加亮度
    s->set_saturation(s, -2);  // 降低饱和度
  }
  // 下拉帧大小以获得更高的初始帧速率
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)//M5Stack 相关的 ESP32 相机模块 在硬件安装时默认方向不同，导致图像可能是颠倒或反向的。因此，需要软件调整来修正图像方向，使其符合正常的预期
  s->set_vflip(s, 1);//上下翻转
  s->set_hmirror(s, 1);//水平翻转
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

//如果在camera_pins.h中定义了LED引脚，则设置LED FLash
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

  WiFi.begin(ssid, password); 
  WiFi.setSleep(false);//关闭休眠提速
  WiFi.setTxPower(WIFI_POWER_19_5dBm);  // 提高信号强度


  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void loop() {
  //延迟循环 所有工作都由web服务器在另一个任务中完成
  delay(10000);
}




/*
#include "esp_camera.h"
#include <WiFi.h>
#include <stdio.h>
#include "camera_pins.h"

const char *ssid = "震惊长安第一拳";
const char *password = "18353611063";

void startCameraServer();
void setupLedFlash(int pin);

void setup() {
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    Serial.println();

    // 配置摄像头参数
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.frame_size = FRAMESIZE_UXGA;
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 12;
    config.fb_count = 1;

    if (psramFound()) {
        config.jpeg_quality = 10;
        config.fb_count = 2;
        config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
        config.frame_size = FRAMESIZE_SVGA;
        config.fb_location = CAMERA_FB_IN_DRAM;
    }

    // 初始化摄像头
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);
        s->set_brightness(s, 1);
        s->set_saturation(s, -2);
    }
    if (config.pixel_format == PIXFORMAT_JPEG) {
        s->set_framesize(s, FRAMESIZE_QVGA);
    }

    // 连接 WiFi
    WiFi.begin(ssid, password);
    WiFi.setSleep(false);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected!");
    Serial.print("ESP32 IP Address: ");
    Serial.println(WiFi.localIP());

    // 启动摄像头服务器
    startCameraServer();
    Serial.println("Camera server ready");
}

void loop() {
    delay(10000);
}
*/
/*
#include "esp_camera.h"
#include <WiFi.h>

// ===================
// 摄像头型号选择
// ===================
#define CAMERA_MODEL_AI_THINKER  // 使用AI Thinker开发板（带PSRAM）
#include "camera_pins.h"         // 引脚配置文件

// ========================
// 修改为你的WiFi账号密码
// ========================
const char* ssid = "震惊长安第一拳_Game";
const char* password = "18353611063";

// ====================
// 摄像头初始化函数
// ====================
bool initCamera() {
  camera_config_t config;

  // 摄像头硬件配置
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  
  // 图像参数配置
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;  // 视频流使用JPEG格式
  
  // PSRAM自适应配置
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;   // 高分辨率模式
    config.jpeg_quality = 10;             // 图像质量（10-63，数值越小质量越高）
    config.fb_count = 2;                  // 双缓冲
    config.grab_mode = CAMERA_GRAB_LATEST;// 获取最新帧
    config.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    config.frame_size = FRAMESIZE_SVGA;   // 受限分辨率
    config.jpeg_quality = 12;
    config.fb_count = 1;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  // 初始化摄像头
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("摄像头初始化失败 错误码: 0x%x", err);
    return false;
  }

  // 摄像头参数微调
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor->id.PID == OV3660_PID) {
    sensor->set_vflip(sensor, 1);   // 垂直翻转
    sensor->set_brightness(sensor, 1);
    sensor->set_saturation(sensor, -2);
  }
  
  // 初始降低分辨率提升帧率
  sensor->set_framesize(sensor, FRAMESIZE_QVGA);  // 320x240
  
  return true;
}

// ====================
// 主要程序
// ====================
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);  // 启用详细日志
  Serial.println("\n系统启动...");

  // 第1阶段：摄像头初始化
  Serial.println("[1/3] 初始化摄像头");
  if(!initCamera()){
    Serial.println("致命错误：摄像头初始化失败，系统停止！");
    while(1) delay(100);  // 死循环阻止继续执行
  }

  // 第2阶段：WiFi连接
  Serial.printf("\n[2/3] 连接WiFi: %s", ssid);
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);  // 禁用WiFi睡眠模式
  
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    
    // 30秒连接超时检测
    if(millis() - startTime > 30000){
      Serial.println("\n错误：WiFi连接超时！");
      ESP.restart();  // 自动重启
    }
  }
  Serial.printf("\nWiFi已连接\nIP地址: %s\n", WiFi.localIP().toString().c_str());

  // 第3阶段：启动网络服务
  Serial.println("[3/3] 启动视频流服务");
  startCameraServer();  // 启动默认HTTP服务
  
  Serial.println("系统准备就绪");
  Serial.println("========================");
  Serial.println("访问地址:");
  Serial.print("http://");
  Serial.println(WiFi.localIP());
  Serial.println("========================");
}

void loop() {
  delay(10000);  // 主循环无需操作
  
  // 可选：添加心跳指示
  static uint32_t lastTick = 0;
  if(millis() - lastTick > 10000){
    Serial.printf("[系统状态] 运行时间: %.1f 分钟\n", millis()/60000.0);
    lastTick = millis();
  }
}
*/