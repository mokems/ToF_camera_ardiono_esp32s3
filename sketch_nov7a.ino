#include <sstream>
#include <Arduino.h>
// The i2c library
#include <Wire.h>
#include "esp_camera.h"
#include <Base64.h>  // 导入 Base64 编码库
#include <WiFi.h>
#include <WiFiUdp.h>

#include <vl53l7cx_class.h>
#include "MLX90641_API.h"
#include "MLX90640_API.h"
#include "MLX9064X_I2C_Driver.h"
#define VL53L7CX_NB_TARGET_PER_ZONE 3U
#include "platform_config_default.h"



#define CHUNK_LENGTH 1460/2

#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5
#define Y9_GPIO_NUM    16
#define Y8_GPIO_NUM    17
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    12
#define Y5_GPIO_NUM    10
#define Y4_GPIO_NUM    8
#define Y3_GPIO_NUM    9
#define Y2_GPIO_NUM    11
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13

// const char* ssid = "AIOT";
// const char* password = "12345678";
// const int udpPort = 8000;
// const char* udpAddress = "192.168.3.41";

char ssid[32]; // 存储 WiFi SSID
char password[32]; // 存储 WiFi 密码
char udpAddress[32]; // 存储 UDP 目标地址
int udpPort; // 存储 UDP 端口


boolean connected = false;
WiFiUDP udp;



bool startSignalReceived = false;
// ========== Configuration of the ToF sensor ==========

#ifdef ARDUINO_SAM_DUE
  #define DEV_I2C Wire1
#else
  #define DEV_I2C Wire
#endif
#define SerialPort Serial

#define LPN_PIN 45
// #define I2C_RST_PIN A1
// #define PWREN_PIN A5


// SparkFun_VL53L5CX VL53L5CX;
// VL53L5CX_ResultsData tofData;
#define numPixels   64   
#define numZones    8*8 // 4x4 zones  = 16 zones
#define numTargets 3    // the number of depth/targets per zone; 4 is the max; need to set the VL53L5CX_NB_TARGET_PER_ZONE to 4 in platform.h

// ========== Configuration of the Thermal sensor ==========
// configuration for MLX sensor
int MLX_type = 0; // 0: MLX90640; 1: MLX90641
float emissivity = 1.0; // the emissivity setting of sensor
int ambient_temperature_shift = 8; // 5:mlx90641; 8:mlx90640

uint8_t REFRESH_0_5_HZ = 0b000;  // 0.5Hz
uint8_t  REFRESH_1_HZ = 0b001;  // 1Hz
uint8_t REFRESH_2_HZ = 0b010;   // 2Hz
uint8_t REFRESH_4_HZ = 0b011;   // 4Hz
uint8_t REFRESH_8_HZ = 0b100;   // 8Hz
uint8_t REFRESH_16_HZ = 0b101;  // 16Hz
uint8_t REFRESH_32_HZ = 0b110;  // 32Hz
uint8_t REFRESH_64_HZ = 0b111;  // 64Hz
// default refreshrate
uint8_t refreshrate_u8 = REFRESH_16_HZ; // May be modified based on the above 'refreshrate' configuration

const byte MLX9064x_address = 0x33; //Default 7-bit unshifted address of the MLX9064x // the default I2C address of the sensor
#define TA_SHIFT ambient_temperature_shift //Default shift for MLX9064x in open air
// for mlx90641:
static float mlx90641To[192];
paramsMLX90641 mlx90641;
// for mlx90640:
static float mlx90640To[768];
paramsMLX90640 mlx90640;


// ========== Helper functions ==========

std::string int_array_to_string(int int_array[], int size_of_array) {
    std::ostringstream oss;
    oss << "[";
    for (int i = 0; i < size_of_array; i++) {
        oss << int_array[i];
        oss << ",";
    }
    oss << "]";
    return oss.str();
}

std::string float_array_to_string(float float_array[], int size_of_array) {
    std::ostringstream oss;
    oss << "[";
    for (int i = 0; i < size_of_array; i++) {
        oss << float_array[i];
        oss << ",";
    }
    oss << "]";
    return oss.str();
}

std::string int16_array_to_string(int16_t int_array[], int size_of_array) {
    std::ostringstream oss;
    oss << "[";
    for (int i = 0; i < size_of_array; i++) {
        oss << int_array[i];
        oss << ",";
    }
    oss << "]";
    return oss.str();
}

std::string int8_array_to_string(uint8_t int_array[], int size_of_array) {
    std::ostringstream oss;
    oss << "[";
    for (int i = 0; i < size_of_array; i++) {
        oss << (unsigned int) int_array[i]; // Explicit cast as uint8_t is often treated as char
        oss << ",";
    }
    oss << "]";
    return oss.str();
}

std::string uint16_array_to_string(uint16_t int_array[], int size_of_array) {
    std::ostringstream oss;
    oss << "[";
    for (int i = 0; i < size_of_array; i++) {
        oss << int_array[i];
        oss << ",";
    }
    oss << "]";
    return oss.str();
}

std::string uint32_array_to_string(uint32_t int_array[], int size_of_array) {
    std::ostringstream oss;
    oss << "[";
    for (int i = 0; i < size_of_array; i++) {
        oss << int_array[i];
        oss << ",";
    }
    oss << "]";
    return oss.str();
}
// MLX sensor
boolean isConnected() {
    Wire.beginTransmission((uint8_t)MLX9064x_address);
    if (Wire.endTransmission() != 0) {
        return (false);    //Sensor did not ACK
    }
    return (true); //Returns true if the MLX9064x is detected on the I2C bus
}

// ========== Global variables ==========

int num_samples = 0;
int start_time = 0;
// for mlx9064x:
float ta;  // Ambient temperature
float Ta;  // Sensor ambient temperature
float tr;  // Reflected temperature based on the sensor ambient temperature
int reading_status = 0; // MLX9064x reading status

bool MLX_reading_flag = false;
bool VL53_reading_flag = false;


void print_result(VL53L7CX_ResultsData *Result,std::string &output);
// void clear_screen(void);
// void handle_cmd(uint8_t cmd);
// void display_commands_banner(void);

// Components.
VL53L7CX sensor_vl53l7cx_top(&DEV_I2C, LPN_PIN);

bool EnableAmbient = false;
bool EnableSignal = false;
uint8_t res = VL53L7CX_RESOLUTION_8X8;











// ========== Main code ==========
void setup() {
  Serial.begin(921600);
  // Serial.begin(115200);  // 使用较低的波特率

  camera_config_t config;
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
  config.ledc_timer = LEDC_TIMER_0;
  config.ledc_channel   = LEDC_CHANNEL_0;

  config.jpeg_quality = 20;
  config.fb_count = 3;    
  config.xclk_freq_hz = 24000000;
  config.pixel_format = PIXFORMAT_JPEG; //YUV422,GRAYSCALE,RGB565,JPEG
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.frame_size = FRAMESIZE_VGA;
  
  if (psramFound()) {
      Serial.println("PSRAM FOUND"); // when run code alwayse PSRAM FOUND.
    }

    // Initialize the camera
    if (esp_camera_init(&config) != ESP_OK) {
        Serial.println("Failed to initialize camera.");
        return;
    }
  // Serial.println("PSRAM initialization failed");
  // if (err != ESP_OK) {
  //     Serial.printf("Camera init failed with error 0x%x", err);
  //     return;
  // }
  // connectToWiFi(ssid, password);
  
  
  // 尝试连接WiFi
  while (!(strlen(ssid) > 0 && strlen(password) > 0)) {
    getWiFiAndUdpConfig();
    connectToWiFi(ssid, password);
  }
  delay(100);


  Serial.setRxBufferSize(1024);
  Wire.begin(4,5);
  Wire.setClock(1000000);
  // config tof
  sensor_vl53l7cx_top.begin();
  sensor_vl53l7cx_top.init_sensor();
  sensor_vl53l7cx_top.vl53l7cx_set_resolution(VL53L7CX_RESOLUTION_8X8);
  uint8_t resolution = 15U;
  sensor_vl53l7cx_top.vl53l7cx_set_ranging_frequency_hz(resolution);
  sensor_vl53l7cx_top.vl53l7cx_set_detection_thresholds_enable(0U);
  VL53L7CX_DetectionThresholds thresholds[VL53L7CX_NB_THRESHOLDS];
  memset(&thresholds, 0, sizeof(thresholds));
  uint8_t i;
  for (i = 0; i < res; i++)
  {
    thresholds[i].zone_num = i;
    thresholds[i].measurement = VL53L7CX_DISTANCE_MM;
    thresholds[i].type = VL53L7CX_IN_WINDOW;
    thresholds[i].mathematic_operation = VL53L7CX_OPERATION_NONE;
    thresholds[i].param_low_thresh = 20;
    thresholds[i].param_high_thresh = 600;
  }
  // Send array of thresholds to the sensor.
    sensor_vl53l7cx_top.vl53l7cx_set_detection_thresholds(thresholds);

    // Enable thresholds detection.
    sensor_vl53l7cx_top.vl53l7cx_set_detection_thresholds_enable(1U);


  thresholds[i].zone_num |= VL53L7CX_LAST_THRESHOLD;

  // Start Measurements
  sensor_vl53l7cx_top.vl53l7cx_start_ranging();
 
  start_time = millis();

}

// void loop(){

// }
uint16_t frame_id = 0;
// uint16_t mlx90640Frame[834];
uint8_t NewDataReady = 0;
uint8_t status;
VL53L7CX_ResultsData Results;

void loop() {
    static unsigned long previousMillisCamera = 0;
    std:: string output = "";
    unsigned long startTime = millis();

    

    NewDataReady = 0;
    status = sensor_vl53l7cx_top.vl53l7cx_check_data_ready(&NewDataReady);
    if ((!status) && (NewDataReady != 0)) {
      status = sensor_vl53l7cx_top.vl53l7cx_get_ranging_data(&Results);
      // print_result(&Results);
      // print_result(&Results, output);
      VL53_reading_flag = true;

    }
    // unsigned long endTime = millis();
    // unsigned long timeDifference = endTime - startTime; 
    // Serial.print("代码块2执行时间: ");
    // Serial.print(timeDifference);
    // Serial.println(" 毫秒");
    // startTime = millis();
    if (connected) {
      camera_fb_t* fb = NULL;
      esp_err_t res = ESP_OK;
      fb = esp_camera_fb_get();
      if (!fb) {
        Serial.println("Camera capture failed");
        esp_camera_fb_return(fb);
        return;
      }

      if (fb->format != PIXFORMAT_JPEG) {
        Serial.println("PIXFORMAT_JPEG not implemented");
        esp_camera_fb_return(fb);
        return;
      }
      frame_id++;
      sendPacketData((const char*)fb->buf, fb->len, CHUNK_LENGTH, frame_id, &Results);
      if (frame_id>5000){
        frame_id = 0;
      }
      // delay(100);
      // Serial.println("sended");
      esp_camera_fb_return(fb);
    }
    // endTime = millis();
    // timeDifference = endTime - startTime; 
    // Serial.print("代码块3执行时间: ");
    // Serial.print(timeDifference);
    // Serial.println(" 毫秒");
    
  // if (VL53_reading_flag == true && MLX_reading_flag == true) {
  //   output += "}";
  //   // Serial.println(output.c_str());
  //   num_samples++;
  //   VL53_reading_flag = false;
  //   MLX_reading_flag = false;
  // }
}



// 从串口读取WiFi和UDP配置信息
void getWiFiAndUdpConfig() {
    if (Serial.available() > 0) {
        // 读取并解析一行输入，假设每个参数之间用 ',' 分隔
        char inputBuffer[128];
        readSerialInput(inputBuffer, sizeof(inputBuffer));

        // 假设输入格式是: "ssid,password,udpAddress,udpPort"
        char* token = strtok(inputBuffer, ",");
        if (token != nullptr) {
            strncpy(ssid, token, sizeof(ssid) - 1);
            ssid[sizeof(ssid) - 1] = '\0';
        }

        token = strtok(nullptr, ",");
        if (token != nullptr) {
            strncpy(password, token, sizeof(password) - 1);
            password[sizeof(password) - 1] = '\0';
        }

        token = strtok(nullptr, ",");
        if (token != nullptr) {
            strncpy(udpAddress, token, sizeof(udpAddress) - 1);
            udpAddress[sizeof(udpAddress) - 1] = '\0';
        }

        token = strtok(nullptr, ",");
        if (token != nullptr) {
            udpPort = atoi(token);
        }

        Serial.println("Configuration received:");
        Serial.print("SSID: ");
        Serial.println(ssid);
        Serial.print("Password: ");
        Serial.println(password);
        Serial.print("UDP Address: ");
        Serial.println(udpAddress);
        Serial.print("UDP Port: ");
        Serial.println(udpPort);
    }
}

// 从串口读取输入
void readSerialInput(char* buffer, size_t bufferSize) {
    size_t index = 0;
    unsigned long startMillis = millis();
    while (true) {
        if (Serial.available() > 0) {
            char incomingByte = Serial.read();
            if (incomingByte == '\n' || incomingByte == '\r') {
                // 如果遇到换行符或回车符，表示输入结束
                buffer[index] = '\0'; // 添加字符串结束符
                break;
            } else {
                if (index < bufferSize - 1) {
                    buffer[index++] = incomingByte; // 存储输入字符
                }
            }
        }

        // // 超时控制，避免一直等待输入
        // if (millis() - startMillis > 5000) {
        //     buffer[0] = '\0'; // 如果超时未输入任何内容，则清空缓冲区
        //     break;
        // }
        Serial.println(" dengdai");

    }
}

void connectToWiFi(const char* ssid, const char* pwd) {
  Serial.println("Connecting to WiFi network: " + String(ssid));

  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);  // 确保 ESP32 处于正确的工作模式
  WiFi.begin(ssid, pwd);

  int retryCount = 0;
  while (WiFi.status() != WL_CONNECTED && retryCount < 20) {
    delay(500);
    Serial.print(".");
    retryCount++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    connected = true;
  } else {
    Serial.println("\nFailed to connect to WiFi");
  }
}


void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case IP_EVENT_STA_GOT_IP:
      Serial.print("WiFi connected! IP address: ");
      Serial.println(WiFi.localIP());
      udp.begin(WiFi.localIP(), udpPort);
      connected = true;
      break;
    case WIFI_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      connected = false;
      break;
  }
}






void sendPacketData(const char* buf, uint16_t len, uint16_t chunkLength, uint16_t frameId, VL53L7CX_ResultsData *Results) {
    uint8_t serializedResults[1024]; // 分配一个足够大的缓冲区存储序列化的数据
    size_t serializedLength = serializeResults(Results, serializedResults, sizeof(serializedResults));

    // 发送 Results 数据包，附加类型标识符 'R'
    uint8_t buffer[chunkLength + 5]; // 1字节数据类型 + 2字节帧序号 + 2字节包序号 + 数据
    size_t blen = chunkLength;
    size_t rest = serializedLength % blen;
    uint16_t packetCount = 0;

    for (size_t i = 0; i < serializedLength / blen; ++i) {
        packetCount++; // 包序号递增
        buffer[0] = 'R';                // 数据类型标识符，'R' 表示是 Results 数据
        buffer[1] = frameId & 0xFF;      // 帧序号低位
        buffer[2] = (frameId >> 8) & 0xFF; // 帧序号高位
        buffer[3] = packetCount & 0xFF;    // 包序号低位
        buffer[4] = (packetCount >> 8) & 0xFF; // 包序号高位

        memcpy(buffer + 5, serializedResults + (i * blen), blen);
        
        udp.beginPacket(udpAddress, udpPort);
        udp.write(buffer, blen + 5);
        udp.endPacket();
    }

    if (rest) {
        packetCount++;
        buffer[0] = 'R';                // 数据类型标识符，'R' 表示是 Results 数据
        buffer[1] = frameId & 0xFF;      // 帧序号低位
        buffer[2] = (frameId >> 8) & 0xFF; // 帧序号高位
        buffer[3] = packetCount & 0xFF;    // 包序号低位
        buffer[4] = (packetCount >> 8) & 0xFF; // 包序号高位

        memcpy(buffer + 5, serializedResults + (serializedLength - rest), rest);
        
        udp.beginPacket(udpAddress, udpPort);
        udp.write(buffer, rest + 5);
        udp.endPacket();
    }

    // 发送相机数据包，附加类型标识符 'C'
    packetCount = 0;  // 重置包序号
    rest = len % blen;

    for (size_t i = 0; i < len / blen; ++i) {
        packetCount++; // 包序号递增
        buffer[0] = 'C';                // 数据类型标识符，'C' 表示是相机数据
        buffer[1] = frameId & 0xFF;      // 帧序号低位
        buffer[2] = (frameId >> 8) & 0xFF; // 帧序号高位
        buffer[3] = packetCount & 0xFF;    // 包序号低位
        buffer[4] = (packetCount >> 8) & 0xFF; // 包序号高位

        memcpy(buffer + 5, buf + (i * blen), blen);
        
        udp.beginPacket(udpAddress, udpPort);
        udp.write(buffer, blen + 5);
        udp.endPacket();
    }

    if (rest) {
        packetCount++;
        buffer[0] = 'C';                // 数据类型标识符，'C' 表示是相机数据
        buffer[1] = frameId & 0xFF;      // 帧序号低位
        buffer[2] = (frameId >> 8) & 0xFF; // 帧序号高位
        buffer[3] = packetCount & 0xFF;    // 包序号低位
        buffer[4] = (packetCount >> 8) & 0xFF; // 包序号高位

        memcpy(buffer + 5, buf + (len - rest), rest);
        
        udp.beginPacket(udpAddress, udpPort);
        udp.write(buffer, rest + 5);
        udp.endPacket();
    }
    
}


size_t serializeResults(VL53L7CX_ResultsData *Results, uint8_t *buffer, size_t bufferSize) {
    size_t offset = 0;

    // 序列化 distance_mm
    for (int t = 0; t < 3; t++) {
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 8; j++) {
                if (offset + 2 >= bufferSize) {
                    return offset; // 防止缓冲区溢出
                }
                int index = t * 64 + i * 8 + j;
                uint16_t distance = Results->distance_mm[index];
                buffer[offset++] = (distance >> 8) & 0xFF;
                buffer[offset++] = distance & 0xFF;
            }
        }
    }

    // 序列化 reflectance
    for (int t = 0; t < 3; t++) {
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 8; j++) {
                if (offset + 1 >= bufferSize) {
                    return offset; // 防止缓冲区溢出
                }
                int index = t * 64 + i * 8 + j;
                buffer[offset++] = Results->reflectance[index];
            }
        }
    }

    // 序列化 target_status
    for (int t = 0; t < 3; t++) {
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 8; j++) {
                if (offset + 1 >= bufferSize) {
                    return offset; // 防止缓冲区溢出
                }
                int index = t * 64 + i * 8 + j;
                buffer[offset++] = Results->target_status[index];
            }
        }
    }

    return offset; // 返回序列化的总长度
}




