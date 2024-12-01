#include "secrets.h"
#include "esp_camera.h"
#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "soc/soc.h" //disable brownout problems
#include "soc/rtc_cntl_reg.h" //disable brownout problems
#include "driver/gpio.h"
// #include <Servo.h> // For servo control

// configuration for AI Thinker Camera board
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Motor and servo pins
#define MOTOR_PIN 12 // Pin for motor control
#define SERVO_PIN 13 // Pin for servo control

const char* ssid     = WIFI_SSID; // from secrets.h
const char* password = WIFI_PASSWORD; // from secrets.h

const char* websockets_server_host = WS_SERVER_URL; // from secrets.h

camera_fb_t * fb = NULL;
size_t _jpg_buf_len = 0;
uint8_t * _jpg_buf = NULL;
uint8_t state = 0;
// Servo servo; // Servo object for controlling the servo

using namespace websockets;
WebsocketsClient client;

void onMessageCallback(WebsocketsMessage message) {
  String command = message.data();
  Serial.print("Got Message: ");
  Serial.println(command);

  if (command.startsWith("MOTOR:")) {
      // Control motor speed (0-255)
      int speed = command.substring(6).toInt();
      speed = constrain(speed, 0, 255);
      analogWrite(MOTOR_PIN, speed);
      Serial.printf("Motor speed set to %d\n", speed);
  } else if (command.startsWith("SERVO:")) {
      // Control servo angle (0-180)
      int angle = command.substring(6).toInt();
      angle = constrain(angle, 0, 180);
      // servo.write(angle);
      Serial.printf("Servo angle set to %d\n", angle);
  }
}

esp_err_t init_camera() {
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // parameters for image quality and size
  config.frame_size = FRAMESIZE_QVGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
  config.jpeg_quality = 15; //10-63 lower number means higher quality
  config.fb_count = 1;

  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("camera init FAIL: 0x%x", err);
    return err;
  }
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
  Serial.println("camera init OK");
  return ESP_OK;
};


esp_err_t init_wifi() {
  WiFi.begin(ssid, password);
  Serial.println("Wifi init ");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi OK");
  Serial.println("connecting to WS: ");
  client.onMessage(onMessageCallback);
  bool connected = client.connect(websockets_server_host);
  if (!connected) {
    Serial.println("WS connect failed!");
    Serial.println(WiFi.localIP());
    return ESP_FAIL;
  }

  Serial.println("WS OK");
  // client.send("hello from ESP32 camera stream!");
  return ESP_OK;
};

// Servo setup
esp_err_t init_servo() {
  // servo.attach(SERVO_PIN);   // Attach servo
  // servo.write(90);           // Set servo to mid-position
  return ESP_OK;
};

// Motor setup
esp_err_t init_motor() {
  pinMode(MOTOR_PIN, OUTPUT);
  analogWrite(MOTOR_PIN, 0); // Start motor at 0 speed
  return ESP_OK;
};

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  Serial.setDebugOutput(true);

  init_camera();
  init_wifi();
  // init_servo();
  // init_motor();
}

void loop() {
  if (client.available()) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("img capture failed");
      esp_camera_fb_return(fb);
      ESP.restart();
    }
    client.sendBinary((const char*) fb->buf, fb->len);
    Serial.println("image sent");
    esp_camera_fb_return(fb);
    client.poll();
    delay(50);
  }
}
