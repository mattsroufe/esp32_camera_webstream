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
#include <ESP32Servo.h>  // ESP32Servo library for servo control

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
// Motor and Servo setup
const int motorPin1 = 12;  // Motor control pin 1
const int motorPin2 = 13;  // Motor control pin 2
const int servoPin = 14;   // Servo control pin


// Define dead zone threshold (adjust as needed)
const int MOTOR_DEAD_ZONE = 10;  // Threshold for motor to ignore small values
const int SERVO_DEAD_ZONE = 10;  // Threshold for servo to ignore small values

// Smoothing factor (higher values make the smoothing slower)
const float MOTOR_SMOOTHING_FACTOR = 0.4;  // Smoothing factor for motor
const float SERVO_SMOOTHING_FACTOR = 0.6;  // Smoothing factor for servo

// Variables to store smoothed values
int smoothedMotorSpeed = 0;
int smoothedServoPos = 90;
int motorSpeed = 0;        // Motor speed (controlled by left joystick Y-axis)
int lastMotorSpeed = 0;    // To store last motor speed for smoothing
int lastServoPos = 90;     // To store last servo position for smoothing
int motorSpeedSmoothFactor = 5; // The factor to control smoothing speed
int servoPosSmoothFactor = 5;  // The factor to control servo smoothing speed

int deadZone = 50;  // Configurable dead zone for joystick input

const int RIGHT_STEERING_OFFSET = 50;
const int LEFT_STEERING_OFFSET = 20;

using namespace websockets;
WebsocketsClient client;

// Create an instance of the ESP32Servo class
Servo myServo;

void controlMotor(int throttle) {
    // Apply dead zone to the throttle value
    if (abs(throttle) < MOTOR_DEAD_ZONE) {
        throttle = 0;  // Ignore small values within dead zone
    }

    // Smooth the motor speed
    smoothedMotorSpeed = smoothedMotorSpeed + MOTOR_SMOOTHING_FACTOR * (throttle - smoothedMotorSpeed);

    Serial.print(" - Throttle: ");
    Serial.println(smoothedMotorSpeed);

    // Motor control using two pins for direction
    if (smoothedMotorSpeed > 0) {
        // Forward direction
        analogWrite(motorPin1, smoothedMotorSpeed);
        analogWrite(motorPin2, 0);
    } else if (smoothedMotorSpeed < 0) {
        // Reverse direction
        analogWrite(motorPin1, 0);
        analogWrite(motorPin2, -smoothedMotorSpeed);
    } else {
        // Stop motor
        analogWrite(motorPin1, 0);
        analogWrite(motorPin2, 0);
    }
}

// Function to control the servo based on the joystick X-axis
void controlServo(int servoPos) {
    // Apply dead zone to the servo position (ignore small values near neutral)
    if (abs(servoPos - 90) < SERVO_DEAD_ZONE) {
        servoPos = 90;  // Ignore small values near the center (neutral position)
    }

    // Smooth the servo position
    smoothedServoPos = smoothedServoPos + SERVO_SMOOTHING_FACTOR * (servoPos - smoothedServoPos);

    // Ensure the servo position converges to 90 when near it
    if (abs(smoothedServoPos - 90) < 5) {
        smoothedServoPos = 90;  // Force it to return exactly to 90 if close enough
    }

    Serial.print(" - Servo position: ");
    Serial.println(smoothedServoPos);

    // Write the smoothed position to the servo
    myServo.write(constrain(smoothedServoPos, 0 + LEFT_STEERING_OFFSET, 180 - RIGHT_STEERING_OFFSET));  // Constrain within car's steering range
}

void onMessageCallback(WebsocketsMessage message) {
  String command = message.data();
  // Serial.print("Got Message: ");
  // Serial.println(command);

  if (command.startsWith("MOTOR:")) {
      // Control motor speed (0-255)
      int speed = command.substring(6).toInt();
      speed = constrain(speed, -255, 255);
      controlMotor(speed);
  } else if (command.startsWith("SERVO:")) {
      // Control servo angle (0-180)
      int angle = command.substring(6).toInt();
      angle = constrain(angle, 0, 180);
      controlServo(angle);
  } else if (command.startsWith("CONTROL:")) {
      // Control servo angle (0-180)
      String commands_str = command.substring(8);
      int colonIndex = commands_str.indexOf(":");  // Find the index of the colon
      int throttle = commands_str.substring(0, colonIndex).toInt();
      int angle = commands_str.substring(colonIndex + 1).toInt();
      controlMotor(throttle);
      controlServo(angle);
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

    
  // Initialize the servo
  myServo.attach(servoPin);  // Attach servo to pin
  controlServo(90);
  return ESP_OK;
};

// Motor setup
esp_err_t init_motor() {
    // Initialize motor pins
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  controlMotor(0); // Start motor at 0 speed

  return ESP_OK;
};

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  Serial.setDebugOutput(true);

  init_camera();
  init_wifi();
  init_servo();
  init_motor();
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
    // Serial.println("image sent");
    esp_camera_fb_return(fb);
    client.poll();
  }
  vTaskDelay(1);
}
