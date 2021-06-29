#define SOFTAP
#ifdef SOFTAP
const char *ssid = "ESP32CameraRobot";
const char *password = "";
const int channel = 1;
#else
const char *ssid = "YourAP";
const char *password = "PleaseInputYourPasswordHere";
#endif

// Select camera model
// #define CAMERA_MODEL_WROVER_KIT
// #define CAMERA_MODEL_ESP_EYE
// #define CAMERA_MODEL_ESP32_CAM
// #define CAMERA_MODEL_M5CAM
#define CAMERA_MODEL_M5CAM_PSRAM
// #define CAMERA_MODEL_M5CAM_PSRAM_WIDE
// #define CAMERA_MODEL_AI_THINKER
// #define CAMERA_MODEL_JSZWY_CYIS

#include <WiFi.h>
#include <WiFiAP.h>
#include <esp_wifi.h>
#include <esp_camera.h>

#define SERVO_FREQ 60 // Analog servos run at ~60 Hz updates
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "camera_pins.h"

static Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void startCameraServer();
int getNewAngle(int servo_id);

void setup()
{
  Wire.begin(I2C_SDA_NUM, I2C_SCL_NUM);
  pwm.begin();
  // In theory the internal oscillator is 25MHz but it really isn't
  // that precise. You can 'calibrate' by tweaking this number till
  // you get the frequency you're expecting!
  pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz  
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~60 Hz updates

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
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound())
  {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  }
  else
  {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    log_i("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  //initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID)
  {
    s->set_vflip(s, 1);       //flip it back
    s->set_brightness(s, 1);  //up the blightness just a bit
    s->set_saturation(s, -2); //lower the saturation
  }
  else
  {
    // s->set_brightness(s, 2);
    // s->set_contrast(s, 2);
    s->set_saturation(s, 2);
    s->set_aec2(s, true);
    s->set_gainceiling(s, GAINCEILING_128X);
    s->set_lenc(s, true);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  esp_wifi_set_max_tx_power(127); // max tx
#ifdef SOFTAP
  WiFi.softAP(ssid, password, channel);
#else
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    log_i(".");
  }
  log_i("WiFi connected");
#endif

  startCameraServer();

#ifdef SOFTAP
  log_i("Camera Ready! Use 'http://%s to connect", WiFi.softAPIP().toString());
#else
  log_i("Camera Ready! Use 'http://%s to connect", WiFi.localIP().toString());
#endif
}

void loop()
{
  for (int i = 0; i < 16; ++i)
  {
    int newAngle = getNewAngle(i);
    if (newAngle >= 0)
    {
      log_i("pwm.writeMicroseconds(%d, %d)", i, newAngle);
      pwm.writeMicroseconds(i, newAngle);
    } else {
      pwm.writeMicroseconds(i, 0);
    }
  }

  delay(20);
}
