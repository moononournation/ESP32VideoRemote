#define SSID_NAME "Strider Walker V6"
#define SSID_PASSWORD ""

#define MJPEG_BUFFER_SIZE (320 * 240 * 2 / 4)

#define HTTP_HOST "192.168.4.1"
#define HTTP_PORT 80
#define STREAM_PORT 81
#define STREAM_PATH "/stream"
#define MOTOR_STOP_PATH "/motor?la=0&lb=0&ra=0&rb=0"
#define MOTOR_FORWARD_PATH "/motor?la=0&lb=255&ra=0&rb=255"
#define MOTOR_BACKWARD_PATH "/motor?la=255&lb=0&ra=255&rb=0"
#define MOTOR_LEFT_PATH "/motor?la=255&lb=0&ra=0&rb=255"
#define MOTOR_RIGHT_PATH "/motor?la=0&lb=255&ra=255&rb=0"

enum motorCommand
{
  stop,
  forward,
  backward,
  left,
  right
};
motorCommand lastMotorCommand = stop;

#include <WiFi.h>
#include <HTTPClient.h>
WiFiClient streamClient;
HTTPClient streamHttp;
WiFiClient motorClient;
HTTPClient motorHttp;

/* Arduino_GFX */
#include <Arduino_GFX_Library.h>
#define TFT_BRIGHTNESS 128
#define TFT_BL 14
Arduino_ESP32SPI *bus = new Arduino_ESP32SPI(21 /* DC */, 5 /* CS */, SCK, MOSI, MISO);
Arduino_GFX *gfx = new Arduino_ST7789(bus, -1 /* RST */, 1 /* rotation */, true /* IPS */);

/* MJPEG Video */
#include "MjpegClass.h"
static MjpegClass mjpeg;

// pixel drawing callback
static int drawMCU(JPEGDRAW *pDraw)
{
  // log_v("Draw pos = %d,%d. size = %d x %d", pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight);
  gfx->draw16bitBeRGBBitmap(pDraw->x, pDraw->y, pDraw->pPixels, pDraw->iWidth, pDraw->iHeight);
  return 1;
} /* drawMCU() */

void setup()
{
  gfx->begin(80000000);
  gfx->fillScreen(BLACK);

  WiFi.begin(SSID_NAME, SSID_PASSWORD);

#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
#endif

  pinMode(34, INPUT); // X-axis
  pinMode(35, INPUT); // Y-axis
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    // wait for WiFi connection
    delay(500);
  }
  else
  {
    log_v("[HTTP] begin...");
    streamHttp.begin(streamClient, HTTP_HOST, STREAM_PORT, STREAM_PATH);

    log_v("[HTTP] GET...");
    int httpCode = streamHttp.GET();

    log_v("[HTTP] GET... code: %d", httpCode);
    // HTTP header has been send and Server response header has been handled
    if (httpCode <= 0)
    {
      log_e("[HTTP] GET... failed, error: %s", streamHttp.errorToString(httpCode).c_str());
    }
    else
    {
      if (httpCode != HTTP_CODE_OK)
      {
        log_e("[HTTP] Not OK!");
      }
      else
      {
        // get tcp stream
        WiFiClient *stream = streamHttp.getStreamPtr();

        uint8_t *mjpeg_buf = (uint8_t *)malloc(MJPEG_BUFFER_SIZE);

        // init Video
        mjpeg.setup(stream, mjpeg_buf, drawMCU, true, true);

        while (streamHttp.connected())
        {
          unsigned long s = millis();
          // Read video
          mjpeg.readMjpegBuf();
          unsigned long r = millis() - s;
          // Play video
          mjpeg.drawJpg();
          unsigned long d = millis() - s - r;
          // log_v("[Mjpeg] read used: %lu, draw used: %lu", r, d);

          int xVal = analogRead(34);
          int yVal = analogRead(35);
          // log_v("xVal: %d, yVal: %d", xVal, yVal);

          motorCommand currentMotorCommand = stop;
          if (yVal > 3800)
          {
            currentMotorCommand = forward;
          }
          else if (yVal > 1600)
          {
            currentMotorCommand = backward;
          }
          else if (xVal > 3800)
          {
            currentMotorCommand = left;
          }
          else if (xVal > 1600)
          {
            currentMotorCommand = right;
          }

          if (currentMotorCommand != lastMotorCommand)
          {
            switch (currentMotorCommand)
            {
            case forward:
              log_i("forward");
              motorHttp.begin(motorClient, HTTP_HOST, HTTP_PORT, MOTOR_FORWARD_PATH);
              break;
            case backward:
              log_i("backward");
              motorHttp.begin(motorClient, HTTP_HOST, HTTP_PORT, MOTOR_BACKWARD_PATH);
              break;
            case left:
              log_i("left");
              motorHttp.begin(motorClient, HTTP_HOST, HTTP_PORT, MOTOR_LEFT_PATH);
              break;
            case right:
              log_i("right");
              motorHttp.begin(motorClient, HTTP_HOST, HTTP_PORT, MOTOR_RIGHT_PATH);
              break;
            default:
              log_i("stop");
              motorHttp.begin(motorClient, HTTP_HOST, HTTP_PORT, MOTOR_STOP_PATH);
            }
            motorHttp.GET();
            motorHttp.end();
            lastMotorCommand = currentMotorCommand;
          }
        }
      }
    }

    log_v("[HTTP] connection closed.");

    streamHttp.end();
  }
}