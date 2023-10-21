#define SSID_NAME "XIAO nanotank"
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
#define FRAMESIZE_QVGA_PATH "/control?var=framesize&val=5"
#define LED_ON_PATH "/gpio?pin=4&val=1"
#define LED_OFF_PATH "/gpio?pin=4&val=0"

enum motorCommand
{
  stop,
  forward,
  backward,
  left,
  right
};
motorCommand lastMotorCommand = stop;
int lastBtnStartStatus = 1;
int lastBtnAStatus = 1;
int lastLedCommand = 0;

#include <WiFi.h>
#include <HTTPClient.h>
WiFiClient streamClient;
HTTPClient streamHttp;
WiFiClient controlClient;
HTTPClient controlHttp;

/* Arduino_GFX */
#include <Arduino_GFX_Library.h>
#define TFT_BL 14
Arduino_ESP32SPI *bus = new Arduino_ESP32SPI(21 /* DC */, 5 /* CS */, SCK, MOSI, MISO);
//Arduino_ILI9341 *gfx = new Arduino_ILI9341(bus, -1 /* RST */, 3 /* rotation */);
 Arduino_GFX *gfx = new Arduino_ST7789(bus, -1 /* RST */, 3 /* rotation */, true /* IPS */);

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
  gfx->begin();
  gfx->fillScreen(BLACK);

  WiFi.begin(SSID_NAME, SSID_PASSWORD);

#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
#endif

  pinMode(13, INPUT_PULLUP); // Button Menu
  pinMode(0, INPUT_PULLUP);  // Button Volume
  pinMode(27, INPUT_PULLUP); // Button Select
  pinMode(39, INPUT_PULLUP); // Button Start
  pinMode(32, INPUT_PULLUP); // Button A
  pinMode(33, INPUT_PULLUP); // Button B
  pinMode(34, INPUT);        // X-axis
  pinMode(35, INPUT);        // Y-axis
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
    controlHttp.begin(controlClient, HTTP_HOST, HTTP_PORT, FRAMESIZE_QVGA_PATH);
    controlHttp.GET();
    controlHttp.end();

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

        bool recycleStream = false;
        while ((streamHttp.connected()) && (!recycleStream))
        {
          unsigned long s = millis();
          // Read video
          mjpeg.readMjpegBuf();
          unsigned long r = millis() - s;
          // Play video
          mjpeg.drawJpg();
          unsigned long d = millis() - s - r;
          // log_v("[Mjpeg] read used: %lu, draw used: %lu", r, d);

          int btnMenu = digitalRead(13);
          int btnVolume = digitalRead(0);
          int btnSelect = digitalRead(27);
          int btnStart = digitalRead(39);
          int btnA = digitalRead(32);
          int btnB = digitalRead(33);
          int xVal = analogRead(34);
          int yVal = analogRead(35);
          // log_v("btnMenu: %d, btnVolume: %d, btnSelect: %d, btnStart: %d, btnA: %d, btnB: %d, xVal: %d, yVal: %d", btnMenu, btnVolume, btnSelect, btnStart, btnA, btnB, xVal, yVal);

          if (lastBtnStartStatus != btnStart)
          {
            if (btnStart == 0) // pressed
            {
              log_v("Recycle video stream");
              recycleStream = true;
            }
            lastBtnStartStatus = btnStart;
          }

          if (lastBtnAStatus != btnA)
          {
            if (btnA == 0) // pressed
            {
              if (lastLedCommand == 0)
              {
                log_v("LED on");
                controlHttp.begin(controlClient, HTTP_HOST, HTTP_PORT, LED_ON_PATH);
                controlHttp.GET();
                controlHttp.end();
                lastLedCommand = 1;
              }
              else
              {
                log_v("LED off");
                controlHttp.begin(controlClient, HTTP_HOST, HTTP_PORT, LED_OFF_PATH);
                controlHttp.GET();
                controlHttp.end();
                lastLedCommand = 0;
              }
            }
            lastBtnAStatus = btnA;
          }
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
              controlHttp.begin(controlClient, HTTP_HOST, HTTP_PORT, MOTOR_FORWARD_PATH);
              break;
            case backward:
              log_i("backward");
              controlHttp.begin(controlClient, HTTP_HOST, HTTP_PORT, MOTOR_BACKWARD_PATH);
              break;
            case left:
              log_i("left");
              controlHttp.begin(controlClient, HTTP_HOST, HTTP_PORT, MOTOR_LEFT_PATH);
              break;
            case right:
              log_i("right");
              controlHttp.begin(controlClient, HTTP_HOST, HTTP_PORT, MOTOR_RIGHT_PATH);
              break;
            default:
              log_i("stop");
              controlHttp.begin(controlClient, HTTP_HOST, HTTP_PORT, MOTOR_STOP_PATH);
            }
            controlHttp.GET();
            controlHttp.end();
            lastMotorCommand = currentMotorCommand;
          }
        }
        stream->flush();
      }
    }

    log_v("[HTTP] connection closed.");
    streamHttp.end();
  }
}
