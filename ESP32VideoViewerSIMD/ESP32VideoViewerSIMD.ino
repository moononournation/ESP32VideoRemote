#define SSID_NAME "XIAO nanotank"
#define SSID_PASSWORD ""

#define MJPEG_BUFFER_SIZE (640 * 480 * 2 / 7)

#define HTTP_HOST "192.168.4.1"
#define HTTP_PORT 80
#define STREAM_PORT 81
#define STREAM_PATH "/stream"
#define FRAMESIZE_VGA_PATH "/control?var=framesize&val=8"

#include <WiFi.h>
#include <HTTPClient.h>
WiFiClient streamClient;
HTTPClient streamHttp;
WiFiClient controlClient;
HTTPClient controlHttp;

/* Arduino_GFX */
#include <Arduino_GFX_Library.h>
#define GFX_DEV_DEVICE ESP32_S3_RGB
#define GFX_BL 38
Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
    18 /* DE */, 17 /* VSYNC */, 16 /* HSYNC */, 21 /* PCLK */,
    4 /* R0 */, 3 /* R1 */, 2 /* R2 */, 1 /* R3 */, 0 /* R4 */,
    10 /* G0 */, 9 /* G1 */, 8 /* G2 */, 7 /* G3 */, 6 /* G4 */, 5 /* G5 */,
    15 /* B0 */, 14 /* B1 */, 13 /* B2 */, 12 /* B3 */, 11 /* B4 */,
    1 /* hsync_polarity */, 20 /* hsync_front_porch */, 30 /* hsync_pulse_width */, 38 /* hsync_back_porch */,
    1 /* vsync_polarity */, 4 /* vsync_front_porch */, 3 /* vsync_pulse_width */, 15 /* vsync_back_porch */,
    10, 16000000);
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(
    640 /* width */, 480 /* height */, rgbpanel, 0 /* rotation */, true /* auto_flush */);

/* MJPEG Video */
#include "MjpegClass.h"
static MjpegClass mjpeg;

void setup()
{
  Serial.begin(115200);
  // Serial.setDebugOutput(true);
  // while(!Serial);
  Serial.println("ESP32VideoViewerRGB");

#ifdef GFX_EXTRA_PRE_INIT
  GFX_EXTRA_PRE_INIT();
#endif

  // Init Display
  if (!gfx->begin())
  {
    Serial.println("gfx->begin() failed!");
  }
  gfx->fillScreen(BLACK);

#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
#endif

  WiFi.begin(SSID_NAME, SSID_PASSWORD);
  log_v("Connect to "SSID_NAME);
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    // wait for WiFi connection
    delay(500);
    log_v(".");
  }
  else
  {
    controlHttp.begin(controlClient, HTTP_HOST, HTTP_PORT, FRAMESIZE_VGA_PATH);
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
        mjpeg.setup(
            stream, mjpeg_buf, gfx->getFramebuffer(), true /* useBigEndian */,
            gfx->width() /* widthLimit */, gfx->height() /* heightLimit */);

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
          log_v("[Mjpeg] read used: %lu, draw used: %lu", r, d);
        }
        stream->flush();
      }
    }

    log_v("[HTTP] connection closed.");
    streamHttp.end();
  }
}
