#define SSID_NAME "ESP32CameraRobot"
#define SSID_PASSWORD ""
// #define URL "http://192.168.4.1:81/vga"
// #define JPG_SCALE JPG_SCALE_2X
// #define JPG_WIDTH 640
// #define JPG_HEIGHT 480
// #define DISP_WIDTH 320
// #define DISP_HEIGHT 240
// #define RATIO_BOUND 4
#define URL "http://192.168.4.1:81/qvga"
#define JPG_SCALE JPG_SCALE_NONE
#define JPG_WIDTH 320
#define JPG_HEIGHT 240
#define DISP_WIDTH 320
#define DISP_HEIGHT 240
#define RATIO_BOUND 4
// #define URL "http://192.168.4.1:81/hqvga"
// #define JPG_SCALE JPG_SCALE_NONE
// #define JPG_WIDTH 240
// #define JPG_HEIGHT 160
// #define DISP_WIDTH 240
// #define DISP_HEIGHT 160
// #define RATIO_BOUND 4
// #define URL "http://192.168.4.1:81/qcif"
// #define JPG_SCALE JPG_SCALE_NONE
// #define JPG_WIDTH 176
// #define JPG_HEIGHT 144
// #define DISP_WIDTH 176
// #define DISP_HEIGHT 144
// #define RATIO_BOUND 4

#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_jpg_decode.h>

#include "Arduino_GFX_Library.h"
#define TFT_BL 14
Arduino_ESP32SPI *bus = new Arduino_ESP32SPI(21 /* DC */, 5 /* CS */, SCK, MOSI, MISO);
Arduino_ST7789 *tft = new Arduino_ST7789(bus, -1 /* RST */, 1 /* rotation */, true /* IPS */);
// Arduino_ESP32SPI *bus = new Arduino_ESP32SPI(27 /* DC */, 5 /* CS */, SCK, MOSI, MISO);
// Arduino_ST7789 *tft = new Arduino_ST7789(bus, 33 /* RST */, 3 /* rotation */, true /* IPS */);

typedef struct
{
  int len;
  int offset;
  uint8_t *buff;
} buffer_t;

buffer_t buffer1;
buffer_t buffer2;
uint8_t curr_buffer_idx = 1;
buffer_t *curr_buffer = &buffer1;
bool bufflock = false;
uint16_t drop_frame = 0;
uint16_t decode_frame = 0;

uint16_t *framebuffer = NULL;

void setup()
{
  tft->begin();
  tft->fillScreen(BLACK);
  tft->setAddrWindow((tft->width() - DISP_WIDTH) / 2, (tft->height() - DISP_HEIGHT) / 2, DISP_WIDTH, DISP_HEIGHT);

  WiFi.begin(SSID_NAME, SSID_PASSWORD);

#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
#endif

  buffer1.buff = (uint8_t *)heap_caps_malloc(2 * JPG_WIDTH * JPG_HEIGHT / RATIO_BOUND, MALLOC_CAP_SPIRAM);
  buffer2.buff = (uint8_t *)heap_caps_malloc(2 * JPG_WIDTH * JPG_HEIGHT / RATIO_BOUND, MALLOC_CAP_SPIRAM);
  framebuffer = (uint16_t *)heap_caps_malloc(DISP_WIDTH * DISP_WIDTH * 2, MALLOC_CAP_SPIRAM);
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
    HTTPClient http;

    log_i("[HTTP] begin...\n");
    http.begin(URL);

    log_i("[HTTP] GET...\n");
    int httpCode = http.GET();

    log_i("[HTTP] GET... code: %d\n", httpCode);
    // HTTP header has been send and Server response header has been handled
    if (httpCode <= 0)
    {
      log_i("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
    }
    else
    {
      if (httpCode != HTTP_CODE_OK)
      {
        log_i("[HTTP] Not OK!\n");
      }
      else
      {
        // get tcp stream
        WiFiClient *stream = http.getStreamPtr();
        while (http.connected())
        {
          curr_buffer->len = -1;
          while (http.connected() && (curr_buffer->len < 0))
          {
            stream->find('\n');
            if (stream->read() == 'C')
            {
              if (stream->read() == 'o')
              {
                if (stream->read() == 'n')
                {
                  if (stream->read() == 't')
                  {
                    if (stream->read() == 'e')
                    {
                      if (stream->read() == 'n')
                      {
                        if (stream->read() == 't')
                        {
                          if (stream->read() == '-')
                          {
                            if (stream->read() == 'L')
                            {
                              if (stream->read() == 'e')
                              {
                                if (stream->read() == 'n')
                                {
                                  if (stream->read() == 'g')
                                  {
                                    if (stream->read() == 't')
                                    {
                                      if (stream->read() == 'h')
                                      {
                                        if (stream->read() == ':')
                                        {
                                          if (stream->read() == ' ')
                                          {
                                            curr_buffer->len = stream->parseInt();
                                          }
                                        }
                                      }
                                    }
                                  }
                                }
                              }
                            }
                          }
                        }
                      }
                    }
                  }
                }
              }
            }
          }
          log_i("[HTTP] size: %d\n", curr_buffer->len);

          stream->find('\n');
          stream->find('\n');
          stream->find('\n');
          stream->find('\n');

          // read all data from server
          uint8_t *p = curr_buffer->buff;
          int l = curr_buffer->len;
          while (http.connected() && (l > 0))
          {
            // get available data size
            size_t size = stream->available();

            if (size)
            {
              int s = ((size > l) ? l : size);
              int c = stream->readBytes(p, s);
              p += c;

              // log_i("[HTTP] read: %d\n", c);

              if (l > 0)
              {
                l -= c;
              }
            }
          }

          if (bufflock)
          {
            ++drop_frame;
          }
          else
          {
            ++decode_frame;
            bufflock = true;
            curr_buffer->offset = 0;

            xTaskCreate(
                drawFramebufferTask,   /* Task function. */
                "DrawFramebufferTask", /* String with name of task. */
                10000,                 /* Stack size in bytes. */
                curr_buffer,           /* Parameter passed as input of the task */
                1,                     /* Priority of the task. */
                NULL);                 /* Task handle. */

            if (curr_buffer_idx == 1)
            {
              curr_buffer_idx = 2;
              curr_buffer = &buffer2;
            }
            else
            {
              curr_buffer_idx = 1;
              curr_buffer = &buffer1;
            }
          }
          log_i("[JPG] drop: %d, decode: %d\n", drop_frame, decode_frame);
        }
      }
    }

    log_i"[HTTP] connection closed.\n");

    http.end();
  }
}

void drawFramebufferTask(void *parameter)
{
  buffer_t *b = (buffer_t *)parameter;

  log_i("[JPG] start: %d\n", millis());
  esp_jpg_decode(b->len, JPG_SCALE, buff_reader, framebuffer_writer, b /* arg */);
  log_i("[JPG] end: %d\n", millis());

  bufflock = false;

  log_i("[TFT] start: %d\n", millis());
  tft->startWrite();
  tft->writePixels(framebuffer, DISP_WIDTH * DISP_HEIGHT);
  tft->endWrite();
  log_i("[TFT] end: %d\n", millis());

  vTaskDelete(NULL);
}

static size_t buff_reader(void *arg, size_t index, uint8_t *buf, size_t len)
{
  buffer_t *b = (buffer_t *)arg;

  int l = len;
  if ((b->offset + l) > b->len)
  {
    l = b->len - b->offset;
  }

  if (buf)
  {
    memcpy(buf, b->buff + b->offset, l);
  }
  b->offset += l;
  return l; // Returns number of bytes read
}

static bool framebuffer_writer(void *arg, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *data)
{
  if (data)
  {
    uint16_t *pixels = (uint16_t *)data;
    log_d("%d, %d, %d, %d\n", x, y, w, h);
    for (int i = 0; i < h; ++i)
    {
      for (int j = 0; j < w; ++j)
      {
        framebuffer[(y + i) * DISP_WIDTH + x + j] = tft->color565(*(data++), *(data++), *(data++));
      }
    }
  }
  return true; // Continue to decompression
}
