#define SSID_NAME "ESP32CameraRobot"
#define SSID_PASSWORD ""
#define URL "http://192.168.4.1:81/stream"

#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_jpg_decode.h>
#include <SPI.h>
#include "Arduino_HWSPI.h"
#include "Arduino_GFX.h"    // Core graphics library by Adafruit
#include "Arduino_ST7789.h" // Hardware-specific library for ST7789 (with or without CS pin)

#define TFT_BL 14
Arduino_HWSPI *bus = new Arduino_HWSPI(21 /* DC */, 5 /* CS */, SCK, MOSI, MISO);
Arduino_ST7789 *tft = new Arduino_ST7789(bus, 33 /* RST */, 1 /* rotation */, true /* IPS */);
// Arduino_HWSPI *bus = new Arduino_HWSPI(27 /* DC */, 5 /* CS */, SCK, MOSI, MISO);
// Arduino_ST7789 *tft = new Arduino_ST7789(bus, 33 /* RST */, 3 /* rotation */, true /* IPS */);

#define WIDTH 320
#define HEIGHT 240
#define RATIO_BOUND 10

typedef struct
{
  int len;
  int offset;
  uint8_t buff[2 * WIDTH * HEIGHT / RATIO_BOUND];
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
  Serial.begin(115200);
  tft->begin();
  tft->fillScreen(BLACK);
  // tft->setAddrWindow(40, 30, WIDTH, HEIGHT);

  WiFi.begin(SSID_NAME, SSID_PASSWORD);

#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
#endif

  framebuffer = (uint16_t *)heap_caps_malloc(WIDTH * HEIGHT * 2, MALLOC_CAP_SPIRAM);
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

    Serial.print("[HTTP] begin...\n");
    http.begin(URL);

    Serial.print("[HTTP] GET...\n");
    int httpCode = http.GET();

    Serial.printf("[HTTP] GET... code: %d\n", httpCode);
    // HTTP header has been send and Server response header has been handled
    if (httpCode <= 0)
    {
      Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
    }
    else
    {
      if (httpCode != HTTP_CODE_OK)
      {
        Serial.printf("[HTTP] Not OK!\n");
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
          Serial.printf("[HTTP] size: %d\n", curr_buffer->len);

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

              // Serial.printf("[HTTP] read: %d\n", c);

              if (l > 0)
              {
                l -= c;
              }
            }
          }

          if (bufflock)
          {
            drop_frame++;
          }
          else
          {
            decode_frame++;
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
          Serial.printf("[JPG] drop: %d, decode: %d\n", drop_frame, decode_frame);
        }
      }
    }

    Serial.println();
    Serial.print("[HTTP] connection closed.\n");

    http.end();
  }
}

void drawFramebufferTask(void *parameter)
{
  buffer_t *b = (buffer_t *)parameter;

  Serial.printf("[JPG] start: %d\n", millis());
  esp_jpg_decode(b->len, JPG_SCALE_NONE, buff_reader, framebuffer_writer, b /* arg */);
  Serial.printf("[JPG] end: %d\n", millis());

  bufflock = false;

  Serial.printf("[TFT] start: %d\n", millis());
  tft->startWrite();
  tft->writePixels(framebuffer, WIDTH * HEIGHT * 2);
  tft->endWrite();
  Serial.printf("[TFT] end: %d\n", millis());

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
    // Serial.printf("%d, %d, %d, %d\n", x, y, w, h);
    for (int i = 0; i < h; i++)
    {
      for (int j = 0; j < w; j++)
      {
        framebuffer[(y + i) * WIDTH + x + j] = tft->color565(*(data++), *(data++), *(data++));
      }
    }
  }
  return true; // Continue to decompression
}
