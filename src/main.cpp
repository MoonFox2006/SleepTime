#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "Date.h"

const uint32_t RTC_SIGNATURE = 0xA1B2C3D4;
const uint32_t SLEEP_TIME = 600; // 10 min.

const char WIFI_SSID[] PROGMEM = "******"; // Your WiFi SSID
const char WIFI_PSWD[] PROGMEM = "******"; // Your WiFi password
const uint32_t WIFI_TIMEOUT = 30000; // 30 sec.

const char NTP_SERVER[] PROGMEM = "pool.ntp.org";
const int8_t NTP_TZ = 3; // Your time zone
const uint32_t NTP_TIMEOUT = 1000; // 1 sec.

struct __packed rtcdata_t {
  uint32_t lastTime;
  int32_t fixTime;
  uint8_t crc;
};

static uint8_t crc8(const uint8_t *data, uint16_t len, uint8_t crc = 0xFF) {
  while (len--) {
    crc ^= *data++;
    for (uint8_t bit = 8; bit > 0; --bit) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x31;
      else
        crc <<= 1;
    }
  }
  return crc;
}

static bool getFixTime(uint32_t &lastTime, int32_t &fixTime) {
  uint32_t signature;

  if (ESP.rtcUserMemoryRead(0, &signature, sizeof(signature)) && (signature == RTC_SIGNATURE)) {
    rtcdata_t data;

    if (ESP.rtcUserMemoryRead(1, (uint32_t*)&data, sizeof(data)) &&
      (data.crc == crc8((uint8_t*)&data, sizeof(data) - sizeof(data.crc), crc8((uint8_t*)&signature, sizeof(signature))))) {
      lastTime = data.lastTime;
      fixTime = data.fixTime;
      return true;
    }
  }
  lastTime = fixTime = 0;
  return false;
}

static bool setFixTime(uint32_t lastTime, int32_t fixTime) {
  uint32_t signature = RTC_SIGNATURE;

  if (ESP.rtcUserMemoryWrite(0, &signature, sizeof(signature))) {
    rtcdata_t data;

    data.lastTime = lastTime;
    data.fixTime = fixTime;
    data.crc = crc8((uint8_t*)&data, sizeof(data) - sizeof(data.crc), crc8((uint8_t*)&signature, sizeof(signature)));
    return ESP.rtcUserMemoryWrite(1, (uint32_t*)&data, sizeof(data));
  }
  return false;
}

static bool wifiConnect(PGM_P ssid, PGM_P pswd, uint32_t timeout) {
  char *_ssid, *_pswd;

  _ssid = new char[strlen_P(ssid) + 1];
  if (! _ssid)
    return false;
  _pswd = new char[strlen_P(pswd) + 1];
  if (! _pswd) {
    delete[] _ssid;
    return false;
  }
  strcpy_P(_ssid, ssid);
  strcpy_P(_pswd, pswd);
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.begin(_ssid, _pswd);
  Serial.print(F("Connecting to \""));
  Serial.print(_ssid);
  Serial.print('"');
  delete[] _pswd;
  delete[] _ssid;

  uint32_t start = millis();

  while ((! WiFi.isConnected()) && (millis() - start < timeout)) {
    Serial.print('.');
    digitalWrite(LED_BUILTIN, LOW);
    delay(25);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500 - 25);
  }
  if (WiFi.isConnected()) {
    Serial.print(F(" OK (IP "));
    Serial.print(WiFi.localIP());
    Serial.println(')');
    return true;
  } else {
    WiFi.disconnect();
    Serial.println(F("FAIL!"));
    return false;
  }
}

static uint32_t ntpTime(PGM_P server, int8_t tz, uint32_t timeout) {
  const uint8_t NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
  const uint32_t SEVENTY_YEARS = 2208988800UL;

  uint32_t result = 0;
  WiFiUDP udp;

  if (udp.begin(random(65536 - 1024) + 1024)) {
    char *_server;

    _server = new char[strlen_P(server) + 1];
    if (! _server)
      return 0;
    strcpy_P(_server, server);
    if (udp.beginPacket(_server, 123)) {
      uint8_t buf[NTP_PACKET_SIZE]; // buffer to hold incoming and outgoing packets

      delete[] _server;
      memset(buf, 0, sizeof(buf));
      // Initialize values needed to form NTP request
      buf[0] = 0B11100011; // LI, Version, Mode
      buf[1] = 0; // Stratum, or type of clock
      buf[2] = 6; // Polling Interval
      buf[3] = 0xEC; // Peer Clock Precision
      // 8 bytes of zero for Root Delay & Root Dispersion
      buf[12] = 49;
      buf[13] = 0x4E;
      buf[14] = 49;
      buf[15] = 52;
      if ((udp.write(buf, sizeof(buf)) == sizeof(buf)) && udp.endPacket()) {
        uint32_t start = millis();
        int16_t in;

        while ((! (in = udp.parsePacket())) && (millis() - start < timeout)) {
          delay(1);
        }
        if (in) {
          udp.read(buf, sizeof(buf));
          result = ((uint32_t)buf[40] << 24) | ((uint32_t)buf[41] << 16) | ((uint32_t)buf[42] << 8) | buf[43];
          result -= SEVENTY_YEARS;
          result += tz * 3600;
        }
      }
    } else {
      delete[] _server;
    }
  }
  return result;
}

void setup() {
  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
  Serial.println();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  uint32_t lastTime;
  int32_t fixTime;

  if (getFixTime(lastTime, fixTime)) {
    Serial.print(F("RTC previous fix time is "));
    Serial.println(fixTime);
  }
  if (wifiConnect(WIFI_SSID, WIFI_PSWD, WIFI_TIMEOUT)) {
    uint32_t time;

    {
      uint8_t repeat = 2;

      do {
        time = ntpTime(NTP_SERVER, NTP_TZ, NTP_TIMEOUT);
      } while ((! time) && repeat--);
    }
    if (time) {
      time -= millis() / 1000;
      {
        char str[20];

        Serial.print(F("Wake time is "));
        Serial.println(dateTimeToStr(str, time));
      }
      if (lastTime) {
        int32_t diffTime = time - (lastTime + SLEEP_TIME);

        Serial.print(F("Time difference is "));
        Serial.print(diffTime);
        Serial.println(F(" sec."));
        if (abs(diffTime) > SLEEP_TIME / 10) {
          Serial.println(F("Abnormal time difference ignored!"));
        } else {
          fixTime += diffTime;
        }
      }
      if (! setFixTime(time, fixTime)) {
        Serial.println(F("RTC store error!"));
      }
    } else {
      Serial.println(F("NTP error!"));
    }
  }

  Serial.println(F("Going to sleep..."));
  Serial.flush();
  ESP.deepSleep(((SLEEP_TIME - fixTime) * 1000 - millis()) * 1000);
}

void loop() {}
