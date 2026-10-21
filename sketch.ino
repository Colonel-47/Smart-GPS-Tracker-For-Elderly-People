#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

#define SOS_BTN 4
#define BUZZER 5
#define LED_R 25
#define LED_G 26
#define LED_B 27
#define MPU_ADDR 0x68
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASS = "";
const char* MQTT_HOST = "broker.hivemq.com";
const int   MQTT_PORT = 1883;
const char* MQTT_TOPIC = "smartgps-tn-4821/status";   

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
unsigned long lastReconnect = 0;

Adafruit_SSD1306 display(128, 64, &Wire, -1);

const double HOME_LAT = 34.7406, HOME_LON = 10.7603;
const double SAFE_RADIUS = 200.0; 

const float IMPACT_G = 2.5;
const float FREEFALL_G = 0.5;

double lat = HOME_LAT, lon = HOME_LON, dist = 0;
int dir = 1;
unsigned long lastTick = 0, lastDraw = 0, sosUntil = 0, fallUntil = 0;
bool lastPressed = false, wasOut = false, mpuOk = false;

bool watching = false, windowStarted = false, armed = true;
unsigned long watchStart = 0;
float wMin = 0, wMax = 0, g = 1.0;

double distanceM(double la1, double lo1, double la2, double lo2) {
  double R = 6371000.0;
  double dLat = radians(la2 - la1), dLon = radians(lo2 - lo1);
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(radians(la1)) * cos(radians(la2)) * sin(dLon / 2) * sin(dLon / 2);
  return R * 2 * atan2(sqrt(a), sqrt(1 - a));
}

void setLed(bool r, bool gr, bool b) {
  digitalWrite(LED_R, r);
  digitalWrite(LED_G, gr);
  digitalWrite(LED_B, b);
}

const char* stateNow(unsigned long now) {
  if (now < sosUntil) return "SOS";
  if (now < fallUntil) return "FALL";
  if (dist > SAFE_RADIUS) return "ALERT";
  return "SAFE";
}

void publishState(unsigned long now) {
  if (!mqtt.connected()) return;
  char buf[128];
  snprintf(buf, sizeof(buf),
           "{\"lat\":%.5f,\"lon\":%.5f,\"dist\":%.0f,\"g\":%.2f,\"state\":\"%s\"}",
           lat, lon, dist, g, stateNow(now));
  mqtt.publish(MQTT_TOPIC, buf);
}

void wifiConnect() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASS, 6);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) {
    delay(250);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) Serial.println(" connected");
  else Serial.println(" failed (running offline)");
}

void mqttMaintain(unsigned long now) {
  if (WiFi.status() != WL_CONNECTED) return;
  if (!mqtt.connected()) {
    if (lastReconnect == 0 || now - lastReconnect > 5000) {
      lastReconnect = now;
      String id = "esp32-gps-" + String((uint32_t)esp_random(), HEX);
      if (mqtt.connect(id.c_str())) Serial.println("MQTT connected");
      else Serial.printf("MQTT failed, rc=%d\n", mqtt.state());
    }
  } else {
    mqtt.loop();
  }
}

void mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0);
  if (Wire.endTransmission() != 0) { mpuOk = false; return; }
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x10);
  Wire.endTransmission();
  mpuOk = true;
}

float readG() {
  if (!mpuOk) return 1.0;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6);
  if (Wire.available() < 6) return 1.0;
  int16_t v[3];
  for (int i = 0; i < 3; i++) {
    int hi = Wire.read();
    int lo = Wire.read();
    v[i] = (int16_t)((hi << 8) | lo);
  }
  float x = v[0] / 4096.0, y = v[1] / 4096.0, z = v[2] / 4096.0;
  return sqrt(x * x + y * y + z * z);
}

void checkFall(unsigned long now) {
  g = readG();
  if (!watching) {
    if (armed && (g > IMPACT_G || g < FREEFALL_G)) {
      watching = true; windowStarted = false; watchStart = now;
    }
    if (g > 0.7 && g < 1.3) armed = true;
  } else {
    if (!windowStarted && now - watchStart >= 500) {
      windowStarted = true; wMin = wMax = g;
    }
    if (windowStarted) {
      if (g < wMin) wMin = g;
      if (g > wMax) wMax = g;
    }
    if (now - watchStart >= 1500) {
      watching = false;
      if (wMax - wMin < 0.5) {
        fallUntil = now + 5000;
        armed = false;
        Serial.printf("FALL DETECTED! Location: https://maps.google.com/?q=%.5f,%.5f\n", lat, lon);
        tone(BUZZER, 1500, 1000);
        publishState(now);
      }
    }
  }
}

void drawScreen(const char* st) {
  char label[12];
  snprintf(label, sizeof(label), "%s%s", st, strcmp(st, "SAFE") ? "!" : "");
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("SMART GPS TRACKER");
  display.setCursor(104, 0);
  display.print(mqtt.connected() ? "NET" : "---");
  display.drawFastHLine(0, 10, 128, SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 16);
  display.print(label);
  display.setTextSize(1);
  display.setCursor(0, 40);
  display.printf("Lat: %.5f", lat);
  display.setCursor(0, 48);
  display.printf("Lon: %.5f", lon);
  display.setCursor(0, 56);
  display.printf("Dist:%.0fm G:%.1f", dist, g);
  display.display();
}

void setup() {
  Serial.begin(115200);
  pinMode(SOS_BTN, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) Serial.println("OLED not found");
  mpuInit();
  if (!mpuOk) Serial.println("MPU6050 not found");

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  wifiConnect();
  Serial.println("Smart GPS Tracker started");
}

void loop() {
  unsigned long now = millis();

  bool pressed = (digitalRead(SOS_BTN) == LOW);
  if (pressed && !lastPressed) {
    sosUntil = now + 4000;
    Serial.printf("SOS! Location: https://maps.google.com/?q=%.5f,%.5f\n", lat, lon);
    tone(BUZZER, 2000, 1000);
    publishState(now);
  }
  lastPressed = pressed;

  checkFall(now);
  mqttMaintain(now);

  if (now - lastTick >= 1000) {
    lastTick = now;
    lat += dir * 0.0002;
    lon += dir * 0.0001;
    dist = distanceM(HOME_LAT, HOME_LON, lat, lon);
    if (dir == 1 && dist > 320) dir = -1;
    else if (dir == -1 && dist < 15) dir = 1;

    Serial.printf("Pos: %.5f, %.5f | Distance: %.0f m\n", lat, lon, dist);

    bool out = dist > SAFE_RADIUS;
    if (out && !wasOut) Serial.println("ALERT: Left safe zone!");
    if (!out && wasOut) Serial.println("Back in safe zone");
    wasOut = out;
    if (out && now >= sosUntil && now >= fallUntil) tone(BUZZER, 1000, 300);
    publishState(now);
  }

  const char* st = stateNow(now);
  if (!strcmp(st, "SOS"))        setLed(0, 0, 1);
  else if (!strcmp(st, "FALL"))  setLed(1, 0, 1);
  else if (!strcmp(st, "ALERT")) setLed(1, 0, 0);
  else                           setLed(0, 1, 0);

  if (now - lastDraw >= 200) {
    lastDraw = now;
    drawScreen(st);
  }
  delay(20);
}