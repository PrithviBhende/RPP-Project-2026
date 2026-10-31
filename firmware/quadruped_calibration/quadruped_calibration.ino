#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "calibration-portal.h"

// --- Access Point Configuration ---
#define AP_SSID  "Calibration-Tool"
#define AP_PASS  "12345678"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_I2C_ADDR 0x3C

// I2C Pins for ESP32 WROOM-32
#define I2C_SDA 21
#define I2C_SCL 22

DNSServer dnsServer;
const byte DNS_PORT = 53;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
WebServer server(80);

Servo servos[8];
// ESP32 WROOM-32 Pinout: R1, R2, L1, L2, R4, R3, L3, L4
const int servoPins[8] = {15, 2, 23, 19, 4, 16, 17, 18};

int servoNameToIndex(const String& servo) {
  if (servo == "R1") return 0;
  if (servo == "R2") return 1;
  if (servo == "L1") return 2;
  if (servo == "L2") return 3;
  if (servo == "R4") return 4;
  if (servo == "R3") return 5;
  if (servo == "L3") return 6;
  if (servo == "L4") return 7;
  return -1;
}

const char* getJointFriendlyName(const String& motor) {
  if (motor == "R1") return "FR-Hip (15)";
  if (motor == "R4") return "FR-Knee (4)";
  if (motor == "L1") return "FL-Hip (23)";
  if (motor == "L3") return "FL-Knee (17)";
  if (motor == "R2") return "RR-Hip (2)";
  if (motor == "R3") return "RR-Knee (16)";
  if (motor == "L2") return "RL-Hip (19)";
  if (motor == "L4") return "RL-Knee (18)";
  return "Unknown";
}

void updateOLED(const String& motor, int angle) {
  if (!display.getBuffer()) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Header
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("CALIBRATION MODE"));
  display.print(F("IP: "));
  display.println(WiFi.softAPIP());
  display.drawLine(0, 18, 128, 18, SSD1306_WHITE);

  // Joint Name
  display.setCursor(0, 23);
  display.print(F("Joint: "));
  display.print(motor);
  display.print(F(" "));
  display.println(getJointFriendlyName(motor));

  // Angle Display
  display.setCursor(0, 36);
  display.setTextSize(2);
  display.print(angle);
  display.setTextSize(1);
  display.print((char)247);
  display.print(F(" deg"));

  // Progress Bar (0 - 180 mapped to 0 - 124 px)
  int barWidth = map(angle, 0, 180, 0, 124);
  display.drawRect(0, 54, 128, 9, SSD1306_WHITE);
  display.fillRect(2, 56, barWidth, 5, SSD1306_WHITE);

  display.display();
}

void updateOLEDAll(int angle) {
  if (!display.getBuffer()) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("CALIBRATION MODE"));
  display.print(F("IP: "));
  display.println(WiFi.softAPIP());
  display.drawLine(0, 18, 128, 18, SSD1306_WHITE);

  display.setCursor(0, 23);
  display.println(F("ALL 8 MOTORS:"));

  display.setCursor(0, 36);
  display.setTextSize(2);
  display.print(angle);
  display.setTextSize(1);
  display.print((char)247);
  display.print(F(" deg"));

  int barWidth = map(angle, 0, 180, 0, 124);
  display.drawRect(0, 54, 128, 9, SSD1306_WHITE);
  display.fillRect(2, 56, barWidth, 5, SSD1306_WHITE);

  display.display();
}

void handleRoot() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/html", calibration_index_html);
}

void handleRawCommand() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (server.hasArg("motor") && server.hasArg("val")) {
    String motorName = server.arg("motor");
    int val = server.arg("val").toInt();

    int idx = servoNameToIndex(motorName);
    if (idx != -1 && val >= 0 && val <= 180) {
      servos[idx].write(val);
      updateOLED(motorName, val);
      Serial.print("Motor ");
      Serial.print(motorName);
      Serial.print(" (");
      Serial.print(getJointFriendlyName(motorName));
      Serial.print(") set to RAW ");
      Serial.println(val);
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "Invalid motor or angle");
    }
  } else {
    server.send(400, "text/plain", "Missing args");
  }
}

void handleAllCommand() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (server.hasArg("val")) {
    int val = server.arg("val").toInt();
    if (val >= 0 && val <= 180) {
      for (int i = 0; i < 8; i++) {
        servos[i].write(val);
        delay(10); // slight stagger to mitigate voltage dips
      }
      updateOLEDAll(val);
      Serial.print("All motors set to ");
      Serial.println(val);
      server.send(200, "text/plain", "OK");
      return;
    }
  }
  server.send(400, "text/plain", "Missing or invalid val");
}

void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
    Serial.println(F("SSD1306 OLED init failed. Continuing..."));
  } else {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println(F("CALIBRATION MODE"));
    display.println(F("Starting Servos..."));
    display.display();
  }

  // Allocate 4 PWM timers for ESP32Servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < 8; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(servoPins[i], 500, 2500);
    servos[i].write(90); // Default neutral spawn
    delay(40);          // Stagger power ramp
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress myIP = WiFi.softAPIP();

  Serial.print("AP Created. IP: ");
  Serial.println(myIP);

  if (display.getBuffer()) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println(F("CALIBRATION READY"));
    display.drawLine(0, 12, 128, 12, SSD1306_WHITE);
    display.setCursor(0, 18);
    display.print(F("WiFi: "));
    display.println(AP_SSID);
    display.print(F("Pass: "));
    display.println(AP_PASS);
    display.print(F("IP:   "));
    display.println(myIP);
    display.display();
  }

  dnsServer.start(DNS_PORT, "*", myIP);

  server.on("/", handleRoot);
  server.on("/raw", handleRawCommand);
  server.on("/all", handleAllCommand);

  server.onNotFound([]() {
    server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString(), true);
    server.send(302, "text/plain", "");
  });

  server.begin();
  Serial.println("Web server started on http://192.168.4.1");
}

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();
  delay(5);
}
