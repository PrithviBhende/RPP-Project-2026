#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "face-bitmaps.h"
#include "movement-sequences.h"
#include "captive-portal.h"

// --- Access Point Configuration ---
#define AP_SSID  "Quadruped-Controller"  // CHANGED from Sesame-Controller
#define AP_PASS  "12345678"

// --- Station Mode Configuration (Optional) ---
#define NETWORK_SSID ""
#define NETWORK_PASS ""
#define ENABLE_NETWORK_MODE false

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

// Global state for animations
String currentCommand = "";
String currentFaceName = "default";
const unsigned char* const* currentFaceFrames = nullptr;
uint8_t currentFaceFrameCount = 0;
uint8_t currentFaceFrameIndex = 0;
unsigned long lastFaceFrameMs = 0;
int faceFps = 8;
FaceAnimMode currentFaceMode = FACE_ANIM_LOOP;
int8_t faceFrameDirection = 1;
bool faceAnimFinished = false;
int currentFaceFps = 0;
bool idleActive = false;
bool idleBlinkActive = false;
unsigned long nextIdleBlinkMs = 0;
uint8_t idleBlinkRepeatsLeft = 0;

// WiFi Info Scrolling
unsigned long lastInputTime = 0;
bool firstInputReceived = false;
bool showingWifiInfo = false;
int wifiScrollPos = 0;
unsigned long lastWifiScrollMs = 0;
String wifiInfoText = "";

// Network Mode
bool networkConnected = false;
IPAddress networkIP;
String deviceHostname = "quadruped-robot";

Servo servos[8];
// ESP32 WROOM-32 Pinout: R1, R2, L1, L2, R4, R3, L3, L4
const int servoPins[8] = {15, 2, 23, 19, 4, 16, 17, 18};

// Per-servo mechanical calibration
// Order: R1, R2, L1, L2, R4, R3, L3, L4
// Scaling logic has been removed. All angles in this firmware are absolute hardware degrees.

int frameDelay = 100;
int walkCycles = 10;
int motorCurrentDelay = 20;

struct FaceEntry {
  const char* name;
  const unsigned char* const* frames;
  uint8_t maxFrames;
};

static const uint8_t MAX_FACE_FRAMES = 6;

#define MAKE_FACE_FRAMES(name) \
  const unsigned char* const face_##name##_frames[] = { \
    epd_bitmap_##name, epd_bitmap_##name##_1, epd_bitmap_##name##_2, \
    epd_bitmap_##name##_3, epd_bitmap_##name##_4, epd_bitmap_##name##_5 \
  };

#define X(name) MAKE_FACE_FRAMES(name)
FACE_LIST
#undef X
#undef MAKE_FACE_FRAMES

const FaceEntry faceEntries[] = {
#define X(name) { #name, face_##name##_frames, MAX_FACE_FRAMES },
  FACE_LIST
#undef X
  { "default", face_defualt_frames, MAX_FACE_FRAMES }
};

struct FaceFpsEntry {
  const char* name;
  uint8_t fps;
};

const FaceFpsEntry faceFpsEntries[] = {
  { "walk", 1 },
  { "rest", 1 },
  { "swim", 1 },
  { "dance", 1 },
  { "wave", 1 },
  { "point", 5 },
  { "stand", 1 },
  { "cute", 1 },
  { "pushup", 1 },
  { "freaky", 1 },
  { "bow", 1 },
  { "worm", 1 },
  { "shake", 1 },
  { "shrug", 1 },
  { "dead", 2 },
  { "crab", 1 },
  { "idle", 1 },
  { "idle_blink", 7 },
  { "default", 1 },
  { "happy", 1 },
  { "talk_happy", 1 },
  { "sad", 1 },
  { "talk_sad", 1 },
  { "angry", 1 },
  { "talk_angry", 1 },
  { "surprised", 1 },
  { "talk_surprised", 1 },
  { "sleepy", 1 },
  { "talk_sleepy", 1 },
  { "love", 1 },
  { "talk_love", 1 },
  { "excited", 1 },
  { "talk_excited", 1 },
  { "confused", 1 },
  { "talk_confused", 1 },
  { "thinking", 1 },
  { "talk_thinking", 1 },
};

void setServoAngle(uint8_t channel, int angle);
void updateFaceBitmap(const unsigned char* bitmap);
void setFace(const String& faceName);
void setFaceMode(FaceAnimMode mode);
void setFaceWithMode(const String& faceName, FaceAnimMode mode);
void updateAnimatedFace();
void delayWithFace(unsigned long ms);
void enterIdle();
void exitIdle();
void updateIdleBlink();
int getFaceFpsForName(const String& faceName);
bool pressingCheck(String cmd, int ms);
void handleGetSettings();
void handleSetSettings();
void handleGetStatus();
void handleApiCommand();
void updateWifiInfoScroll();
void recordInput();

void handleRoot() {
  server.send(200, "text/html", index_html);
}

void handleCommandWeb() {
  if (server.hasArg("pose")) {
    currentCommand = server.arg("pose");
    recordInput();
    exitIdle();
    server.send(200, "text/plain", "OK"); 
  } 
  else if (server.hasArg("go")) {
    currentCommand = server.arg("go");
    recordInput();
    exitIdle();
    server.send(200, "text/plain", "OK");
  } 
  else if (server.hasArg("stop")) {
    currentCommand = "";
    recordInput();
    server.send(200, "text/plain", "OK");
  }
  else if (server.hasArg("motor") && server.hasArg("value")) {
    int motorNum = server.arg("motor").toInt();
    int servoIdx = servoNameToIndex(server.arg("motor"));
    int angle = server.arg("value").toInt();
    if (motorNum >= 1 && motorNum <= 8 && angle >= 0 && angle <= 180) {
      setServoAngle(motorNum - 1, angle);
      recordInput();
      server.send(200, "text/plain", "OK");
    } else if (servoIdx != -1 && angle >= 0 && angle <= 180) {
      setServoAngle(servoIdx, angle);
      recordInput();
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "Invalid motor or angle");
    }
  }
  else if (server.hasArg("speed")) {
    int spd = server.arg("speed").toInt();
    if (spd >= 20 && spd <= 500) {
      frameDelay = spd;
      recordInput();
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "Invalid speed");
    }
  }
  else {
    server.send(400, "text/plain", "Bad Args");
  }
}

void handleGetSettings() {
  String json = "{";
  json += "\"frameDelay\":" + String(frameDelay) + ",";
  json += "\"walkCycles\":" + String(walkCycles) + ",";
  json += "\"motorCurrentDelay\":" + String(motorCurrentDelay) + ",";
  json += "}";
  server.send(200, "application/json", json);
}

void handleSetSettings() {
  if (server.hasArg("frameDelay")) frameDelay = server.arg("frameDelay").toInt();
  if (server.hasArg("walkCycles")) walkCycles = server.arg("walkCycles").toInt();
  if (server.hasArg("motorCurrentDelay")) motorCurrentDelay = server.arg("motorCurrentDelay").toInt();
  if (server.hasArg("faceFps")) faceFps = (int)max(1L, server.arg("faceFps").toInt());
  server.send(200, "text/plain", "OK");
}

void handleGetStatus() {
  String json = "{";
  json += "\"currentCommand\":\"" + currentCommand + "\",";
  json += "\"currentFace\":\"" + currentFaceName + "\",";
  json += "\"networkConnected\":" + String(networkConnected ? "true" : "false") + ",";
  json += "\"apIP\":\"" + WiFi.softAPIP().toString() + "\"";
  if (networkConnected) {
    json += ",\"networkIP\":\"" + networkIP.toString() + "\"";
  }
  json += "}";
  server.send(200, "application/json", json);
}

void handleApiCommand() {
  if (server.method() != HTTP_POST) {
    server.send(405, "application/json", "{\"error\":\"Method not allowed\"}");
    return;
  }
  
  String body = server.arg("plain");
  Serial.println("API Command received:");
  Serial.println(body);
  
  int faceOnlyStart = body.indexOf("\"face\":\"");
  if (faceOnlyStart == -1) faceOnlyStart = body.indexOf("\"face\": \"");
  
  bool faceOnly = (faceOnlyStart > 0 && body.indexOf("\"command\":") == -1 && body.indexOf("\"command\": ") == -1);
  
  String command = "";
  String face = "";
  
  if (faceOnlyStart > 0) {
    faceOnlyStart = body.indexOf("\"", faceOnlyStart + 6) + 1;
    int faceEnd = body.indexOf("\"", faceOnlyStart);
    if (faceEnd > faceOnlyStart) {
      face = body.substring(faceOnlyStart, faceEnd);
    }
  }
  
  if (!faceOnly) {
    int cmdStart = body.indexOf("\"command\":\"");
    if (cmdStart == -1) cmdStart = body.indexOf("\"command\": \"");
    if (cmdStart == -1) {
      server.send(400, "application/json", "{\"error\":\"Missing command field\"}");
      return;
    }
    cmdStart = body.indexOf("\"", cmdStart + 10) + 1;
    int cmdEnd = body.indexOf("\"", cmdStart);
    if (cmdEnd <= cmdStart) {
      server.send(400, "application/json", "{\"error\":\"Invalid command format\"}");
      return;
    }
    command = body.substring(cmdStart, cmdEnd);
  }
  
  if (face.length() > 0) setFace(face);
  
  if (faceOnly) {
    recordInput();
    server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Face updated\"}");
    return;
  }
  
  if (command == "stop") {
    currentCommand = "";
    recordInput();
    server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Command stopped\"}");
  } else {
    currentCommand = command;
    recordInput();
    exitIdle();
    server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Command executed\"}");
  }
}

void setup() {
  Serial.begin(115200);
  randomSeed(micros());
  
  Wire.begin(I2C_SDA, I2C_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
    Serial.println(F("SSD1306 allocation failed."));
    while (1);
  }
  
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println(F("Setting up WiFi..."));
  display.display();

  if (ENABLE_NETWORK_MODE && String(NETWORK_SSID).length() > 0) {
    Serial.println("Attempting to connect to network: " + String(NETWORK_SSID));
    WiFi.mode(WIFI_AP_STA);
    WiFi.setHostname(deviceHostname.c_str());
    WiFi.begin(NETWORK_SSID, NETWORK_PASS);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      networkConnected = true;
      networkIP = WiFi.localIP();
      Serial.println();
      Serial.print("Connected to network! IP: ");
      Serial.println(networkIP);
    } else {
      Serial.println();
      Serial.println("Failed to connect to network. Running in AP-only mode.");
      WiFi.mode(WIFI_AP);
    }
  } else {
    WiFi.mode(WIFI_AP);
    Serial.println("Network mode disabled. Running in AP-only mode.");
  }
  
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP Created. IP: ");
  Serial.println(myIP);

  if (networkConnected) {
    wifiInfoText = "AP: " + String(AP_SSID) + " (" + myIP.toString() + ")  |  Network: " + String(NETWORK_SSID) + " (" + networkIP.toString() + ") or " + deviceHostname + ".local  |  ";
  } else {
    wifiInfoText = "Connect to WiFi: " + String(AP_SSID) + "  |  Pass: " + String(AP_PASS) + "  |  IP: " + myIP.toString() + "  |  Captive Portal will auto-open!  |  ";
  }
  
  lastInputTime = millis();
  firstInputReceived = false;
  showingWifiInfo = false;

  if (MDNS.begin(deviceHostname.c_str())) {
    Serial.println("mDNS responder started");
    MDNS.addService("http", "tcp", 80);
  }

  dnsServer.start(DNS_PORT, "*", myIP);

  server.on("/", handleRoot);
  server.on("/cmd", handleCommandWeb);
  server.on("/getSettings", handleGetSettings);
  server.on("/setSettings", handleSetSettings);
  server.on("/api/status", handleGetStatus);
  server.on("/api/command", handleApiCommand);
  server.onNotFound(handleRoot);
  
  server.begin();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  for (int i = 0; i < 8; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(servoPins[i], 732, 2929);
  }
  delay(10);
  
  setFace("rest");
  
  Serial.println(F("HTTP server & Captive Portal started."));
}

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();
  updateAnimatedFace();
  updateIdleBlink();
  updateWifiInfoScroll();

  if (currentCommand != "") {
    String cmd = currentCommand;
    if (cmd == "forward") runWalkPose();
    else if (cmd == "backward") runWalkBackward();
    else if (cmd == "left") runTurnLeft();
    else if (cmd == "right") runTurnRight();
    else if (cmd == "rest") { runRestPose(); if (currentCommand == "rest") currentCommand = ""; }
    else if (cmd == "stand") { runStandPose(1); if (currentCommand == "stand") currentCommand = ""; }
    else if (cmd == "wave") runWavePose();
    else if (cmd == "dance") runDancePose();
    else if (cmd == "swim") runSwimPose();
    else if (cmd == "point") runPointPose();
    else if (cmd == "pushup") runPushupPose();
    else if (cmd == "bow") runBowPose();
    else if (cmd == "cute") runCutePose();
    else if (cmd == "freaky") runFreakyPose();
    else if (cmd == "worm") runWormPose();
    else if (cmd == "shake") runShakePose();
    else if (cmd == "shrug") runShrugPose();
    else if (cmd == "dead") runDeadPose();
    else if (cmd == "crab") runCrabPose();
  }
  
  if (Serial.available()) {
    static char command_buffer[32];
    static byte buffer_pos = 0;
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (buffer_pos > 0) {
        command_buffer[buffer_pos] = '\0';
        int motorNum, angle;
        recordInput();
        if(strcmp(command_buffer, "run walk") == 0 || strcmp(command_buffer, "rn wf") == 0) { currentCommand = "forward"; runWalkPose(); currentCommand = ""; }
        else if(strcmp(command_buffer, "rn wb") == 0) { currentCommand = "backward"; runWalkBackward(); currentCommand = ""; }
        else if(strcmp(command_buffer, "rn tl") == 0) { currentCommand = "left"; runTurnLeft(); currentCommand = ""; }
        else if(strcmp(command_buffer, "rn tr") == 0) { currentCommand = "right"; runTurnRight(); currentCommand = ""; }
        else if(strcmp(command_buffer, "run rest") == 0 || strcmp(command_buffer, "rn rs") == 0) runRestPose();
        else if(strcmp(command_buffer, "run stand") == 0 || strcmp(command_buffer, "rn st") == 0) runStandPose(1);
        else if(strcmp(command_buffer, "rn wv") == 0) { currentCommand = "wave"; runWavePose(); }
        else if(strcmp(command_buffer, "rn dn") == 0) { currentCommand = "dance"; runDancePose(); }
        else if(strcmp(command_buffer, "rn sw") == 0) { currentCommand = "swim"; runSwimPose(); }
        else if(strcmp(command_buffer, "rn pt") == 0) { currentCommand = "point"; runPointPose(); }
        else if(strcmp(command_buffer, "rn pu") == 0) { currentCommand = "pushup"; runPushupPose(); }
        else if(strcmp(command_buffer, "rn bw") == 0) { currentCommand = "bow"; runBowPose(); }
        else if(strcmp(command_buffer, "rn ct") == 0) { currentCommand = "cute"; runCutePose(); }
        else if(strcmp(command_buffer, "rn fk") == 0) { currentCommand = "freaky"; runFreakyPose(); }
        else if(strcmp(command_buffer, "rn wm") == 0) { currentCommand = "worm"; runWormPose(); }
        else if(strcmp(command_buffer, "rn sk") == 0) { currentCommand = "shake"; runShakePose(); }
        else if(strcmp(command_buffer, "rn sg") == 0) { currentCommand = "shrug"; runShrugPose(); }
        else if(strcmp(command_buffer, "rn dd") == 0) { currentCommand = "dead"; runDeadPose(); }
        else if(strcmp(command_buffer, "rn cb") == 0) { currentCommand = "crab"; runCrabPose(); }

        else if (strncmp(command_buffer, "all ", 4) == 0) {
             if (sscanf(command_buffer + 4, "%d", &angle) == 1) {
                 for (int i = 0; i < 8; i++) setServoAngle(i, angle);
                 Serial.print("All servos set to "); Serial.println(angle);
             }
        }
        else if (sscanf(command_buffer, "%d %d", &motorNum, &angle) == 2) {
             if (motorNum >= 0 && motorNum < 8) {
                 setServoAngle(motorNum, angle);
                 Serial.print("Servo "); Serial.print(motorNum); Serial.print(" set to "); Serial.println(angle);
             } else {
                 Serial.println("Invalid motor number (0-7)");
             }
        }
        buffer_pos = 0;
      }
    } else if (buffer_pos < sizeof(command_buffer) - 1) {
      command_buffer[buffer_pos++] = c;
    }
  }
}

void updateFaceBitmap(const unsigned char* bitmap) {
  display.clearDisplay();
  display.drawBitmap(0, 0, bitmap, 128, 64, SSD1306_WHITE);
  display.display();
}

uint8_t countFrames(const unsigned char* const* frames, uint8_t maxFrames) {
  if (frames == nullptr || frames[0] == nullptr) return 0;
  uint8_t count = 0;
  for (uint8_t i = 0; i < maxFrames; i++) {
    if (frames[i] == nullptr) break;
    count++;
  }
  return count;
}

void setFace(const String& faceName) {
  if (faceName == currentFaceName && currentFaceFrames != nullptr) return;
  currentFaceName = faceName;
  currentFaceFrameIndex = 0;
  lastFaceFrameMs = 0;
  faceFrameDirection = 1;
  faceAnimFinished = false;
  currentFaceFps = getFaceFpsForName(faceName);
  currentFaceFrames = face_defualt_frames;
  currentFaceFrameCount = countFrames(face_defualt_frames, MAX_FACE_FRAMES);
  for (size_t i = 0; i < (sizeof(faceEntries) / sizeof(faceEntries[0])); i++) {
    if (faceName.equalsIgnoreCase(faceEntries[i].name)) {
      currentFaceFrames = faceEntries[i].frames;
      currentFaceFrameCount = countFrames(faceEntries[i].frames, faceEntries[i].maxFrames);
      break;
    }
  }
  if (currentFaceFrameCount == 0) {
    currentFaceFrames = face_defualt_frames;
    currentFaceFrameCount = countFrames(face_defualt_frames, MAX_FACE_FRAMES);
    currentFaceName = "default";
    currentFaceFps = getFaceFpsForName(currentFaceName);
  }
  if (currentFaceFrameCount > 0 && currentFaceFrames[0] != nullptr) {
    updateFaceBitmap(currentFaceFrames[0]);
  }
}

void setFaceMode(FaceAnimMode mode) {
  currentFaceMode = mode;
  faceFrameDirection = 1;
  faceAnimFinished = false;
}

void setFaceWithMode(const String& faceName, FaceAnimMode mode) {
  setFaceMode(mode);
  setFace(faceName);
}

int getFaceFpsForName(const String& faceName) {
  for (size_t i = 0; i < (sizeof(faceFpsEntries) / sizeof(faceFpsEntries[0])); i++) {
    if (faceName.equalsIgnoreCase(faceFpsEntries[i].name)) {
      return faceFpsEntries[i].fps;
    }
  }
  return faceFps;
}

void updateAnimatedFace() {
  if (currentFaceFrames == nullptr || currentFaceFrameCount <= 1) return;
  if (currentFaceMode == FACE_ANIM_ONCE && faceAnimFinished) return;
  unsigned long now = millis();
  int fps = max(1, (currentFaceFps > 0 ? currentFaceFps : faceFps));
  unsigned long interval = 1000UL / fps;
  if (now - lastFaceFrameMs >= interval) {
    lastFaceFrameMs = now;
    if (currentFaceMode == FACE_ANIM_LOOP) {
      currentFaceFrameIndex = (currentFaceFrameIndex + 1) % currentFaceFrameCount;
    } else if (currentFaceMode == FACE_ANIM_ONCE) {
      if (currentFaceFrameIndex + 1 >= currentFaceFrameCount) {
        currentFaceFrameIndex = currentFaceFrameCount - 1;
        faceAnimFinished = true;
      } else {
        currentFaceFrameIndex++;
      }
    } else {
      if (faceFrameDirection > 0) {
        if (currentFaceFrameIndex + 1 >= currentFaceFrameCount) {
          faceFrameDirection = -1;
          if (currentFaceFrameIndex > 0) currentFaceFrameIndex--;
        } else {
          currentFaceFrameIndex++;
        }
      } else {
        if (currentFaceFrameIndex == 0) {
          faceFrameDirection = 1;
          if (currentFaceFrameCount > 1) currentFaceFrameIndex++;
        } else {
          currentFaceFrameIndex--;
        }
      }
    }
    updateFaceBitmap(currentFaceFrames[currentFaceFrameIndex]);
  }
}

void delayWithFace(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    updateAnimatedFace();
    server.handleClient();
    dnsServer.processNextRequest();
    delay(5);
  }
}

void scheduleNextIdleBlink(unsigned long minMs, unsigned long maxMs) {
  unsigned long now = millis();
  unsigned long interval = (unsigned long)random(minMs, maxMs);
  nextIdleBlinkMs = now + interval;
}

void enterIdle() {
  idleActive = true;
  idleBlinkActive = false;
  idleBlinkRepeatsLeft = 0;
  setFaceWithMode("idle", FACE_ANIM_BOOMERANG);
  scheduleNextIdleBlink(3000, 7000);
}

void exitIdle() {
  idleActive = false;
  idleBlinkActive = false;
}

void updateIdleBlink() {
  if (!idleActive) return;
  if (!idleBlinkActive) {
    if (millis() >= nextIdleBlinkMs) {
      idleBlinkActive = true;
      if (idleBlinkRepeatsLeft == 0 && random(0, 100) < 30) {
        idleBlinkRepeatsLeft = 1;
      }
      setFaceWithMode("idle_blink", FACE_ANIM_ONCE);
    }
    return;
  }
  if (currentFaceMode == FACE_ANIM_ONCE && faceAnimFinished) {
    idleBlinkActive = false;
    setFaceWithMode("idle", FACE_ANIM_BOOMERANG);
    if (idleBlinkRepeatsLeft > 0) {
      idleBlinkRepeatsLeft--;
      scheduleNextIdleBlink(120, 220);
    } else {
      scheduleNextIdleBlink(3000, 7000);
    }
  }
}

void setServoAngle(uint8_t channel, int angle) { 
  if (channel < 8) {
    servos[channel].write(angle);
    delayWithFace(motorCurrentDelay);
  }
}

bool pressingCheck(String cmd, int ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    server.handleClient();
    dnsServer.processNextRequest();
    updateAnimatedFace();
    if (currentCommand != cmd) {
      runStandPose(1);
      return false;
    }
    yield();
  }
  return true;
}

void recordInput() {
  lastInputTime = millis();
  if (!firstInputReceived) {
    firstInputReceived = true;
    showingWifiInfo = false;
  }
}

void updateWifiInfoScroll() {
  if (firstInputReceived) {
    if (showingWifiInfo) {
      showingWifiInfo = false;
      if (currentFaceFrames != nullptr && currentFaceFrameCount > 0) {
        updateFaceBitmap(currentFaceFrames[currentFaceFrameIndex]);
      }
    }
    return;
  }
  
  unsigned long now = millis();
  if (!showingWifiInfo && (now - lastInputTime >= 30000)) {
    showingWifiInfo = true;
    wifiScrollPos = 0;
    lastWifiScrollMs = now;
  }
  
  if (!showingWifiInfo) return;
  
  if (now - lastWifiScrollMs >= 150) {
    lastWifiScrollMs = now;
    display.clearDisplay();
    if (currentFaceFrames != nullptr && currentFaceFrameCount > 0) {
      display.drawBitmap(0, 0, currentFaceFrames[currentFaceFrameIndex], 128, 64, SSD1306_WHITE);
    }
    display.fillRect(0, 0, 128, 10, SSD1306_BLACK);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setTextWrap(false);
    display.setCursor(-wifiScrollPos, 1);
    display.print(wifiInfoText);
    display.setTextWrap(true);
    display.display();
    wifiScrollPos += 2;
    if (wifiScrollPos >= (int)(wifiInfoText.length() * 6)) {
      wifiScrollPos = 0;
    }
  }
}
