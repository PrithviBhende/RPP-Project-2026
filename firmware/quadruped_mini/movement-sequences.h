#pragma once

#include <Arduino.h>

enum ServoName : uint8_t {
  R1 = 0, 
  R2 = 1,
  L1 = 2,
  L2 = 3,
  R4 = 4,
  R3 = 5,
  L3 = 6,
  L4 = 7
};

const String ServoNames[]={"R1","R2","L1","L2","R4","R3","L3","L4"};

inline int servoNameToIndex(const String& servo) {
  if (servo == "L1") return L1;
  if (servo == "L2") return L2;
  if (servo == "L3") return L3;
  if (servo == "L4") return L4;
  if (servo == "R1") return R1;
  if (servo == "R2") return R2;
  if (servo == "R3") return R3;
  if (servo == "R4") return R4;
  return -1;
}

enum FaceAnimMode : uint8_t {
  FACE_ANIM_LOOP = 0,
  FACE_ANIM_ONCE = 1,
  FACE_ANIM_BOOMERANG = 2
};

// External globals and helpers used by movement/pose sequences
extern int frameDelay;
extern int walkCycles;
extern String currentCommand;

extern void setServoAngle(uint8_t channel, int angle);
extern void setFace(const String& faceName);
extern void setFaceMode(FaceAnimMode mode);
extern void setFaceWithMode(const String& faceName, FaceAnimMode mode);
extern void delayWithFace(unsigned long ms);
extern void enterIdle();
extern bool pressingCheck(String cmd, int ms);

// Pose/animation prototypes
void runRestPose();
void runStandPose(int face = 1);
void runWavePose();
void runDancePose();
void runSwimPose();
void runPointPose();
void runPushupPose();
void runBowPose();
void runCutePose();
void runFreakyPose();
void runWormPose();
void runShakePose();
void runShrugPose();
void runDeadPose();
void runCrabPose();
void runWalkPose();
void runWalkBackward();
void runTurnLeft();
void runTurnRight();

// ====== POSES ======
inline void runRestPose() { 
  Serial.println(F("REST")); 
  setFaceWithMode("rest", FACE_ANIM_BOOMERANG); 
  setServoAngle(R1, 78);
  setServoAngle(R2, 80);
  setServoAngle(L1, 98);
  setServoAngle(L2, 80);
  setServoAngle(R4, 90);
  setServoAngle(R3, 84);
  setServoAngle(L3, 80);
  setServoAngle(L4, 77);
}

inline void runStandPose(int face) { 
  Serial.println(F("STAND")); 
  if (face == 1) setFaceWithMode("stand", FACE_ANIM_ONCE); 
  setServoAngle(R1, 115); 
  setServoAngle(R2, 50); 
  setServoAngle(L1, 65); 
  setServoAngle(L2, 110); 
  setServoAngle(R4, 20); 
  setServoAngle(R3, 156); 
  setServoAngle(L3, 10); 
  setServoAngle(L4, 150); 
  if (face == 1) enterIdle();
}

inline void runWavePose() { 
  Serial.println(F("WAVE")); 
  setFaceWithMode("wave", FACE_ANIM_ONCE); 
  setServoAngle(R1, 87);
  setServoAngle(R2, 45);
  setServoAngle(L1, 49);
  setServoAngle(L2, 94);
  setServoAngle(R4, 56);
  setServoAngle(R3, 165);
  setServoAngle(L4, 149);
  delayWithFace(200);
  setServoAngle(L3, 150); 
  delayWithFace(200);
  setServoAngle(L3, 150); 
  delayWithFace(300); 
  for (int i = 0; i < 4; i++) { 
    setServoAngle(L3, 150); delayWithFace(300); 
    setServoAngle(L3, 88); delayWithFace(300); 
  } 
  runStandPose(1); 
  if (currentCommand == "wave") currentCommand = "";
}

inline void runDancePose() { 
  Serial.println(F("DANCE")); 
  setFaceWithMode("dance", FACE_ANIM_LOOP); 
  setServoAngle(R1, 122); setServoAngle(R2, 50); 
  setServoAngle(L1, 67); setServoAngle(L2, 120); 
  setServoAngle(R4, 145); setServoAngle(R3, 140); 
  setServoAngle(L3, 18); setServoAngle(L4, 12); 
  delayWithFace(300); 
  for (int i = 0; i < 5; i++) { 
    setServoAngle(R4, 110); setServoAngle(R3, 104); 
    setServoAngle(L3, 18); setServoAngle(L4, 12); 
    delayWithFace(300); 
    setServoAngle(R4, 145); setServoAngle(R3, 140); 
    setServoAngle(L3, 61); setServoAngle(L4, 57); 
    delayWithFace(300); 
  } 
  runStandPose(1); 
  if (currentCommand == "dance") currentCommand = "";
}

inline void runSwimPose() { 
  Serial.println(F("SWIM")); 
  setFaceWithMode("swim", FACE_ANIM_ONCE); 
  setServoAngle(R4, 135); setServoAngle(R3, 35);
  setServoAngle(L3, 135); setServoAngle(L4, 32);
  setServoAngle(R1, 78); setServoAngle(R2, 80);
  setServoAngle(L1, 98); setServoAngle(L2, 80);
  for (int i = 0; i < 4; i++) { 
    setServoAngle(R1, 115); setServoAngle(R2, 50); 
    setServoAngle(L1, 65); setServoAngle(L2, 110); 
    delayWithFace(400); 
    setServoAngle(R1, 78); setServoAngle(R2, 80); 
    setServoAngle(L1, 98); setServoAngle(L2, 80); 
    delayWithFace(400); 
  } 
  runStandPose(1); 
  if (currentCommand == "swim") currentCommand = "";
}

inline void runPointPose() { 
  Serial.println(F("POINT")); 
  setFaceWithMode("point", FACE_ANIM_BOOMERANG); 
  setServoAngle(R1, 87);
  setServoAngle(R2, 45);
  setServoAngle(L1, 49);
  setServoAngle(L2, 94);
  setServoAngle(R4, 92);
  setServoAngle(R3, 165);
  setServoAngle(L3, 144);
  setServoAngle(L4, 149);
  delayWithFace(2000); 
  runStandPose(1); 
  if (currentCommand == "point") currentCommand = "";
}

inline void runPushupPose() {
  Serial.println(F("PUSHUP"));
  setFaceWithMode("pushup", FACE_ANIM_ONCE);
  setServoAngle(R1, 122);
  setServoAngle(R2, 43);
  setServoAngle(L1, 62);
  setServoAngle(L2, 129);
  setServoAngle(R4, 53);
  setServoAngle(L4, 114);
  setServoAngle(L3, 80);
  setServoAngle(R3, 84);
  delayWithFace(700);
  for (int i = 0; i < 4; i++) {
    setServoAngle(L3, 10);
    setServoAngle(R3, 156);
    delayWithFace(600);
    setServoAngle(L3, 80);
    setServoAngle(R3, 84);
    delayWithFace(500);
  }
  runStandPose(1);
  if (currentCommand == "pushup") currentCommand = "";
}

inline void runBowPose() {
  Serial.println(F("BOW"));
  setFaceWithMode("bow", FACE_ANIM_ONCE);
  setServoAngle(R1, 134);
  setServoAngle(R2, 35);
  setServoAngle(L1, 43);
  setServoAngle(L2, 133);
  setServoAngle(R4, 85);
  setServoAngle(L4, 90);
  setServoAngle(R3, 20);
  setServoAngle(L3, 144);
  delayWithFace(800);
  setServoAngle(L3, 80);
  setServoAngle(R3, 84);
  delayWithFace(3000);
  runStandPose(1);
  if (currentCommand == "bow") currentCommand = "";
}

inline void runCutePose() {
  Serial.println(F("CUTE"));
  setFaceWithMode("cute", FACE_ANIM_ONCE);
  runStandPose(0); 
  delayWithFace(200);
  setServoAngle(L2, 127);
  setServoAngle(R2, 33);
  setServoAngle(R4, 160);
  setServoAngle(L4, 4);

  setServoAngle(L1, 32);
  setServoAngle(R1, 152);
  setServoAngle(L3, 150);
  setServoAngle(R3, 12);
  delayWithFace(200);
  for (int i = 0; i < 5; i++) {
    setServoAngle(R4, 160);
    setServoAngle(L4, 41);
    delayWithFace(300);
    setServoAngle(R4, 125);
    setServoAngle(L4, 4);
    delayWithFace(300);
  }
  runStandPose(1);
  if (currentCommand == "cute") currentCommand = "";
}

inline void runFreakyPose() {
  Serial.println(F("FREAKY"));
  setFaceWithMode("freaky", FACE_ANIM_ONCE);
  runStandPose(0); 
  delayWithFace(200);
  setServoAngle(L1, 32);
  setServoAngle(R1, 152);
  setServoAngle(L2, 140);
  setServoAngle(R2, 20);
  setServoAngle(R4, 90);
  setServoAngle(R3, 12);
  delayWithFace(200);
  for (int i = 0; i < 3; i++) {
    setServoAngle(R3, 32);
    delayWithFace(400);
    setServoAngle(R3, 12);
    delayWithFace(400);
  }
  runStandPose(1);
  if (currentCommand == "freaky") currentCommand = "";
}

inline void runWormPose() {
  Serial.println(F("WORM"));
  setFaceWithMode("worm", FACE_ANIM_ONCE);
  runStandPose(0);
  delayWithFace(200);
  setServoAngle(R1, 152); setServoAngle(R2, 20); setServoAngle(L1, 32); setServoAngle(L2, 140);
  setServoAngle(R4, 90); setServoAngle(R3, 84); setServoAngle(L3, 80); setServoAngle(L4, 77);
  delayWithFace(200);
  for(int i=0; i<5; i++) {
    setServoAngle(R3, 48); setServoAngle(L3, 115); setServoAngle(R4, 55); setServoAngle(L4, 113);
    delayWithFace(300);
    setServoAngle(R3, 120); setServoAngle(L3, 45); setServoAngle(R4, 125); setServoAngle(L4, 41);
    delayWithFace(300);
  }
  runStandPose(1);
  if (currentCommand == "worm") currentCommand = "";
}

inline void runShakePose() {
  Serial.println(F("SHAKE"));
  setFaceWithMode("shake", FACE_ANIM_ONCE);
  runStandPose(0);
  delayWithFace(200);
  setServoAngle(R1, 115); setServoAngle(L1, 65); setServoAngle(L3, 80); setServoAngle(R3, 84);
  setServoAngle(L2, 80); setServoAngle(R2, 80);
  delayWithFace(200);
  for(int i=0; i<5; i++) {
    setServoAngle(R4, 55); setServoAngle(L4, 113);
    delayWithFace(300);
    setServoAngle(R4, 20); setServoAngle(L4, 150);
    delayWithFace(300);
  }
  runStandPose(1);
  if (currentCommand == "shake") currentCommand = "";
}

inline void runShrugPose() {
  Serial.println(F("SHRUG"));
  runStandPose(0);
  setFaceWithMode("dead", FACE_ANIM_ONCE);
  delayWithFace(200);
  setServoAngle(R3, 84); setServoAngle(R4, 90); setServoAngle(L3, 80); setServoAngle(L4, 77);
  delayWithFace(1000);
  setFaceWithMode("shrug", FACE_ANIM_ONCE);
  setServoAngle(R3, 12); setServoAngle(R4, 160); setServoAngle(L3, 150); setServoAngle(L4, 4);
  delayWithFace(1500);
  runStandPose(1);
  if (currentCommand == "shrug") currentCommand = "";
}

inline void runDeadPose() {
  Serial.println(F("DEAD"));
  runStandPose(0);
  setFaceWithMode("dead", FACE_ANIM_BOOMERANG);
  delayWithFace(200);
  setServoAngle(R3, 84); setServoAngle(R4, 90); setServoAngle(L3, 80); setServoAngle(L4, 77);
  if (currentCommand == "dead") currentCommand = "";
}

inline void runCrabPose() {
  Serial.println(F("CRAB"));
  setFaceWithMode("crab", FACE_ANIM_ONCE);
  runStandPose(0);
  delayWithFace(200);
  setServoAngle(R1, 78); setServoAngle(R2, 80); setServoAngle(L1, 98); setServoAngle(L2, 80);
  setServoAngle(R4, 20); setServoAngle(R3, 156); setServoAngle(L3, 45); setServoAngle(L4, 113);
  for(int i=0; i<5; i++) {
    setServoAngle(R4, 55); setServoAngle(R3, 120); setServoAngle(L3, 10); setServoAngle(L4, 150);
    delayWithFace(300);
    setServoAngle(R4, 20); setServoAngle(R3, 156); setServoAngle(L3, 45); setServoAngle(L4, 113);
    delayWithFace(300);
  }
  runStandPose(1);
  if (currentCommand == "crab") currentCommand = "";
}

// --- MOVEMENT ANIMATIONS ---
inline void runWalkPose() {
  Serial.println(F("WALK FWD"));
  setFaceWithMode("walk", FACE_ANIM_ONCE);
  // Initial Step
  setServoAngle(R3, 120); setServoAngle(L3, 45);
  setServoAngle(R2, 87); setServoAngle(L1, 51);
  if (!pressingCheck("forward", frameDelay)) return;
  
  for (int i = 0; i < walkCycles; i++) {
    setServoAngle(R3, 120); setServoAngle(L3, 10);
    if (!pressingCheck("forward", frameDelay)) return;
    setServoAngle(L4, 113); setServoAngle(L2, 80);
    setServoAngle(R4, 20); setServoAngle(R1, 152);
    if (!pressingCheck("forward", frameDelay)) return;    
    setServoAngle(R2, 50); setServoAngle(L1, 98);
    if (!pressingCheck("forward", frameDelay)) return;
    setServoAngle(R4, 55); setServoAngle(L4, 150);
    if (!pressingCheck("forward", frameDelay)) return;
    setServoAngle(R3, 156); setServoAngle(L3, 45);
    setServoAngle(R2, 80); setServoAngle(L1, 32);
    if (!pressingCheck("forward", frameDelay)) return;  
    setServoAngle(L2, 110); setServoAngle(R1, 78);
    if (!pressingCheck("forward", frameDelay)) return;
  }
  runStandPose(1);
}

// Logic reversed from Walk
inline void runWalkBackward() {
  Serial.println(F("WALK BACK"));
  setFaceWithMode("walk", FACE_ANIM_ONCE);
  if (!pressingCheck("backward", frameDelay)) return;
  
  for (int i = 0; i < walkCycles; i++) {
    setServoAngle(R3, 120); setServoAngle(L3, 10);
    if (!pressingCheck("backward", frameDelay)) return;
    setServoAngle(L4, 113); setServoAngle(L2, 110);
    setServoAngle(R4, 20); setServoAngle(R1, 78);
    if (!pressingCheck("backward", frameDelay)) return;    
    setServoAngle(R2, 80); setServoAngle(L1, 32);
    if (!pressingCheck("backward", frameDelay)) return;
    setServoAngle(R4, 55); setServoAngle(L4, 150);
    if (!pressingCheck("backward", frameDelay)) return;
    setServoAngle(R3, 156); setServoAngle(L3, 45);
    setServoAngle(R2, 50); setServoAngle(L1, 98);
    if (!pressingCheck("backward", frameDelay)) return;  
    setServoAngle(L2, 80); setServoAngle(R1, 152);
    if (!pressingCheck("backward", frameDelay)) return;
  }
  runStandPose(1);
}

// Simple turn logic
inline void runTurnLeft() {
  Serial.println(F("TURN LEFT"));
  setFaceWithMode("walk", FACE_ANIM_ONCE);
  for (int i = 0; i < walkCycles; i++) {
    //legset 1 (R1 L2)
    setServoAngle(R3, 120); setServoAngle(L4, 113); 
    if (!pressingCheck("left", frameDelay)) return;
    setServoAngle(R1, 152); setServoAngle(L2, 140); 
    if (!pressingCheck("left", frameDelay)) return;
    setServoAngle(R3, 156); setServoAngle(L4, 150); 
    if (!pressingCheck("left", frameDelay)) return;
    setServoAngle(R1, 115); setServoAngle(L2, 110);
    if (!pressingCheck("left", frameDelay)) return;
      //legset 2 (R2 L1)
    setServoAngle(R4, 55); setServoAngle(L3, 45); 
    if (!pressingCheck("left", frameDelay)) return;
    setServoAngle(R2, 80); setServoAngle(L1, 98); 
    if (!pressingCheck("left", frameDelay)) return;
    setServoAngle(R4, 20); setServoAngle(L3, 10); 
    if (!pressingCheck("left", frameDelay)) return;
    setServoAngle(R2, 50); setServoAngle(L1, 65);
    if (!pressingCheck("left", frameDelay)) return;  
  }
  runStandPose(1);
}

inline void runTurnRight() {
  Serial.println(F("TURN RIGHT"));
  setFaceWithMode("walk", FACE_ANIM_ONCE);
  for (int i = 0; i < walkCycles; i++) {
    //legset 2 (R2 L1)
    setServoAngle(R4, 55); setServoAngle(L3, 45); 
    if (!pressingCheck("right", frameDelay)) return;
    setServoAngle(R2, 20); setServoAngle(L1, 32); 
    if (!pressingCheck("right", frameDelay)) return;
    setServoAngle(R4, 20); setServoAngle(L3, 10); 
    if (!pressingCheck("right", frameDelay)) return;
    setServoAngle(R2, 50); setServoAngle(L1, 65);
    if (!pressingCheck("right", frameDelay)) return;  
    //legset 1 (R1 L2)
    setServoAngle(R3, 120); setServoAngle(L4, 113); 
    if (!pressingCheck("right", frameDelay)) return;
    setServoAngle(R1, 78); setServoAngle(L2, 80); 
    if (!pressingCheck("right", frameDelay)) return;
    setServoAngle(R3, 156); setServoAngle(L4, 150); 
    if (!pressingCheck("right", frameDelay)) return;
    setServoAngle(R1, 115); setServoAngle(L2, 110);
    if (!pressingCheck("right", frameDelay)) return;
  }
  runStandPose(1);
}
