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
  Serial.println(F("REST (Calibrated)")); 
  setFaceWithMode("rest", FACE_ANIM_BOOMERANG); 
  setServoAngle(R1, 78);
  setServoAngle(R2, 82);
  setServoAngle(L1, 98);
  setServoAngle(L2, 81);
  setServoAngle(R4, 91);
  setServoAngle(R3, 87);
  setServoAngle(L3, 79);
  setServoAngle(L4, 83);
}

inline void runStandPose(int face) { 
  Serial.println(F("STAND (Calibrated)")); 
  if (face == 1) setFaceWithMode("stand", FACE_ANIM_ONCE); 
  setServoAngle(R1, 104); 
  setServoAngle(R2, 52); 
  setServoAngle(L1, 72); 
  setServoAngle(L2, 101); 
  setServoAngle(R4, 32); 
  setServoAngle(R3, 151); 
  setServoAngle(L3, 19); 
  setServoAngle(L4, 148); 
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

// --- CALIBRATED MOVEMENT GAITS (Physical Hardware Alignment & Ground Contact) ---
// Calibrated Stand Stance:
//   FR (R1: 104, R3: 151) | RR (R2: 52, R4: 32)
//   FL (L1: 72,  L3: 19)  | RL (L2: 101, L4: 148)
//
// Knee Contact Dynamics (Down towards floor):
//   - R3 (FR Knee): Down towards 180. Stand=151, Stance=154 (firm grip), Swing=112 (lifted in air)
//   - L4 (RL Knee): Down towards 180. Stand=148, Stance=152 (firm grip), Swing=110 (lifted in air)
//   - L3 (FL Knee): Down towards 0.   Stand=19,  Stance=15  (firm grip), Swing=62  (lifted in air)
//   - R4 (RR Knee): Down towards 0.   Stand=32,  Stance=28  (firm grip), Swing=72  (lifted in air)
//
// Hip Stroke Dynamics (Forward vs Backward):
//   - R1 (FR Hip): Forward towards 174 (Swing=126, Push=86)
//   - L2 (RL Hip): Forward towards 76  (Swing=84,  Push=118)
//   - L1 (FL Hip): Forward towards 5   (Swing=50,  Push=90)
//   - R2 (RR Hip): Forward towards 90  (Swing=70,  Push=36)

inline void runWalkPose() {
  Serial.println(F("WALK FWD (Calibrated)"));
  setFaceWithMode("walk", FACE_ANIM_ONCE);

  for (int i = 0; i < walkCycles; i++) {
    // --- Phase 1: Lift Pair 1 (FR: R1/R3 + RL: L2/L4), Pair 2 (FL + RR) drives forward ---
    setServoAngle(R3, 112); setServoAngle(L4, 110); // Lift Pair 1 knees
    setServoAngle(L3, 15);  setServoAngle(R4, 28);  // Pair 2 knees firmly on floor
    if (!pressingCheck("forward", frameDelay)) return;

    // Swing Pair 1 forward in air; push Pair 2 backward on ground to propel body forward
    setServoAngle(R1, 126); setServoAngle(L2, 84);  // Pair 1 swings forward
    setServoAngle(L1, 90);  setServoAngle(R2, 36);  // Pair 2 pushes backward on floor
    if (!pressingCheck("forward", frameDelay)) return;

    // Touchdown Pair 1 firmly on floor
    setServoAngle(R3, 154); setServoAngle(L4, 152);
    if (!pressingCheck("forward", frameDelay)) return;

    // --- Phase 2: Lift Pair 2 (FL: L1/L3 + RR: R2/R4), Pair 1 (FR + RL) drives forward ---
    setServoAngle(L3, 62);  setServoAngle(R4, 72);  // Lift Pair 2 knees
    setServoAngle(R3, 154); setServoAngle(L4, 152); // Pair 1 knees firmly on floor
    if (!pressingCheck("forward", frameDelay)) return;

    // Swing Pair 2 forward in air; push Pair 1 backward on ground to propel body forward
    setServoAngle(L1, 50);  setServoAngle(R2, 70);  // Pair 2 swings forward
    setServoAngle(R1, 86);  setServoAngle(L2, 118); // Pair 1 pushes backward on floor
    if (!pressingCheck("forward", frameDelay)) return;

    // Touchdown Pair 2 firmly on floor
    setServoAngle(L3, 15);  setServoAngle(R4, 28);
    if (!pressingCheck("forward", frameDelay)) return;
  }
  runStandPose(1);
  if (currentCommand == "forward") currentCommand = "";
}

inline void runWalkBackward() {
  Serial.println(F("WALK BACK (Calibrated)"));
  setFaceWithMode("walk", FACE_ANIM_ONCE);

  for (int i = 0; i < walkCycles; i++) {
    // --- Phase 1: Lift Pair 1, swing backward in air; push Pair 2 forward on floor ---
    setServoAngle(R3, 112); setServoAngle(L4, 110);
    setServoAngle(L3, 15);  setServoAngle(R4, 28);
    if (!pressingCheck("backward", frameDelay)) return;

    setServoAngle(R1, 86);  setServoAngle(L2, 118); // Swing backward in air
    setServoAngle(L1, 50);  setServoAngle(R2, 70);  // Push forward on floor
    if (!pressingCheck("backward", frameDelay)) return;

    setServoAngle(R3, 154); setServoAngle(L4, 152); // Plant on floor
    if (!pressingCheck("backward", frameDelay)) return;

    // --- Phase 2: Lift Pair 2, swing backward in air; push Pair 1 forward on floor ---
    setServoAngle(L3, 62);  setServoAngle(R4, 72);
    setServoAngle(R3, 154); setServoAngle(L4, 152);
    if (!pressingCheck("backward", frameDelay)) return;

    setServoAngle(L1, 90);  setServoAngle(R2, 36);  // Swing backward in air
    setServoAngle(R1, 126); setServoAngle(L2, 84);  // Push forward on floor
    if (!pressingCheck("backward", frameDelay)) return;

    setServoAngle(L3, 15);  setServoAngle(R4, 28);  // Plant on floor
    if (!pressingCheck("backward", frameDelay)) return;
  }
  runStandPose(1);
  if (currentCommand == "backward") currentCommand = "";
}

inline void runTurnLeft() {
  Serial.println(F("TURN LEFT (Calibrated Step & Twist)"));
  setFaceWithMode("walk", FACE_ANIM_ONCE);

  for (int i = 0; i < walkCycles; i++) {
    // --- Phase 1: Legset 1 (FR: R1/R3 + RL: L2/L4). Legset 2 firmly on floor ---
    setServoAngle(R3, 112); setServoAngle(L4, 110); // Lift Pair 1
    setServoAngle(L3, 15);  setServoAngle(R4, 28);  // Pair 2 on floor
    if (!pressingCheck("left", frameDelay)) return;

    // In air: FR swings forward, RL swings backward
    setServoAngle(R1, 126); setServoAngle(L2, 118);
    if (!pressingCheck("left", frameDelay)) return;

    // Touchdown Pair 1 on floor
    setServoAngle(R3, 154); setServoAngle(L4, 152);
    if (!pressingCheck("left", frameDelay)) return;

    // Rotate body by pulling hips back to stand pose while feet are on floor
    setServoAngle(R1, 104); setServoAngle(L2, 101);
    if (!pressingCheck("left", frameDelay)) return;

    // --- Phase 2: Legset 2 (FL: L1/L3 + RR: R2/R4). Legset 1 firmly on floor ---
    setServoAngle(L3, 62);  setServoAngle(R4, 72);  // Lift Pair 2
    setServoAngle(R3, 154); setServoAngle(L4, 152); // Pair 1 on floor
    if (!pressingCheck("left", frameDelay)) return;

    // In air: RR swings forward, FL swings backward
    setServoAngle(R2, 70);  setServoAngle(L1, 90);
    if (!pressingCheck("left", frameDelay)) return;

    // Touchdown Pair 2 on floor
    setServoAngle(L3, 15);  setServoAngle(R4, 28);
    if (!pressingCheck("left", frameDelay)) return;

    // Rotate body by pulling hips back to stand pose while feet are on floor
    setServoAngle(R2, 52);  setServoAngle(L1, 72);
    if (!pressingCheck("left", frameDelay)) return;
  }
  runStandPose(1);
  if (currentCommand == "left") currentCommand = "";
}

inline void runTurnRight() {
  Serial.println(F("TURN RIGHT (Calibrated Step & Twist)"));
  setFaceWithMode("walk", FACE_ANIM_ONCE);

  for (int i = 0; i < walkCycles; i++) {
    // --- Phase 1: Legset 2 (FL: L1/L3 + RR: R2/R4). Legset 1 firmly on floor ---
    setServoAngle(L3, 62);  setServoAngle(R4, 72);  // Lift Pair 2
    setServoAngle(R3, 154); setServoAngle(L4, 152); // Pair 1 on floor
    if (!pressingCheck("right", frameDelay)) return;

    // In air: FL swings forward, RR swings backward
    setServoAngle(L1, 50);  setServoAngle(R2, 36);
    if (!pressingCheck("right", frameDelay)) return;

    // Touchdown Pair 2 on floor
    setServoAngle(L3, 15);  setServoAngle(R4, 28);
    if (!pressingCheck("right", frameDelay)) return;

    // Rotate body by pulling hips back to stand pose while feet are on floor
    setServoAngle(L1, 72);  setServoAngle(R2, 52);
    if (!pressingCheck("right", frameDelay)) return;

    // --- Phase 2: Legset 1 (FR: R1/R3 + RL: L2/L4). Legset 2 firmly on floor ---
    setServoAngle(R3, 112); setServoAngle(L4, 110); // Lift Pair 1
    setServoAngle(L3, 15);  setServoAngle(R4, 28);  // Pair 2 on floor
    if (!pressingCheck("right", frameDelay)) return;

    // In air: FR swings backward, RL swings forward
    setServoAngle(R1, 86);  setServoAngle(L2, 84);
    if (!pressingCheck("right", frameDelay)) return;

    // Touchdown Pair 1 on floor
    setServoAngle(R3, 154); setServoAngle(L4, 152);
    if (!pressingCheck("right", frameDelay)) return;

    // Rotate body by pulling hips back to stand pose while feet are on floor
    setServoAngle(R1, 104); setServoAngle(L2, 101);
    if (!pressingCheck("right", frameDelay)) return;
  }
  runStandPose(1);
  if (currentCommand == "right") currentCommand = "";
}
