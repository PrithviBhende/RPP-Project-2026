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
  Serial.println(F("REST (Canonical 90deg)")); 
  setFaceWithMode("rest", FACE_ANIM_BOOMERANG); 
  setServoAngle(R1, 90);
  setServoAngle(R2, 90);
  setServoAngle(L1, 90);
  setServoAngle(L2, 90);
  setServoAngle(R4, 90);
  setServoAngle(R3, 90);
  setServoAngle(L3, 90);
  setServoAngle(L4, 90);
}

inline void runStandPose(int face) { 
  Serial.println(F("STAND (Canonical)")); 
  if (face == 1) setFaceWithMode("stand", FACE_ANIM_ONCE); 
  setServoAngle(R1, 135); 
  setServoAngle(R2, 45); 
  setServoAngle(L1, 45); 
  setServoAngle(L2, 135); 
  setServoAngle(R4, 0); 
  setServoAngle(R3, 180); 
  setServoAngle(L3, 0); 
  setServoAngle(L4, 180); 
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

// --- CANONICAL MOVEMENT ANIMATIONS (Official Kinematics & Stance Floor Grounding) ---
// Base Stand: R1=135, R2=45, L1=45, L2=135, R4=0, R3=180, L3=0, L4=180
// Stance feet: R3=180, L4=180, R4=0, L3=0 (planted flat on floor at z = 0).
// Swing feet: R3=140, L4=140, R4=40, L3=40 (lifted 40 deg in air).

inline void runWalkPose() {
  Serial.println(F("WALK FWD (Canonical)"));
  setFaceWithMode("walk", FACE_ANIM_ONCE);

  for (int i = 0; i < walkCycles; i++) {
    // --- Phase 1: Lift Pair 1 (FR: R1/R3 + RL: L2/L4), Pair 2 (FL + RR) on floor ---
    setServoAngle(R3, 140); setServoAngle(L4, 140); // Lift Pair 1 knees
    setServoAngle(R4, 0);   setServoAngle(L3, 0);   // Pair 2 knees planted on floor
    if (!pressingCheck("forward", frameDelay)) return;

    // Swing Pair 1 forward in air; push Pair 2 backward on ground to propel body forward
    setServoAngle(R1, 110); setServoAngle(L2, 110); // Swing forward
    setServoAngle(L1, 20);  setServoAngle(R2, 20);  // Push backward on floor
    if (!pressingCheck("forward", frameDelay)) return;

    // Touchdown Pair 1 firmly on floor
    setServoAngle(R3, 180); setServoAngle(L4, 180);
    if (!pressingCheck("forward", frameDelay)) return;

    // --- Phase 2: Lift Pair 2 (FL: L1/L3 + RR: R2/R4), Pair 1 (FR + RL) on floor ---
    setServoAngle(R4, 40);  setServoAngle(L3, 40);  // Lift Pair 2 knees
    setServoAngle(R3, 180); setServoAngle(L4, 180); // Pair 1 knees planted on floor
    if (!pressingCheck("forward", frameDelay)) return;

    // Swing Pair 2 forward in air; push Pair 1 backward on ground to propel body forward
    setServoAngle(L1, 70);  setServoAngle(R2, 70);  // Swing forward
    setServoAngle(R1, 160); setServoAngle(L2, 160); // Push backward on floor
    if (!pressingCheck("forward", frameDelay)) return;

    // Touchdown Pair 2 firmly on floor
    setServoAngle(R4, 0);   setServoAngle(L3, 0);
    if (!pressingCheck("forward", frameDelay)) return;
  }
  runStandPose(1);
  if (currentCommand == "forward") currentCommand = "";
}

inline void runWalkBackward() {
  Serial.println(F("WALK BACK (Canonical)"));
  setFaceWithMode("walk", FACE_ANIM_ONCE);

  for (int i = 0; i < walkCycles; i++) {
    // --- Phase 1: Lift Pair 1, swing backward in air; push Pair 2 forward on floor ---
    setServoAngle(R3, 140); setServoAngle(L4, 140);
    setServoAngle(R4, 0);   setServoAngle(L3, 0);
    if (!pressingCheck("backward", frameDelay)) return;

    setServoAngle(R1, 160); setServoAngle(L2, 160); // Swing backward in air
    setServoAngle(L1, 70);  setServoAngle(R2, 70);  // Push forward on floor
    if (!pressingCheck("backward", frameDelay)) return;

    setServoAngle(R3, 180); setServoAngle(L4, 180); // Plant on floor
    if (!pressingCheck("backward", frameDelay)) return;

    // --- Phase 2: Lift Pair 2, swing backward in air; push Pair 1 forward on floor ---
    setServoAngle(R4, 40);  setServoAngle(L3, 40);
    setServoAngle(R3, 180); setServoAngle(L4, 180);
    if (!pressingCheck("backward", frameDelay)) return;

    setServoAngle(L1, 20);  setServoAngle(R2, 20);  // Swing backward in air
    setServoAngle(R1, 110); setServoAngle(L2, 110); // Push forward on floor
    if (!pressingCheck("backward", frameDelay)) return;

    setServoAngle(R4, 0);   setServoAngle(L3, 0);   // Plant on floor
    if (!pressingCheck("backward", frameDelay)) return;
  }
  runStandPose(1);
  if (currentCommand == "backward") currentCommand = "";
}

inline void runTurnLeft() {
  Serial.println(F("TURN LEFT (Canonical Floor Grounded)"));
  setFaceWithMode("walk", FACE_ANIM_ONCE);

  for (int i = 0; i < walkCycles; i++) {
    // --- Phase 1: Lift Pair 1 (R1/R3 + L2/L4). Stance Pair 2 (R4=0, L3=0) locked on floor! ---
    setServoAngle(R3, 140); setServoAngle(L4, 140); // Lift Pair 1
    setServoAngle(R4, 0);   setServoAngle(L3, 0);   // Ensure Pair 2 flat on floor
    if (!pressingCheck("left", frameDelay)) return;

    // Right leg forward, Left leg backward
    setServoAngle(R1, 110); setServoAngle(L2, 160); // In air: FR forward, RL backward
    setServoAngle(R2, 20);  setServoAngle(L1, 70);  // On ground: RR pushes, FL pushes
    if (!pressingCheck("left", frameDelay)) return;

    // Touchdown Pair 1 on floor!
    setServoAngle(R3, 180); setServoAngle(L4, 180);
    if (!pressingCheck("left", frameDelay)) return;

    // --- Phase 2: Lift Pair 2 (R2/R4 + L1/L3). Stance Pair 1 (R3=180, L4=180) locked on floor! ---
    setServoAngle(R4, 40);  setServoAngle(L3, 40);  // Lift Pair 2
    setServoAngle(R3, 180); setServoAngle(L4, 180); // Ensure Pair 1 flat on floor
    if (!pressingCheck("left", frameDelay)) return;

    // Right leg forward, Left leg backward
    setServoAngle(R2, 70);  setServoAngle(L1, 20);  // In air: RR forward, FL backward
    setServoAngle(R1, 160); setServoAngle(L2, 110); // On ground: FR pushes, RL pushes
    if (!pressingCheck("left", frameDelay)) return;

    // Touchdown Pair 2 on floor!
    setServoAngle(R4, 0);   setServoAngle(L3, 0);
    if (!pressingCheck("left", frameDelay)) return;
  }
  runStandPose(1);
  if (currentCommand == "left") currentCommand = "";
}

inline void runTurnRight() {
  Serial.println(F("TURN RIGHT (Canonical Floor Grounded)"));
  setFaceWithMode("walk", FACE_ANIM_ONCE);

  for (int i = 0; i < walkCycles; i++) {
    // --- Phase 1: Lift Pair 1 (R1/R3 + L2/L4). Stance Pair 2 (R4=0, L3=0) locked on floor! ---
    setServoAngle(R3, 140); setServoAngle(L4, 140); // Lift Pair 1
    setServoAngle(R4, 0);   setServoAngle(L3, 0);   // Ensure Pair 2 flat on floor
    if (!pressingCheck("right", frameDelay)) return;

    // Right leg backward, Left leg forward
    setServoAngle(R1, 160); setServoAngle(L2, 110); // In air: FR backward, RL forward
    setServoAngle(R2, 70);  setServoAngle(L1, 20);  // On ground: RR pushes, FL pushes
    if (!pressingCheck("right", frameDelay)) return;

    // Touchdown Pair 1 on floor!
    setServoAngle(R3, 180); setServoAngle(L4, 180);
    if (!pressingCheck("right", frameDelay)) return;

    // --- Phase 2: Lift Pair 2 (R2/R4 + L1/L3). Stance Pair 1 (R3=180, L4=180) locked on floor! ---
    setServoAngle(R4, 40);  setServoAngle(L3, 40);  // Lift Pair 2
    setServoAngle(R3, 180); setServoAngle(L4, 180); // Ensure Pair 1 flat on floor
    if (!pressingCheck("right", frameDelay)) return;

    // Right leg backward, Left leg forward
    setServoAngle(R2, 20);  setServoAngle(L1, 70);  // In air: RR backward, FL forward
    setServoAngle(R1, 110); setServoAngle(L2, 160); // On ground: FR pushes, RL pushes
    if (!pressingCheck("right", frameDelay)) return;

    // Touchdown Pair 2 on floor!
    setServoAngle(R4, 0);   setServoAngle(L3, 0);
    if (!pressingCheck("right", frameDelay)) return;
  }
  runStandPose(1);
  if (currentCommand == "right") currentCommand = "";
}
