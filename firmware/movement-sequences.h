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
  setServoAngle(R1, 66);
  setServoAngle(R2, 76);
  setServoAngle(L1, 86);
  setServoAngle(L2, 66);
  setServoAngle(R4, 70);
  setServoAngle(R3, 85);
  setServoAngle(L3, 53);
  setServoAngle(L4, 81);
}

inline void runStandPose(int face) { 
  Serial.println(F("STAND (Calibrated)")); 
  if (face == 1) setFaceWithMode("stand", FACE_ANIM_ONCE); 
  setServoAngle(R1, 85); 
  setServoAngle(R2, 76); 
  setServoAngle(L1, 73); 
  setServoAngle(L2, 81); 
  setServoAngle(R4, 4); 
  setServoAngle(R3, 145); 
  setServoAngle(L3, 0); 
  setServoAngle(L4, 145); 
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
//   FR (R1: 85, R3: 145) | RR (R2: 76, R4: 4)
//   FL (L1: 73, L3: 0)   | RL (L2: 81, L4: 145)
//
// Knee Contact Dynamics (Down towards floor):
//   - R3 (FR Knee): Down towards 180. Stand/Touch=145, Lift/Swing=115
//   - L4 (RL Knee): Down towards 180. Stand/Touch=145, Lift/Swing=113
//   - L3 (FL Knee): Down towards 0.   Stand/Touch=0,   Lift/Swing=27
//   - R4 (RR Knee): Down towards 0.   Stand/Touch=4,   Lift/Swing=37
//
// Hip Stroke Dynamics (Forward vs Backward):
//   - R1 (FR Hip): Forward towards 174 (Swing=104, Push=75, Stand=85)
//   - L2 (RL Hip): Forward towards 76  (Swing=77,  Push=81/96, Stand=81)
//   - L1 (FL Hip): Forward towards 5   (Swing=60,  Push=86, Stand=73)
//   - R2 (RR Hip): Forward towards 90  (Swing=88,  Push=76/61, Stand=76)

inline void runWalkPose() {
  Serial.println(F("WALK FWD (Calibrated)"));
  setFaceWithMode("walk", FACE_ANIM_ONCE);

  // Initial Step (stride preparation)
  setServoAngle(R3, 115); setServoAngle(L3, 27);
  setServoAngle(R2, 88);  setServoAngle(L1, 67);
  if (!pressingCheck("forward", frameDelay)) return;

  for (int i = 0; i < walkCycles; i++) {
    // Step 1: Plant FL knee firmly on floor; keep FR knee lifted
    setServoAngle(R3, 115); setServoAngle(L3, 0);
    if (!pressingCheck("forward", frameDelay)) return;

    // Step 2: Lift Pair 1 (L4=113, R3=115) & swing forward (R1=104, L2=77); Pair 2 planted on floor (R4=4)
    setServoAngle(L4, 113); setServoAngle(L2, 77);
    setServoAngle(R4, 4);   setServoAngle(R1, 104);
    if (!pressingCheck("forward", frameDelay)) return;

    // Step 3: Pair 2 drives body forward (R2 pushes to 76, L1 pushes to 86)
    setServoAngle(R2, 76);  setServoAngle(L1, 86);
    if (!pressingCheck("forward", frameDelay)) return;

    // Step 4: Touchdown Pair 1 on floor (L4=145); Pair 2 begins lift (R4=37)
    setServoAngle(R4, 37);  setServoAngle(L4, 145);
    if (!pressingCheck("forward", frameDelay)) return;

    // Step 5: Plant FR knee (R3=145), lift FL knee (L3=27), swing Pair 2 forward (R2=88, L1=60)
    setServoAngle(R3, 145); setServoAngle(L3, 27);
    setServoAngle(R2, 88);  setServoAngle(L1, 60);
    if (!pressingCheck("forward", frameDelay)) return;

    // Step 6: Pair 1 drives body forward (L2 pushes to 81, R1 pushes to 75)
    setServoAngle(L2, 81);  setServoAngle(R1, 75);
    if (!pressingCheck("forward", frameDelay)) return;
  }
  runStandPose(1);
  if (currentCommand == "forward") currentCommand = "";
}

inline void runWalkBackward() {
  Serial.println(F("WALK BACK (Calibrated)"));
  setFaceWithMode("walk", FACE_ANIM_ONCE);
  if (!pressingCheck("backward", frameDelay)) return;

  for (int i = 0; i < walkCycles; i++) {
    // Step 1
    setServoAngle(R3, 115); setServoAngle(L3, 0);
    if (!pressingCheck("backward", frameDelay)) return;

    // Step 2: Swing Pair 1 backward in air
    setServoAngle(L4, 113); setServoAngle(L2, 81);
    setServoAngle(R4, 4);   setServoAngle(R1, 75);
    if (!pressingCheck("backward", frameDelay)) return;

    // Step 3: Pair 2 pushes body backward by thrusting forward on floor
    setServoAngle(R2, 88);  setServoAngle(L1, 60);
    if (!pressingCheck("backward", frameDelay)) return;

    // Step 4: Plant RL knee, lift RR knee
    setServoAngle(R4, 37);  setServoAngle(L4, 145);
    if (!pressingCheck("backward", frameDelay)) return;

    // Step 5: Plant FR knee, lift FL knee, swing Pair 2 backward in air
    setServoAngle(R3, 145); setServoAngle(L3, 27);
    setServoAngle(R2, 76);  setServoAngle(L1, 86);
    if (!pressingCheck("backward", frameDelay)) return;

    // Step 6: Pair 1 pushes body backward by thrusting forward on floor
    setServoAngle(L2, 77);  setServoAngle(R1, 104);
    if (!pressingCheck("backward", frameDelay)) return;
  }
  runStandPose(1);
  if (currentCommand == "backward") currentCommand = "";
}

inline void runTurnLeft() {
  Serial.println(F("TURN LEFT (Calibrated Step & Twist)"));
  setFaceWithMode("walk", FACE_ANIM_ONCE);

  for (int i = 0; i < walkCycles; i++) {
    // --- Legset 1 (FR: R1/R3 + RL: L2/L4) ---
    setServoAngle(R3, 115); setServoAngle(L4, 113); // Lift Pair 1 knees
    if (!pressingCheck("left", frameDelay)) return;

    // Swing hips to rotate counter-clockwise (R1 forward to 104, L2 backward to 96)
    setServoAngle(R1, 104); setServoAngle(L2, 96);
    if (!pressingCheck("left", frameDelay)) return;

    // Touchdown Pair 1 on floor
    setServoAngle(R3, 145); setServoAngle(L4, 145);
    if (!pressingCheck("left", frameDelay)) return;

    // Rotate body by pulling hips back to stand pose while feet are on floor
    setServoAngle(R1, 85);  setServoAngle(L2, 81);
    if (!pressingCheck("left", frameDelay)) return;

    // --- Legset 2 (FL: L1/L3 + RR: R2/R4) ---
    setServoAngle(R4, 37);  setServoAngle(L3, 27);  // Lift Pair 2 knees
    if (!pressingCheck("left", frameDelay)) return;

    // Swing hips to rotate counter-clockwise (R2 forward to 88, L1 backward to 86)
    setServoAngle(R2, 88);  setServoAngle(L1, 86);
    if (!pressingCheck("left", frameDelay)) return;

    // Touchdown Pair 2 on floor
    setServoAngle(R4, 4);   setServoAngle(L3, 0);
    if (!pressingCheck("left", frameDelay)) return;

    // Rotate body by pulling hips back to stand pose while feet are on floor
    setServoAngle(R2, 76);  setServoAngle(L1, 73);
    if (!pressingCheck("left", frameDelay)) return;
  }
  runStandPose(1);
  if (currentCommand == "left") currentCommand = "";
}

inline void runTurnRight() {
  Serial.println(F("TURN RIGHT (Calibrated Step & Twist)"));
  setFaceWithMode("walk", FACE_ANIM_ONCE);

  for (int i = 0; i < walkCycles; i++) {
    // --- Legset 2 (FL: L1/L3 + RR: R2/R4) ---
    setServoAngle(R4, 37);  setServoAngle(L3, 27);  // Lift Pair 2 knees
    if (!pressingCheck("right", frameDelay)) return;

    // Swing hips to rotate clockwise (R2 backward to 61, L1 forward to 60)
    setServoAngle(R2, 61);  setServoAngle(L1, 60);
    if (!pressingCheck("right", frameDelay)) return;

    // Touchdown Pair 2 on floor
    setServoAngle(R4, 4);   setServoAngle(L3, 0);
    if (!pressingCheck("right", frameDelay)) return;

    // Rotate body by pulling hips back to stand pose while feet are on floor
    setServoAngle(R2, 76);  setServoAngle(L1, 73);
    if (!pressingCheck("right", frameDelay)) return;

    // --- Legset 1 (FR: R1/R3 + RL: L2/L4) ---
    setServoAngle(R3, 115); setServoAngle(L4, 113); // Lift Pair 1 knees
    if (!pressingCheck("right", frameDelay)) return;

    // Swing hips to rotate clockwise (R1 backward to 75, L2 forward to 77)
    setServoAngle(R1, 75);  setServoAngle(L2, 77);
    if (!pressingCheck("right", frameDelay)) return;

    // Touchdown Pair 1 on floor
    setServoAngle(R3, 145); setServoAngle(L4, 145);
    if (!pressingCheck("right", frameDelay)) return;

    // Rotate body by pulling hips back to stand pose while feet are on floor
    setServoAngle(R1, 85);  setServoAngle(L2, 81);
    if (!pressingCheck("right", frameDelay)) return;
  }
  runStandPose(1);
  if (currentCommand == "right") currentCommand = "";
}
