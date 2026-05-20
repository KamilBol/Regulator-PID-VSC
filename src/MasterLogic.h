#pragma once
#include <Arduino.h>
#include <PID_v1.h>

// --- Zmienne pobierane z main.cpp ---
extern int opMode;
extern float limDn;
extern int sirDnHi;
extern int sirDnLo;
extern int sirUpHi;
extern int sirUpLo;
extern float tGrace;
extern float tRamp;
extern int sirStart;
extern int tAutoRes;
extern bool manualSirenActive;

extern bool systemON;
extern bool modeAUTO;
extern bool trippedByOverload;
extern float current_Amps;
extern float minLimit;
extern float maxLimit;
extern float overloadLimit;
extern float recoveryLimit;
extern double Output;
extern PID myPID;

// --- Funkcje pobierane z main.cpp ---
extern void startRegulator();
extern void stopRegulator();

// --- Główne funkcje PLC (Udostępniane na zewnątrz) ---
void runMasterLogicStateMachine();
void handleSiren();
float getSoftStartOutput(float requestedOutput);
void triggerSlaveAnomaly(int seconds);