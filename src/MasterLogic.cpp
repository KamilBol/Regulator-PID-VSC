#include "MasterLogic.h"

// --- ZMIENNE WEWNĘTRZNE MASZYNY STANÓW ---
bool masterAlarmUp = false;
bool masterAlarmDn = false;
bool isRamping = false;

unsigned long systemStartTime = 0;
unsigned long alarmTriggerTime = 0;
unsigned long rampStartTime = 0;
unsigned long sirenStartBeepTime = 0;
bool startBeepActive = false;
bool wasSystemON = false;

unsigned long slaveAnomalyEndTime = 0;
bool slaveAnomalyActive = false;

// Funkcja pomocnicza do uderzenia anomalii z poziomu WWW (w trybie Slave)
void triggerSlaveAnomaly(int seconds) {
    slaveAnomalyActive = true;
    slaveAnomalyEndTime = millis() + (seconds * 1000UL);
}

// GŁÓWNA MASZYNA STANÓW (Wywoływana z loop)
void runMasterLogicStateMachine() {
    // 1. DETEKCJA ZBOCZA NARASTAJĄCEGO (Kliknięcie ON na ekranie/PC)
    if (systemON && !wasSystemON) {
        systemStartTime = millis();
        masterAlarmUp = false;
        masterAlarmDn = false;
        trippedByOverload = false;
        slaveAnomalyActive = false;
        
        // Reset członu całkującego "I" w PID, aby maszyna nie wystrzeliła po starcie
        myPID.SetMode(MANUAL);
        Output = 0.0;
        myPID.SetMode(AUTOMATIC);

        // Odpalenie sekwencji miękkiego startu
        isRamping = true;
        rampStartTime = millis();
        
        if (sirStart > 0) {
            startBeepActive = true;
            sirenStartBeepTime = millis();
        }
    }
    
    // 2. DETEKCJA TWARDEGO WYŁĄCZENIA (Kliknięcie OFF / Acknowledge)
    if (!systemON && wasSystemON) {
        masterAlarmUp = false;
        masterAlarmDn = false;
        isRamping = false;
        startBeepActive = false;
    }
    
    wasSystemON = systemON;

    // =========================================================
    // 3. LOGIKA SLAVE (Pół-Miękka / Ustępliwa)
    // =========================================================
    if (opMode == 1) {
        if (systemON && current_Amps >= (maxLimit + overloadLimit)) {
            stopRegulator(); // Rozłącza przekaźniki i sprzętowo oddaje władzę
            trippedByOverload = true;
        }
        if (!systemON && trippedByOverload && modeAUTO) {
            if (current_Amps <= (minLimit + recoveryLimit)) {
                startRegulator(); // Zwiera przekaźniki z powrotem
                trippedByOverload = false;
            }
        }
        
        // Wyłączenie anomalii po upływie czasu
        if (slaveAnomalyActive && millis() > slaveAnomalyEndTime) {
            slaveAnomalyActive = false;
        }
        return; // Koniec pracy dla trybu SLAVE
    }

    // =========================================================
    // 4. LOGIKA MASTER (Twardy Dyktator PLC)
    // =========================================================
    if (opMode == 0 && systemON) {
        bool isAlarmActive = (masterAlarmUp || masterAlarmDn);

        // A. Sprawdzanie wyzwalaczy (jeśli maszyna jeszcze nie wyje)
        if (!isAlarmActive) {
            // Przeciążenie górne (Zacięcie) - reaguje NATYCHMIAST
            if (current_Amps >= (maxLimit + overloadLimit)) {
                masterAlarmUp = true;
                alarmTriggerTime = millis();
                isRamping = false; // Przerwanie ewentualnego startu
            }
            // Przeciążenie dolne (Mokra Trocina) - pomija czas tGrace od startu
            else if (!isRamping && (millis() - systemStartTime > (unsigned long)(tGrace * 1000))) {
                if (current_Amps <= limDn) {
                    masterAlarmDn = true;
                    alarmTriggerTime = millis();
                }
            }
        } 
        // B. Reakcja na aktywny błąd
        else {
            if (modeAUTO) {
                // W trybie AUTO, ESP32 samo spróbuje wstać po czasie tAutoRes
                if (millis() - alarmTriggerTime >= (unsigned long)(tAutoRes * 1000)) {
                    masterAlarmUp = false;
                    masterAlarmDn = false;
                    
                    // Reset PID przed wstaniem
                    myPID.SetMode(MANUAL);
                    Output = 0.0;
                    myPID.SetMode(AUTOMATIC);
                    
                    // Inicjalizacja Soft-Startu
                    isRamping = true;
                    rampStartTime = millis();
                    if (sirStart > 0) {
                        startBeepActive = true;
                        sirenStartBeepTime = millis();
                    }
                }
            } else {
                // W trybie MANUAL (MAN) zrzucamy flagę systemową
                // Syrena wyje aż operator kliknie na ekranie "Zasilanie: OFF" i potem znów "ON"
                stopRegulator();
            }
        }
    }
}

// MODYFIKATOR NAPIĘCIA DAC (Odpowiada za ucinanie zasilania i miękki start)
float getSoftStartOutput(float requestedOutput) {
    if (opMode == 1) return requestedOutput; // W SLAVE DAC steruje liniowo (przekaźniki robią robotę)
    
    // TWARDE ZBICIE FALOWNIKA DO 0.0V (Bez puszczania przekaźnika!)
    if (masterAlarmUp || masterAlarmDn || slaveAnomalyActive) {
        return 0.0; 
    }

    // MIĘKKI START (Najazd napięcia)
    if (isRamping) {
        unsigned long elapsed = millis() - rampStartTime;
        unsigned long rampDur = (unsigned long)(tRamp * 1000);
        
        if (elapsed >= rampDur || rampDur == 0) {
            isRamping = false;
            return requestedOutput;
        }
        
        // Liniowy przyrost napięcia od zera do celu z algorytmu PID
        float progress = (float)elapsed / (float)rampDur;
        return requestedOutput * progress;
    }

    return requestedOutput;
}

// POLIMORFICZNY STEROWNIK SYRENY (Bez użycia delay!)
void handleSiren() {
    // 1. Priorytet bezwzględny - Ręczna syrena "Ewakuacja" z PC/WWW
    if (manualSirenActive) {
        digitalWrite(32, HIGH);
        return;
    }

    // 2. Obsługa wymuszonej anomalii w SLAVE
    if (opMode == 1) {
        if (slaveAnomalyActive) digitalWrite(32, HIGH);
        else digitalWrite(32, LOW);
        return;
    }

    // 3. Maszyna wyłączona - kategoryczna cisza na hali
    if (!systemON && !startBeepActive) {
        digitalWrite(32, LOW);
        return;
    }

    // 4. Krótkie piknięcie ostrzegawcze przed ruszeniem falownika
    if (startBeepActive) {
        if (millis() - sirenStartBeepTime < (unsigned long)sirStart) {
            digitalWrite(32, HIGH);
        } else {
            digitalWrite(32, LOW);
            startBeepActive = false;
        }
        return;
    }

    // 5. Złożone sygnały alarmowe w MASTER (Przez zmienne czasy wycia w milisekundach)
    if (masterAlarmUp) {
        unsigned long cycle = sirUpHi + sirUpLo;
        if (cycle > 0) {
            digitalWrite(32, ((millis() % cycle) < (unsigned long)sirUpHi) ? HIGH : LOW);
        } else digitalWrite(32, HIGH);
    }
    else if (masterAlarmDn) {
        unsigned long cycle = sirDnHi + sirDnLo;
        if (cycle > 0) {
            digitalWrite(32, ((millis() % cycle) < (unsigned long)sirDnHi) ? HIGH : LOW);
        } else digitalWrite(32, HIGH);
    }
    else {
        digitalWrite(32, LOW);
    }
}