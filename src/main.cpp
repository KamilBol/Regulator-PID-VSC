#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <PZEM004Tv30.h>
#include <EasyNextionLibrary.h>
#include <DFRobot_GP8403.h>
#include <PID_v1.h>
#include <DHT.h>
#include <ModbusMaster.h>
#include <WiFi.h> 

// ================================================================
// 1. PINOLOGIA (S3)
// ================================================================
#define PIN_PZEM_RX     4
#define PIN_PZEM_TX     5
#define PIN_RS485_RX    1
#define PIN_RS485_TX    2
#define PIN_RS485_DE    6
#define PIN_RELAY_1     47
#define PIN_RELAY_2     48
#define PIN_NEXT_RX     13
#define PIN_NEXT_TX     14
#define PIN_DHT         20
#define PIN_SD_CS       15
#define PIN_SD_SCK      16
#define PIN_SD_MOSI     17
#define PIN_SD_MISO     18
#define PIN_DAC_SDA     41
#define PIN_DAC_SCL     40

// LOGIKA PRZEKAŹNIKÓW (Low Trigger: LOW=ON, HIGH=OFF)
#define RELAY_ON        LOW
#define RELAY_OFF       HIGH

// ================================================================
// 2. OBIEKTY
// ================================================================
HardwareSerial NextionSerial(1);
HardwareSerial PzemSerial(2);
HardwareSerial ModbusSerial(0);

EasyNex myNex(NextionSerial);
PZEM004Tv30 pzem(PzemSerial, PIN_PZEM_RX, PIN_PZEM_TX);

// ADRES DAC: 0x5F (Dla zworek w górze - ON)
DFRobot_GP8403 dac(&Wire, 0x5F); 

DHT dht(PIN_DHT, DHT11);
ModbusMaster node;

// PID
double Setpoint = 5.0, Input, Output;
PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);

// Zmienne Systemowe
float minLimit = 10.0;
float maxLimit = 40.0;
bool systemON = false;    
bool modeAUTO = true;     
float currentDac1 = 0.0;
float currentDac2 = 0.0;
String logFileName = "/log_000.csv";

// Timery
unsigned long lastUpdate = 0;
unsigned long lastLogTime = 0;
unsigned long resetPressTime = 0;
bool isResetPressed = false;

// ================================================================
// 3. FUNKCJE POMOCNICZE
// ================================================================
void preTransmission() { digitalWrite(PIN_RS485_DE, HIGH); }
void postTransmission() { digitalWrite(PIN_RS485_DE, LOW); }

void initSD() {
    SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
    if(!SD.begin(PIN_SD_CS)) return;
    int i = 0;
    while(true) {
        logFileName = "/log_" + String(i) + ".csv";
        if(!SD.exists(logFileName)) break;
        i++;
    }
    File f = SD.open(logFileName, FILE_WRITE);
    if(f) { f.println("Czas;PID;I;U;DAC1;Freq_Inv"); f.close(); }
}

void startRegulator() {
    systemON = true;
    myNex.writeStr("pidonoff.txt", "ON");
    digitalWrite(PIN_RELAY_1, RELAY_ON);
    digitalWrite(PIN_RELAY_2, RELAY_ON);
    // Startowe wartości
    currentDac1 = 5.00; 
    currentDac2 = currentDac1 * 0.90;
    dac.setDACOutVoltage(currentDac1 * 1000, 0);
    dac.setDACOutVoltage(currentDac2 * 1000, 1);
}

void stopRegulator() {
    systemON = false;
    myNex.writeStr("pidonoff.txt", "OFF");
    digitalWrite(PIN_RELAY_1, RELAY_OFF);
    digitalWrite(PIN_RELAY_2, RELAY_OFF);
    currentDac1 = 0.00; 
    currentDac2 = 0.00;
    dac.setDACOutVoltage(0, 0);
    dac.setDACOutVoltage(0, 1);
}

void updateSettingsScreen() {
    myNex.writeStr("min.txt", String(minLimit, 1));
    myNex.writeStr("max.txt", String(maxLimit, 1));
}

void processButtonAction(int id) {
    if (id == 1) { minLimit += 0.1; if(minLimit > maxLimit) minLimit = maxLimit; updateSettingsScreen(); }
    if (id == 2) { minLimit -= 0.1; if(minLimit < 0) minLimit = 0; updateSettingsScreen(); }
    if (id == 3) { maxLimit += 0.1; if(maxLimit > 100) maxLimit = 100; updateSettingsScreen(); }
    if (id == 4) { maxLimit -= 0.1; if(maxLimit < minLimit) maxLimit = minLimit; updateSettingsScreen(); }
}

// ================================================================
// 4. SETUP
// ================================================================
void setup() {
    // 1. Oszczędzanie prądu
    WiFi.mode(WIFI_OFF);
    pinMode(PIN_RELAY_1, OUTPUT);
    pinMode(PIN_RELAY_2, OUTPUT);
    digitalWrite(PIN_RELAY_1, RELAY_OFF);
    digitalWrite(PIN_RELAY_2, RELAY_OFF);

    // 2. Serial
    Serial.begin(115200);
    delay(1000); 
    Serial.println("\n--- REGULATOR V14 FINAL ---");

    // 3. RS485 (Master)
    pinMode(PIN_RS485_DE, OUTPUT);
    digitalWrite(PIN_RS485_DE, LOW);
    ModbusSerial.begin(9600, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);
    node.begin(1, ModbusSerial); // Szukamy Slave ID 1
    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission);

    // 4. Nextion
    delay(500);
    NextionSerial.begin(9600, SERIAL_8N1, PIN_NEXT_RX, PIN_NEXT_TX);
    myNex.begin(9600);
    myNex.writeStr("dim=50"); 

    // 5. PZEM
    delay(200);
    PzemSerial.begin(9600, SERIAL_8N1, PIN_PZEM_RX, PIN_PZEM_TX);

    // 6. DHT & SD
    dht.begin();
    initSD();

    // 7. DAC
    Wire.begin(PIN_DAC_SDA, PIN_DAC_SCL);
    if(dac.begin() != 0) {
        Serial.println("DAC ERR (Check 0x5F)");
    } else {
        Serial.println("DAC OK");
        dac.setDACOutRange(dac.eOutputRange10V);
        // TEST: 5V na start
        dac.setDACOutVoltage(5000, 0);
        delay(1000);
        dac.setDACOutVoltage(0, 0);
    }

    Serial.println("SYSTEM GOTOWY");
}

// ================================================================
// 5. LOOP
// ================================================================
int activeButtonID = 0;
unsigned long buttonHoldTimer = 0;
bool isButtonHeld = false;

void loop() {
    // --- 1. OBSŁUGA NEXTION ---
    while (NextionSerial.available()) {
        byte b = NextionSerial.read();
        if (b == 0x65) {
            delay(5);
            if (NextionSerial.available() >= 3) {
                byte pageId = NextionSerial.read();
                byte cmpId  = NextionSerial.read();
                byte event  = NextionSerial.read(); 
                while (NextionSerial.available()) NextionSerial.read();

                if (pageId == 0) {
                    if (cmpId == 11 && event == 0x01) { 
                        if(systemON) stopRegulator(); else startRegulator(); 
                    }
                    if (cmpId == 12 && event == 0x01) { 
                        modeAUTO = !modeAUTO;
                        myNex.writeStr("pracaautoman.txt", modeAUTO ? "AUT" : "MAN");
                    }
                    if (cmpId == 8 && event == 0x01) ESP.restart();
                }
                if (pageId == 2) {
                    if (event == 0x01) { activeButtonID = cmpId; isButtonHeld = true; processButtonAction(activeButtonID); buttonHoldTimer = millis() + 400; }
                    else if (event == 0x00) { activeButtonID = 0; isButtonHeld = false; }
                }
            }
        }
    }
    if (isButtonHeld && activeButtonID > 0 && millis() > buttonHoldTimer) {
        processButtonAction(activeButtonID);
        buttonHoldTimer = millis() + 100;
    }

    // --- 2. GŁÓWNA PĘTLA (Co 1s) ---
    if (millis() - lastUpdate >= 1000) {
        lastUpdate = millis();

        // ODCZYTY
        float u = pzem.voltage();
        float i = pzem.current();
        float p = pzem.power();
        float t = dht.readTemperature();
        float inverterFreq = 0.0;

        // MODBUS (Tylko w trybie AUTO)
        if (modeAUTO) {
            uint8_t result = node.readHoldingRegisters(0x1000, 1);
            if (result == node.ku8MBSuccess) {
                inverterFreq = node.getResponseBuffer(0) / 100.0; // np. 5000 -> 50.00 Hz
                Serial.printf("Falownik: %.2f Hz\n", inverterFreq);
                
                // LOGIKA AUTO-STARTU
                if (inverterFreq > 45.0 && !systemON) startRegulator();
                if (inverterFreq < 40.0 && systemON) stopRegulator();
            }
        }

        // GUI UPDATE
        if(!isnan(u)) {
            char buf[16];
            sprintf(buf, "%.3f A", i); myNex.writeStr("granampery.txt", buf); myNex.writeStr("natgr.txt", buf);
            sprintf(buf, "%.1f V", u); myNex.writeStr("napgr.txt", buf);
            sprintf(buf, "%.0f W", p); myNex.writeStr("mocczy.txt", buf);
        } else {
            myNex.writeStr("napgr.txt", "ERR");
        }

        // DAC Logic (Prosta symulacja PID)
        if(systemON) { 
            currentDac1 = 5.00; 
            currentDac2 = currentDac1 * 0.90; 
        } else { 
            currentDac1 = 0; 
            currentDac2 = 0; 
        }
        
        dac.setDACOutVoltage(currentDac1 * 1000, 0);
        dac.setDACOutVoltage(currentDac2 * 1000, 1);
        
        char dacBuf[10];
        sprintf(dacBuf, "%.2f V", currentDac1); myNex.writeStr("pod1.txt", dacBuf); myNex.writeStr("dac1.txt", dacBuf);
        sprintf(dacBuf, "%.2f V", currentDac2); myNex.writeStr("pod2.txt", dacBuf); myNex.writeStr("dac2.txt", dacBuf);
    }
}