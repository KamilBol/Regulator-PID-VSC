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
#include <WiFi.h> // Potrzebne tylko po to, żeby je WYŁĄCZYĆ

// ================================================================
// 1. PINOLOGIA
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

// LOGIKA PRZEKAŹNIKÓW (Low Trigger)
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
// UWAGA: ADRES 0x5F (dla zworek w górze)
DFRobot_GP8403 dac(&Wire, 0x5F); 

DHT dht(PIN_DHT, DHT11);
ModbusMaster node;

// --- TU BYŁ BŁĄD - PRZYWRACAM ZMIENNE PID ---
double Setpoint = 5.0, Input, Output;
PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);
// ---------------------------------------------

// Zmienne
float minLimit = 10.0;
float maxLimit = 40.0;
bool systemON = false;    
bool modeAUTO = true;     
float currentDac1 = 0.0;
float currentDac2 = 0.0;
String logFileName = "/log_000.csv";
unsigned long lastUpdate = 0;
unsigned long lastLogTime = 0;

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
    if(f) { f.println("Czas;PID;I;U;DAC1"); f.close(); }
}

void startRegulator() {
    systemON = true;
    myNex.writeStr("pidonoff.txt", "ON");
    digitalWrite(PIN_RELAY_1, RELAY_ON);
    digitalWrite(PIN_RELAY_2, RELAY_ON);
    currentDac1 = 5.00; currentDac2 = currentDac1 * 0.90;
    dac.setDACOutVoltage(currentDac1 * 1000, 0);
    dac.setDACOutVoltage(currentDac2 * 1000, 1);
}

void stopRegulator() {
    systemON = false;
    myNex.writeStr("pidonoff.txt", "OFF");
    digitalWrite(PIN_RELAY_1, RELAY_OFF);
    digitalWrite(PIN_RELAY_2, RELAY_OFF);
    currentDac1 = 0.00; currentDac2 = 0.00;
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
// 4. SETUP (ULTRA LOW POWER)
// ================================================================
void setup() {
    // 1. WYŁĄCZ WIFI/BT (Oszczędza prąd na start!)
    WiFi.mode(WIFI_OFF);
    
    // 2. PRZEKAŹNIKI OFF (Oszczędza prąd!)
    pinMode(PIN_RELAY_1, OUTPUT);
    pinMode(PIN_RELAY_2, OUTPUT);
    digitalWrite(PIN_RELAY_1, RELAY_OFF);
    digitalWrite(PIN_RELAY_2, RELAY_OFF);

    // 3. Serial
    Serial.begin(115200);
    delay(1000); 
    Serial.println("\n--- ALIENWARE V13.1 (LOW POWER FIX) ---");

    // 4. RS485
    pinMode(PIN_RS485_DE, OUTPUT);
    digitalWrite(PIN_RS485_DE, LOW);
    ModbusSerial.begin(9600, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);

    // 5. NEXTION (Startujemy powoli)
    delay(500);
    NextionSerial.begin(9600, SERIAL_8N1, PIN_NEXT_RX, PIN_NEXT_TX);
    myNex.begin(9600);
    // Próba przyciemnienia ekranu (oszczędność prądu)
    myNex.writeStr("dim=50"); 

    // 6. PZEM
    delay(200);
    PzemSerial.begin(9600, SERIAL_8N1, PIN_PZEM_RX, PIN_PZEM_TX);

    // 7. DHT & SD
    dht.begin();
    initSD();

    // 8. DAC (I2C)
    Wire.begin(PIN_DAC_SDA, PIN_DAC_SCL);
    if(dac.begin() != 0) {
        Serial.println("DAC ERR - Sprawdz 0x5F");
        myNex.writeStr("logi0.txt", "DAC ERR");
    } else {
        Serial.println("DAC OK");
        dac.setDACOutRange(dac.eOutputRange10V);
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
    // 1. NEXTION INPUT
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

    // 2. TIMED LOOP (1s)
    if (millis() - lastUpdate >= 1000) {
        lastUpdate = millis();

        float u = pzem.voltage();
        float i = pzem.current();
        float p = pzem.power();
        float pf = pzem.pf();
        float t = dht.readTemperature();
        float h = dht.readHumidity();

        // PZEM -> LCD
        if(!isnan(u)) {
            char buf[16];
            sprintf(buf, "%.3f A", i); myNex.writeStr("granampery.txt", buf); myNex.writeStr("natgr.txt", buf);
            sprintf(buf, "%.1f V", u); myNex.writeStr("napgr.txt", buf);
            sprintf(buf, "%.0f W", p); myNex.writeStr("mocczy.txt", buf);
            float s = u*i; sprintf(buf, "%.0f VA", s); myNex.writeStr("mocpoz.txt", buf);
            float q = (s>p)?sqrt(s*s-p*p):0; sprintf(buf, "%.0f Var", q); myNex.writeStr("mocbie.txt", buf);
            sprintf(buf, "%.2f", pf); myNex.writeStr("wspmoc.txt", buf);
            myNex.writeStr("logi0.txt", "OK");
        } else {
            myNex.writeStr("napgr.txt", "ERR");
        }

        // DHT -> LCD
        if(!isnan(t)) {
            myNex.writeStr("temperatura.txt", String(t, 1));
            myNex.writeStr("wilgotnosc.txt", String(h, 0));
        }

        // DAC
        if(systemON) { currentDac1 = 5.00; currentDac2 = currentDac1 * 0.90; }
        else { currentDac1 = 0; currentDac2 = 0; }
        
        dac.setDACOutVoltage(currentDac1 * 1000, 0);
        dac.setDACOutVoltage(currentDac2 * 1000, 1);
        
        char dacBuf[10];
        sprintf(dacBuf, "%.2f V", currentDac1); myNex.writeStr("pod1.txt", dacBuf); myNex.writeStr("dac1.txt", dacBuf);
        sprintf(dacBuf, "%.2f V", currentDac2); myNex.writeStr("pod2.txt", dacBuf); myNex.writeStr("dac2.txt", dacBuf);

        // SD LOG
        if(SD.cardType() != CARD_NONE) {
             float gb = SD.totalBytes() / (1024.0*1024.0*1024.0);
             myNex.writeStr("sd.txt", String(gb, 1) + " GB");
             if(millis() - lastLogTime >= 2000) {
                 lastLogTime = millis();
                 File f = SD.open(logFileName, FILE_APPEND);
                 if(f) {
                     f.print(millis()); f.print(";"); f.print(systemON); f.print(";");
                     f.print(i); f.print(";"); f.print(u); f.print(";");
                     f.print(currentDac1); f.println(currentDac2);
                     f.close();
                 }
             }
        }
    }
}