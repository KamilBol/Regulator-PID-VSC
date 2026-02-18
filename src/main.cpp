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
#define PIN_DAC_SDA     41  // Pin D na module DAC
#define PIN_DAC_SCL     40  // Pin C na module DAC

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

// POPRAWKA ADRESU DAC: Skoro przełączniki są w górze (ON/1), to adres to 0x5F
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

void scanI2C() {
    Serial.println("[I2C] Skanowanie (Szukam DACa)...");
    byte error, address;
    int nDevices = 0;
    for(address = 1; address < 127; address++ ) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            Serial.print("[I2C] Znalazlem: 0x");
            Serial.println(address,HEX);
            if(address == 0x5F) Serial.println(">>> TO TWÓJ DAC! ADRES OK.");
            nDevices++;
        }
    }
    if (nDevices == 0) Serial.println("[I2C] PUSTO! Sprawdz kable D/C i zasilanie.");
}

void initSD() {
    SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
    if(!SD.begin(PIN_SD_CS)) {
        Serial.println("[SD] BLAD!");
        myNex.writeStr("sd.txt", "ERR");
        return;
    }
    
    int i = 0;
    while(true) {
        logFileName = "/log_" + String(i) + ".csv";
        if(!SD.exists(logFileName)) break;
        i++;
    }
    
    File f = SD.open(logFileName, FILE_WRITE);
    if(f) {
        f.println("Czas_ms;SystemON;I;U;P;DAC1;DAC2");
        f.close();
        Serial.println("[SD] Utworzono: " + logFileName);
    }
}

void startRegulator() {
    systemON = true;
    myNex.writeStr("pidonoff.txt", "ON");
    digitalWrite(PIN_RELAY_1, RELAY_ON);
    digitalWrite(PIN_RELAY_2, RELAY_ON);
    
    currentDac1 = 5.00; 
    currentDac2 = currentDac1 * 0.90;
    
    // Ustawiamy napięcie na fizycznym DAC
    dac.setDACOutVoltage(currentDac1 * 1000, 0);
    dac.setDACOutVoltage(currentDac2 * 1000, 1);
    Serial.println("[SYS] START -> DAC SET 5V");
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
    Serial.println("[SYS] STOP -> DAC SET 0V");
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
// 4. PARSER DOTYKU
// ================================================================
int activeButtonID = 0;
unsigned long buttonHoldTimer = 0;
bool isButtonHeld = false;

void handleNextionInput() {
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
                    if (cmpId == 8) {
                        if (event == 0x01) { isResetPressed = true; resetPressTime = millis(); myNex.writeStr("logi0.txt", "Trzymaj 5s..."); }
                        else if (event == 0x00) { isResetPressed = false; myNex.writeStr("logi0.txt", "..."); }
                    }
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
    if (isResetPressed && (millis() - resetPressTime > 5000)) {
        myNex.writeStr("logi0.txt", "RESTART!"); delay(500); ESP.restart();
    }
}

// ================================================================
// 5. SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(2000); 
    
    Serial.println("\n--- ALIENWARE V12.0 (DAC ADDRESS FIX) ---");

    // Przekaźniki OFF
    pinMode(PIN_RELAY_1, OUTPUT);
    pinMode(PIN_RELAY_2, OUTPUT);
    digitalWrite(PIN_RELAY_1, RELAY_OFF);
    digitalWrite(PIN_RELAY_2, RELAY_OFF);

    // RS485
    pinMode(PIN_RS485_DE, OUTPUT);
    digitalWrite(PIN_RS485_DE, LOW);
    ModbusSerial.begin(9600, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);
    
    // NEXTION
    NextionSerial.begin(9600, SERIAL_8N1, PIN_NEXT_RX, PIN_NEXT_TX);
    myNex.begin(9600);
    
    // PZEM
    delay(200);
    PzemSerial.begin(9600, SERIAL_8N1, PIN_PZEM_RX, PIN_PZEM_TX);
    
    // DHT
    dht.begin();

    // DAC (Adres 0x5F)
    Wire.begin(PIN_DAC_SDA, PIN_DAC_SCL);
    scanI2C(); // Zobacz w logach czy znajdzie 0x5F
    
    if(dac.begin() != 0) {
        Serial.println("[DAC] Błąd komunikacji! (Czy na pewno adres 0x5F?)");
        myNex.writeStr("logi0.txt", "DAC ERR");
    } else {
        Serial.println("[DAC] Polaczono (Adres 0x5F OK).");
        dac.setDACOutRange(dac.eOutputRange10V);
        
        // TEST NAPIĘCIA: 5V na 5 sekund
        Serial.println("--- TEST DAC 5V START ---");
        dac.setDACOutVoltage(5000, 0);
        dac.setDACOutVoltage(5000, 1);
        myNex.writeStr("pod1.txt", "TEST 5V");
        delay(5000); 
        
        // Powrót do 0V
        dac.setDACOutVoltage(0, 0);
        dac.setDACOutVoltage(0, 1);
        myNex.writeStr("pod1.txt", "0.00 V");
        Serial.println("--- TEST DAC KONIEC ---");
    }

    // SD
    initSD();

    // Domyślne stany GUI
    myNex.writeStr("pidonoff.txt", "OFF");
    myNex.writeStr("pracaautoman.txt", "AUT");
    
    Serial.println("SYSTEM GOTOWY");
}

// ================================================================
// 6. LOOP
// ================================================================
void loop() {
    handleNextionInput();

    if (millis() - lastUpdate >= 1000) {
        lastUpdate = millis();

        float u = pzem.voltage();
        float i = pzem.current();
        float p = pzem.power();
        float pf = pzem.pf();
        float t = dht.readTemperature();
        float h = dht.readHumidity();

        // PZEM
        if(!isnan(u)) {
            float s = u * i;
            float q = (s > p) ? sqrt(s*s - p*p) : 0;
            char buf[16];
            sprintf(buf, "%.3f A", i); myNex.writeStr("granampery.txt", buf); myNex.writeStr("natgr.txt", buf);
            sprintf(buf, "%.1f V", u); myNex.writeStr("napgr.txt", buf);
            sprintf(buf, "%.0f W", p); myNex.writeStr("mocczy.txt", buf);
            sprintf(buf, "%.0f VA", s); myNex.writeStr("mocpoz.txt", buf);
            sprintf(buf, "%.0f Var", q); myNex.writeStr("mocbie.txt", buf);
            sprintf(buf, "%.2f", pf); myNex.writeStr("wspmoc.txt", buf);
            myNex.writeStr("logi0.txt", "OK");
        } else {
            myNex.writeStr("napgr.txt", "ERR");
        }

        // DHT
        if(!isnan(t)) {
            myNex.writeStr("temperatura.txt", String(t, 1));
            myNex.writeStr("wilgotnosc.txt", String(h, 0));
        }

        // DAC Logic
        if(systemON) {
            currentDac1 = 5.00; 
            currentDac2 = currentDac1 * 0.90;
        } else {
            currentDac1 = 0.00; 
            currentDac2 = 0.00;
        }
        
        // Fizyczne wysterowanie
        dac.setDACOutVoltage(currentDac1 * 1000, 0);
        dac.setDACOutVoltage(currentDac2 * 1000, 1);

        // GUI
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
                     f.print(p); f.print(";"); f.print(currentDac1); f.println(currentDac2);
                     f.close();
                 }
             }
        }
    }
}