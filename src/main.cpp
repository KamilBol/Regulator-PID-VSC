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
// 1. PINOLOGIA (Hardcoded)
// ================================================================
#define PIN_PZEM_RX     4
#define PIN_PZEM_TX     5
#define PIN_RS485_RX    1
#define PIN_RS485_TX    2
#define PIN_RS485_DE    6
#define PIN_RELAY_1     47
#define PIN_RELAY_2     38 // <--- ZMIENIONO Z 48 NA BEZPIECZNY PIN 38
#define PIN_NEXT_RX     13
#define PIN_NEXT_TX     14
#define PIN_DHT         20
#define PIN_SD_CS       15
#define PIN_SD_SCK      16
#define PIN_SD_MOSI     17
#define PIN_SD_MISO     18
#define PIN_DAC_SDA     41
#define PIN_DAC_SCL     40

// LOGIKA PRZEKAŹNIKÓW (Dostosuj jeśli masz moduł High-Trigger)
// Dla modułów Low-Trigger (większość): ON = LOW, OFF = HIGH
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
DFRobot_GP8403 dac(&Wire, 0x58); // Adres 0x58 (wszystkie zworki rozwarte)
DHT dht(PIN_DHT, DHT11);
ModbusMaster node;

// PID
double Setpoint = 5.0, Input, Output;
PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);

// Zmienne Systemowe
float minLimit = 10.0;
float maxLimit = 40.0;
bool systemON = false;    // PID ON/OFF
bool modeAUTO = true;     // TRUE = Auto-Restart, FALSE = Manual

// Zmienne Pracy
float currentDac1 = 0.0;
float currentDac2 = 0.0;
String logFileName = "/log_000.csv";

// Timery
unsigned long lastUpdate = 0;
unsigned long lastLogTime = 0;

// Reset
unsigned long resetPressTime = 0;
bool isResetPressed = false;

// ================================================================
// 3. FUNKCJE POMOCNICZE
// ================================================================

void preTransmission() { digitalWrite(PIN_RS485_DE, HIGH); }
void postTransmission() { digitalWrite(PIN_RS485_DE, LOW); }

void scanI2C() {
    Serial.println("[I2C] Skanowanie magistrali...");
    byte error, address;
    int nDevices = 0;
    for(address = 1; address < 127; address++ ) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            Serial.print("[I2C] Znaleziono urzadzenie pod adresem: 0x");
            if (address<16) Serial.print("0");
            Serial.println(address,HEX);
            nDevices++;
        }
    }
    if (nDevices == 0) Serial.println("[I2C] Brak urzadzen! Sprawdz piny 40/41.");
    else Serial.println("[I2C] Koniec skanowania.");
}

void initSD() {
    SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
    if(!SD.begin(PIN_SD_CS)) {
        Serial.println("[SD] BLAD INICJALIZACJI! (Sprawdz format FAT32)");
        myNex.writeStr("sd.txt", "ERR");
        return;
    }
    
    // Szukamy wolnego pliku
    int i = 0;
    while(true) {
        logFileName = "/log_" + String(i) + ".csv";
        if(!SD.exists(logFileName)) break;
        i++;
    }
    
    File f = SD.open(logFileName, FILE_WRITE);
    if(f) {
        f.println("Czas;PID;Prad;Nap;Moc;DAC1;DAC2");
        f.close(); // Zamykamy od razu!
        Serial.println("[SD] Utworzono plik: " + logFileName);
    }
}

// ================================================================
// 4. LOGIKA STEROWANIA
// ================================================================

void startRegulator() {
    systemON = true;
    myNex.writeStr("pidonoff.txt", "ON");
    
    // Włączamy przekaźniki (logika odwrócona)
    digitalWrite(PIN_RELAY_1, RELAY_ON);
    digitalWrite(PIN_RELAY_2, RELAY_ON);
    
    // Testowo ustawiamy 5V na start (zanim PID ruszy)
    currentDac1 = 5.00; 
    currentDac2 = currentDac1 * 0.90;
    
    // Wyślij na DAC
    dac.setDACOutVoltage(currentDac1 * 1000, 0);
    dac.setDACOutVoltage(currentDac2 * 1000, 1);
    
    Serial.println("[SYS] START REGULATORA");
}

void stopRegulator() {
    systemON = false;
    myNex.writeStr("pidonoff.txt", "OFF");
    
    // Wyłączamy przekaźniki
    digitalWrite(PIN_RELAY_1, RELAY_OFF);
    digitalWrite(PIN_RELAY_2, RELAY_OFF);
    
    // Zerujemy DAC
    currentDac1 = 0.00;
    currentDac2 = 0.00;
    dac.setDACOutVoltage(0, 0);
    dac.setDACOutVoltage(0, 1);
    
    Serial.println("[SYS] STOP REGULATORA");
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
// 5. PARSER DOTYKU
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

                // PAGE 0
                if (pageId == 0) {
                    if (cmpId == 11 && event == 0x01) { // PID ON/OFF
                        // Działa ZAWSZE, niezależnie od AUTO/MAN
                        if(systemON) stopRegulator(); else startRegulator();
                    }
                    if (cmpId == 12 && event == 0x01) { // AUTO/MAN
                        modeAUTO = !modeAUTO;
                        myNex.writeStr("pracaautoman.txt", modeAUTO ? "AUT" : "MAN");
                    }
                    if (cmpId == 8) { // RESET (Hold logic)
                        if (event == 0x01) { isResetPressed = true; resetPressTime = millis(); myNex.writeStr("logi0.txt", "Trzymaj 5s..."); }
                        else if (event == 0x00) { isResetPressed = false; myNex.writeStr("logi0.txt", "..."); }
                    }
                }
                
                // PAGE 2 (Limity)
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
// 6. SETUP
// ================================================================
void setup() {
    // 1. ZABEZPIECZENIE PRZEKAŹNIKÓW (Stan OFF na start)
    pinMode(PIN_RELAY_1, OUTPUT);
    pinMode(PIN_RELAY_2, OUTPUT);
    digitalWrite(PIN_RELAY_1, RELAY_OFF);
    digitalWrite(PIN_RELAY_2, RELAY_OFF);

    delay(1000); 
    Serial.begin(115200);
    Serial.println("\n--- SYSTEM V8.0 (DAC + RELAY FIX) ---");

    // 2. RS485
    pinMode(PIN_RS485_DE, OUTPUT);
    digitalWrite(PIN_RS485_DE, LOW);
    ModbusSerial.begin(9600, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);

    // 3. NEXTION
    NextionSerial.begin(9600, SERIAL_8N1, PIN_NEXT_RX, PIN_NEXT_TX);
    myNex.begin(9600);
    
    // 4. PZEM
    PzemSerial.begin(9600, SERIAL_8N1, PIN_PZEM_RX, PIN_PZEM_TX);
    
    // 5. DHT
    dht.begin();

    // 6. DAC (I2C Fix)
    // Najpierw odpalamy Wire na pinach 40/41
    Wire.begin(PIN_DAC_SDA, PIN_DAC_SCL);
    scanI2C(); // Sprawdzamy czy DAC odpowiada
    
    if(dac.begin() != 0) {
        Serial.println("[DAC] Blad komunikacji! (Adres inny niz 0x58?)");
    } else {
        Serial.println("[DAC] Polaczono poprawnie.");
        dac.setDACOutRange(dac.eOutputRange10V);
        dac.setDACOutVoltage(0, 0);
    }

    // 7. SD
    initSD();

    // Domyślne stany
    myNex.writeStr("pidonoff.txt", "OFF");
    myNex.writeStr("pracaautoman.txt", "AUT");
    stopRegulator(); // Dla pewności
    
    Serial.println("SYSTEM GOTOWY");
}

// ================================================================
// 7. LOOP
// ================================================================
void loop() {
    handleNextionInput();

    if (millis() - lastUpdate >= 1000) {
        lastUpdate = millis();

        // Pomiary
        float u = pzem.voltage();
        float i = pzem.current();
        float p = pzem.power();
        float pf = pzem.pf();
        float t = dht.readTemperature();
        float h = dht.readHumidity();

        // EKRAN: PZEM
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
            
            // Ping w logach że żyje
            Serial.printf("[PZEM] OK | I=%.3f A\n", i);
        } else {
            myNex.writeStr("napgr.txt", "ERR");
            Serial.println("[PZEM] Brak komunikacji!");
        }

        // EKRAN: DHT
        if(!isnan(t)) {
            myNex.writeStr("temperatura.txt", String(t, 1));
            myNex.writeStr("wilgotnosc.txt", String(h, 0));
        }

        // EKRAN: DAC Status
        char dacBuf[10];
        sprintf(dacBuf, "%.2f V", currentDac1);
        myNex.writeStr("pod1.txt", dacBuf);
        myNex.writeStr("dac1.txt", dacBuf);
        sprintf(dacBuf, "%.2f V", currentDac2);
        myNex.writeStr("pod2.txt", dacBuf);
        myNex.writeStr("dac2.txt", dacBuf);

        // SD INFO & LOGOWANIE
        if(SD.cardType() != CARD_NONE) {
             float gb = SD.totalBytes() / (1024.0*1024.0*1024.0);
             myNex.writeStr("sd.txt", String(gb, 1) + " GB");
             
             // Zapis do logu (tylko jak system ON, żeby nie śmiecić?)
             // Zmieniam na: ZAWSZE co 2s, żebyś widział historię
             if(millis() - lastLogTime >= 2000) {
                 lastLogTime = millis();
                 File f = SD.open(logFileName, FILE_APPEND);
                 if(f) {
                     f.print(millis()); f.print(";");
                     f.print(systemON); f.print(";");
                     f.print(i); f.print(";");
                     f.print(u); f.print(";");
                     f.print(p); f.print(";");
                     f.print(currentDac1); f.print(";");
                     f.println(currentDac2);
                     f.close(); // Zamykamy od razu!
                     Serial.println("[SD] Log zapisany.");
                 }
             }
        } else {
            myNex.writeStr("sd.txt", "NO SD");
        }
    }
}