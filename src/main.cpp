#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <PZEM004Tv30.h>
#include <EasyNextionLibrary.h>
#include <DFRobot_GP8403.h>
#include <PID_v1.h>
#include <DHT.h>

// ================================================================
// 1. PINOLOGIA (Hardcoded - SPRAWDZONE)
// ================================================================
// PZEM - Piny 4 i 5 (HardwareSerial 2)
#define PIN_PZEM_RX     4   // TX Modułu -> Pin 4 ESP
#define PIN_PZEM_TX     5   // RX Modułu -> Pin 5 ESP

// NEXTION - Piny 13 i 14 (HardwareSerial 1)
#define PIN_NEXT_RX     13
#define PIN_NEXT_TX     14

// DHT11 - Pin 20
#define PIN_DHT         20
#define DHTTYPE         DHT11

// SD CARD
#define PIN_SD_CS       15
#define PIN_SD_SCK      16
#define PIN_SD_MOSI     17
#define PIN_SD_MISO     18

// DAC
#define PIN_DAC_SDA     41
#define PIN_DAC_SCL     40

// ================================================================
// 2. OBIEKTY
// ================================================================
HardwareSerial NextionSerial(1);
HardwareSerial PzemSerial(2);

// Używamy EasyNex tylko do wysyłania (writeStr)
EasyNex myNex(NextionSerial); 

PZEM004Tv30 pzem(PzemSerial, PIN_PZEM_RX, PIN_PZEM_TX);
DFRobot_GP8403 dac(&Wire, 0x58);
DHT dht(PIN_DHT, DHTTYPE);

// PID
double Setpoint = 5.0, Input, Output;
PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);

// Zmienne Systemowe
float minLimit = 10.0;
float maxLimit = 40.0;
bool systemON = false;
bool modeAUTO = true;

// Timery
unsigned long lastUpdate = 0;

// Obsługa przytrzymania przycisku
int activeButtonID = 0;       // Który przycisk jest trzymany?
unsigned long buttonHoldTimer = 0; // Czas od ostatniej zmiany
bool isButtonHeld = false;    // Czy przycisk jest trzymany?

// ================================================================
// 3. FUNKCJE POMOCNICZE
// ================================================================

void updateSettingsScreen() {
    myNex.writeStr("min.txt", String(minLimit, 1));
    myNex.writeStr("max.txt", String(maxLimit, 1));
}

void togglePID() {
    systemON = !systemON;
    myNex.writeStr("pidonoff.txt", systemON ? "ON" : "OFF");
    Serial.println(systemON ? "PID: ON" : "PID: OFF");
}

void toggleMode() {
    modeAUTO = !modeAUTO;
    myNex.writeStr("pracaautoman.txt", modeAUTO ? "AUT" : "MAN");
    Serial.println(modeAUTO ? "TRYB: AUTO" : "TRYB: MANUAL");
}

// ================================================================
// 4. LOGIKA PRZYCISKÓW (PRESS & HOLD)
// ================================================================
void processButtonAction(int id) {
    // Ta funkcja wykonuje się raz przy kliknięciu, lub cyklicznie przy trzymaniu
    
    if (id == 1) { // PLUS MIN
        minLimit += 0.1;
        if(minLimit > maxLimit) minLimit = maxLimit; // Zabezpieczenie
        updateSettingsScreen();
    }
    if (id == 2) { // MINUS MIN
        minLimit -= 0.1;
        if(minLimit < 0) minLimit = 0;
        updateSettingsScreen();
    }
    if (id == 3) { // PLUS MAX
        maxLimit += 0.1;
        if(maxLimit > 100) maxLimit = 100;
        updateSettingsScreen();
    }
    if (id == 4) { // MINUS MAX
        maxLimit -= 0.1;
        if(maxLimit < minLimit) maxLimit = minLimit;
        updateSettingsScreen();
    }
}

// Główny Parser Nextiona
void handleNextionInput() {
    while (NextionSerial.available()) {
        byte b = NextionSerial.read();
        
        // Szukamy nagłówka 0x65 (Touch Event)
        if (b == 0x65) {
            delay(5); // Krótkie czekanie na resztę
            if (NextionSerial.available() >= 3) {
                byte pageId = NextionSerial.read();
                byte cmpId  = NextionSerial.read();
                byte event  = NextionSerial.read(); // 0x01=Press, 0x00=Release
                
                // Czyścimy resztę (FF FF FF)
                while (NextionSerial.available()) NextionSerial.read();

                // ---------------- LOGIKA ----------------
                
                // STRONA 0 (GŁÓWNA) - Tu nie ma trzymania, tylko klik
                if (pageId == 0 && event == 0x01) { // Tylko przy naciśnięciu
                    if (cmpId == 11) togglePID();   // Przycisk PID
                    if (cmpId == 12) toggleMode();  // Przycisk AUTO/MAN
                    if (cmpId == 8)  ESP.restart(); // RESET
                }

                // STRONA 2 (USTAWIENIA) - Tu obsługujemy trzymanie
                if (pageId == 2) {
                    if (event == 0x01) { 
                        // WCIŚNIĘCIE (PRESS)
                        activeButtonID = cmpId;
                        isButtonHeld = true;
                        processButtonAction(activeButtonID); // Wykonaj od razu raz
                        buttonHoldTimer = millis() + 400;    // Czekaj 400ms zanim zaczniesz przewijać
                    }
                    else if (event == 0x00) {
                        // PUSZCZENIE (RELEASE)
                        activeButtonID = 0;
                        isButtonHeld = false;
                    }
                }
            }
        }
    }

    // Obsługa "Trzymania" (Auto-scroll)
    if (isButtonHeld && activeButtonID > 0) {
        if (millis() > buttonHoldTimer) {
            processButtonAction(activeButtonID);
            buttonHoldTimer = millis() + 100; // Szybkość przewijania (100ms)
        }
    }
}

// ================================================================
// 5. SETUP
// ================================================================
void setup() {
    delay(1000); 
    Serial.begin(115200);
    Serial.println("\n\n--- ALIENWARE SYSTEM v4.0 (HOLD FIX) ---");

    // 1. NEXTION
    NextionSerial.begin(9600, SERIAL_8N1, PIN_NEXT_RX, PIN_NEXT_TX);
    myNex.begin(9600);
    
    // 2. PZEM (Powrót do metody manualnej)
    PzemSerial.begin(9600, SERIAL_8N1, PIN_PZEM_RX, PIN_PZEM_TX);
    delay(200); 
    
    // 3. DHT
    dht.begin();

    // 4. DAC
    Wire.begin(PIN_DAC_SDA, PIN_DAC_SCL);
    if(dac.begin() != 0) {
        Serial.println("DAC: Błąd I2C");
    } else {
        dac.setDACOutRange(dac.eOutputRange10V);
        dac.setDACOutVoltage(0, 0);
    }

    // 5. SD
    SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
    SD.begin(PIN_SD_CS);

    // Domyślne stany
    myPID.SetMode(AUTOMATIC);
    myNex.writeStr("pidonoff.txt", "OFF");
    myNex.writeStr("pracaautoman.txt", "AUT");
    
    Serial.println("SYSTEM GOTOWY");
}

// ================================================================
// 6. LOOP
// ================================================================
void loop() {
    // 1. NON-STOP: Obsługa dotyku
    handleNextionInput();

    // 2. CO 1 SEKUNDĘ: Odczyt i Ekran
    if (millis() - lastUpdate >= 1000) {
        lastUpdate = millis();

        // PZEM
        float u = pzem.voltage();
        float i = pzem.current();
        float p = pzem.power();
        float pf = pzem.pf();
        
        // DHT
        float t = dht.readTemperature();
        float h = dht.readHumidity();

        // LOGIKA BŁĘDÓW I WYSYŁKA
        if (isnan(u)) {
            myNex.writeStr("napgr.txt", "ERR"); 
            // Jeśli PZEM milczy, nie zeruj wszystkiego, tylko pokaż błąd napięcia
        } else {
            // Obliczenia
            float s = u * i; 
            float q = 0.0;
            if (s > p) q = sqrt(s*s - p*p);

            char buf[16];
            
            // Strona 0
            sprintf(buf, "%.3f A", i);
            myNex.writeStr("granampery.txt", buf);
            
            // Strona 7
            sprintf(buf, "%.1f V", u);
            myNex.writeStr("napgr.txt", buf);
            sprintf(buf, "%.3f A", i);
            myNex.writeStr("natgr.txt", buf);
            sprintf(buf, "%.0f W", p);
            myNex.writeStr("mocczy.txt", buf);
            sprintf(buf, "%.0f VA", s);
            myNex.writeStr("mocpoz.txt", buf);
            sprintf(buf, "%.0f Var", q);
            myNex.writeStr("mocbie.txt", buf);
            sprintf(buf, "%.2f", pf);
            myNex.writeStr("wspmoc.txt", buf);

            myNex.writeStr("logi0.txt", "OK");
        }

        if (!isnan(t)) {
            myNex.writeStr("temperatura.txt", String(t, 1));
            myNex.writeStr("wilgotnosc.txt", String(h, 0));
        }
        
        // DAC (Status)
        char dacBuf[10];
        sprintf(dacBuf, "%.2f V", systemON ? 5.00 : 0.00);
        myNex.writeStr("pod1.txt", dacBuf);
        myNex.writeStr("dac1.txt", dacBuf);
        
        // SD Info
        if(SD.cardType() != CARD_NONE) {
            float gb = SD.totalBytes() / (1024.0*1024.0*1024.0);
            myNex.writeStr("sd.txt", String(gb, 1) + " GB");
        }
    }
}