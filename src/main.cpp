// ================================================================
// DOŁĄCZANIE BIBLIOTEK (Narzędzia do obsługi modułów)
// ================================================================
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <PZEM004Tv30.h>
#include <EasyNextionLibrary.h>
#include <DFRobot_GP8403.h>
#include <Adafruit_ADS1X15.h>
#include <PID_v1.h>
#include <DHT.h>
#include <Preferences.h>

// ================================================================
// PINOLOGIA
// ================================================================
#define PIN_PZEM_RX     4
#define PIN_PZEM_TX     5
#define PIN_I2C_SDA     2
#define PIN_I2C_SCL     1
#define PIN_RELAY_1     47
#define PIN_RELAY_2     38
#define PIN_NEXT_RX     13
#define PIN_NEXT_TX     14
#define PIN_DHT         20
#define PIN_SD_CS       15
#define PIN_SD_SCK      16
#define PIN_SD_MOSI     17
#define PIN_SD_MISO     18
#define PIN_POT_SYMULACJA 3 // Potencjometr symulacji 0-3.3V (ADC)

#define RELAY_ON        LOW
#define RELAY_OFF       HIGH

// ================================================================
// TWORZENIE OBIEKTÓW
// ================================================================
HardwareSerial NextionSerial(1);
HardwareSerial PzemSerial(2);
EasyNex myNex(NextionSerial);
PZEM004Tv30 pzem(PzemSerial, PIN_PZEM_RX, PIN_PZEM_TX);
DHT dht(PIN_DHT, DHT11);
DFRobot_GP8403 dac(&Wire, 0x58);
Adafruit_ADS1115 ads;
Preferences memory;

// --- PARAMETRY PID ---
double Setpoint;
double Input;
double Output;
PID myPID(&Input, &Output, &Setpoint, 0.5, 0.1, 0.15, DIRECT);

// --- TWARDE LIMITY MASZYNY ---
const float MIN_DAC_VOLTAGE = 3.5; // 17.5 Hz z 50 Hz = 3.5V (Podłoga!)
const float MAX_DAC_VOLTAGE = 10.0;

// ================================================================
// ZMIENNE GLOBALNE
// ================================================================
float minLimit = 10.0;
float maxLimit = 40.0;
bool systemON = false;
bool modeAUTO = true;
bool trippedByOverload = false;

float napiecieZadajnika = 0.0;
float currentDac1 = 0.0;
float currentDac2 = 0.0;

unsigned long lastUpdate = 0;
unsigned long lastFastUpdate = 0;
unsigned long lastPIDTime = 0;
unsigned long lastAITime = 0;     // Zegar dla sztucznej inteligencji
unsigned long resetPressTime = 0;
bool isResetPressed = false;

const float WSPOLCZYNNIK_DZIELNIKA = 1.982;
float filtr_waga = 0.15;

const int BUTTON_PIN = 0;
bool trybTestowy = false;
unsigned long buttonPressTime_Magia = 0; 
bool isButtonPressed_Magia = false;
const unsigned long LONG_PRESS_TIME = 3000;

float current_Amps = 0.0;

// Zmienne do analizy "AI"
float ai_sumAmps = 0.0;
int ai_samples = 0;

// ================================================================
// FUNKCJE POMOCNICZE
// ================================================================
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
    if (nDevices == 0) Serial.println("[I2C] Brak urzadzen!");
}

void initSD() {
    SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS); 
    if(!SD.begin(PIN_SD_CS)) { 
        Serial.println("[SD] BLAD KARTY!");
        myNex.writeStr("sd.txt", "ERR"); 
        return; 
    }
    // Zapisujemy nagłówek dla logów "AI"
    File f = SD.open("/AI_LOG.txt", FILE_APPEND); 
    if(f) {
        f.println("=== START SYSTEMU - ANALIZATOR TRENDOW ==="); 
        f.close(); 
        Serial.println("[SD] Aktywowano AI_LOG na karcie.");
    }
}

// ================================================================
// LOGIKA STEROWANIA SYSTEMEM
// ================================================================
void startRegulator() {
    systemON = true;                      
    trippedByOverload = false; 
    myNex.writeStr("pidonoff.txt", "ON"); 

    myPID.SetMode(MANUAL);          
    Output = napiecieZadajnika;     

    // Zabezpieczenie miękkiego startu - jeśli zadajnik ma mniej niż 17.5Hz (3.5V), podbijamy
    if (Output < MIN_DAC_VOLTAGE) {
        Output = MIN_DAC_VOLTAGE;
        Serial.println("[SYS] Zadajnik mial za malo! Wymuszam 17.5Hz (3.5V) dla plynnego startu.");
    }

    currentDac1 = Output;
    currentDac2 = currentDac1 * 0.90;

    uint16_t mv_dac1 = (currentDac1 <= 10.0) ? (currentDac1 * 1000) : 10000;
    uint16_t mv_dac2 = (currentDac2 <= 10.0) ? (currentDac2 * 1000) : 10000;
    dac.setDACOutVoltage(mv_dac1, 0); 
    dac.setDACOutVoltage(mv_dac2, 1);
    
    delay(50); 
    digitalWrite(PIN_RELAY_1, RELAY_ON);  
    digitalWrite(PIN_RELAY_2, RELAY_ON);  

    myPID.SetMode(AUTOMATIC);       
    Serial.println("[SYS] START REGULATORA (Przejeto miekko)");
}

void stopRegulator() {
    systemON = false;                      
    myNex.writeStr("pidonoff.txt", "OFF"); 
    digitalWrite(PIN_RELAY_1, RELAY_OFF);  
    digitalWrite(PIN_RELAY_2, RELAY_OFF);  
    Serial.println("[SYS] STOP REGULATORA (Rozlaczono Przekazniki)");
}

void updateSettingsScreen() {
    myNex.writeStr("min.txt", String(minLimit, 1)); 
    myNex.writeStr("max.txt", String(maxLimit, 1)); 
}

void processButtonAction(int id) {
    if (id == 1) { minLimit += 0.1; if(minLimit > maxLimit) minLimit = maxLimit; }
    if (id == 2) { minLimit -= 0.1; if(minLimit < 0) minLimit = 0; }
    if (id == 3) { maxLimit += 0.1; if(maxLimit > 100) maxLimit = 100; }
    if (id == 4) { maxLimit -= 0.1; if(maxLimit < minLimit) maxLimit = minLimit; }
    
    updateSettingsScreen();
    memory.putFloat("minLim", minLimit);
    memory.putFloat("maxLim", maxLimit);
}

// ================================================================
// PARSER DOTYKU
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
                    if (event == 0x01) { 
                        activeButtonID = cmpId; isButtonHeld = true; 
                        processButtonAction(activeButtonID); 
                        buttonHoldTimer = millis() + 400;    
                    } else if (event == 0x00) { 
                        activeButtonID = 0; isButtonHeld = false; 
                    }
                }
            }
        }
    }
    if (isButtonHeld && activeButtonID > 0 && millis() > buttonHoldTimer) {
        processButtonAction(activeButtonID);
        buttonHoldTimer = millis() + 100; 
    }
    if (isResetPressed && (millis() - resetPressTime > 5000)) {
        myNex.writeStr("logi0.txt", "RESTART!"); 
        delay(500); ESP.restart(); 
    }
}

// ================================================================
// SEKCJA SETUP
// ================================================================
void setup() {
    pinMode(PIN_RELAY_1, OUTPUT);         
    pinMode(PIN_RELAY_2, OUTPUT);
    digitalWrite(PIN_RELAY_1, RELAY_OFF); 
    digitalWrite(PIN_RELAY_2, RELAY_OFF);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(PIN_POT_SYMULACJA, INPUT); 

    delay(1000); 
    Serial.begin(115200); 
    Serial.println("\n--- SYSTEM V9.7 (Min 17.5Hz + AI Trend + Serial Log) ---");
    Serial.println("Format Danych (wklej do Excela): Czas_ms;SystemON;Auto;Awaria;Prad_A;Nap_V;Moc_W;Zadajnik_V;MinLim;MaxLim;Cel_PID;Wyjscie_PID;DAC1;DAC2");

    memory.begin("regulator", false); 
    minLimit = memory.getFloat("minLim", 10.0); 
    maxLimit = memory.getFloat("maxLim", 40.0); 

    NextionSerial.begin(9600, SERIAL_8N1, PIN_NEXT_RX, PIN_NEXT_TX);
    myNex.begin(9600);
    PzemSerial.begin(9600, SERIAL_8N1, PIN_PZEM_RX, PIN_PZEM_TX);
    dht.begin();
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL); 
    scanI2C();                            
    
    ads.setGain(GAIN_TWOTHIRDS); 
    ads.begin(0x48);

    if(dac.begin() == 0) {
        dac.setDACOutRange(dac.eOutputRange10V); 
        dac.setDACOutVoltage(0, 0);              
        dac.setDACOutVoltage(0, 1);              
    }

    initSD();
    updateSettingsScreen();
    myNex.writeStr("pidonoff.txt", "OFF");
    myNex.writeStr("pracaautoman.txt", "AUT");
    stopRegulator(); 
    
    myPID.SetMode(AUTOMATIC);           
    // TWARDE OGRANICZENIE: Nie wolno zwalniać poniżej 3.5V (17.5 Hz)!
    myPID.SetOutputLimits(MIN_DAC_VOLTAGE, MAX_DAC_VOLTAGE);   
    myPID.SetSampleTime(200); 
    
    Serial.println("SYSTEM GOTOWY");
}

// ================================================================
// GŁÓWNA PĘTLA PROGRAMU
// ================================================================
void loop() {
    handleNextionInput();

    // --- MAGIA (POTENCJOMETR ON/OFF) ---
    int buttonState = digitalRead(BUTTON_PIN);
    if (buttonState == LOW && !isButtonPressed_Magia) {
        buttonPressTime_Magia = millis();
        isButtonPressed_Magia = true;
    } else if (buttonState == HIGH && isButtonPressed_Magia) {
        isButtonPressed_Magia = false;
        if (millis() - buttonPressTime_Magia >= LONG_PRESS_TIME) {
            trybTestowy = !trybTestowy; 
            Serial.print("[MAGIA] Potencjometr -> PZEM: ");
            Serial.println(trybTestowy ? "ON" : "OFF");
        }
    }

    // --- SZYBKA PĘTLA (50ms - Napięcia) ---
    if (millis() - lastFastUpdate >= 50) {
        lastFastUpdate = millis(); 

        int16_t adc_surowe = ads.readADC_SingleEnded(0); 
        float napiecie_na_pinie = ads.computeVolts(adc_surowe); 
        
        float aktualny_odczyt = napiecie_na_pinie * WSPOLCZYNNIK_DZIELNIKA; 
        if(aktualny_odczyt < 0.05) aktualny_odczyt = 0.0;
        if(aktualny_odczyt > 10.5) aktualny_odczyt = 10.5;

        if (napiecieZadajnika == 0.0 && aktualny_odczyt > 0.0) {
            napiecieZadajnika = aktualny_odczyt; 
        } else {
            napiecieZadajnika = (aktualny_odczyt * filtr_waga) + (napiecieZadajnika * (1.0 - filtr_waga));
        }

        if (!systemON) {
            currentDac1 = napiecieZadajnika;
            currentDac2 = currentDac1 * 0.90; 
        } else {
            if (isnan(Output)) Output = MIN_DAC_VOLTAGE; // Bezpieczeństwo
            currentDac1 = Output;             
            currentDac2 = currentDac1 * 0.90; 
        }

        uint16_t mv_dac1 = (currentDac1 <= 10.0) ? (currentDac1 * 1000) : 10000;
        uint16_t mv_dac2 = (currentDac2 <= 10.0) ? (currentDac2 * 1000) : 10000;

        dac.setDACOutVoltage(mv_dac1, 0); 
        dac.setDACOutVoltage(mv_dac2, 1);
    }

    // --- PĘTLA PID I KONTROLI AWARYJNEJ (200ms) ---
    if (millis() - lastPIDTime >= 200) {
        lastPIDTime = millis();

        float i = pzem.current();
        if (isnan(i)) i = 0.0;

        if (trybTestowy) {
            int pot_raw = analogRead(PIN_POT_SYMULACJA); 
            i = (pot_raw / 4095.0) * 50.0; 
        }
        
        current_Amps = i; 

        // Zbieranie danych dla sztucznej inteligencji
        if (systemON) {
            ai_sumAmps += current_Amps;
            ai_samples++;
        }

        // 1. ZABEZPIECZENIE ABSOLUTNE (Odcięcie + 7A)
        if (systemON && current_Amps >= (maxLimit + 7.0)) {
            Serial.println("[ALARM] Przekroczono limit +7A! Awaryjne rozlaczenie!");
            stopRegulator();
            trippedByOverload = true;
        }

        // 2. LOGIKA POWROTU W AUTO
        if (!systemON && trippedByOverload && modeAUTO) {
            if (current_Amps <= (minLimit + 2.0)) {
                Serial.println("[AUTO] Prad wrocil do normy. Odpalam maszyne ponownie.");
                startRegulator(); 
            }
        }

        // 3. OBLICZENIA PID: Zawsze dążymy do maksymalnego obciążenia
        if (systemON) {
            // PID chce zawsze utrzymać maszynę tuż pod limitem MAX (dajemy 0.5A marginesu oddechu)
            Setpoint = maxLimit - 0.5; 
            Input = current_Amps;         
            myPID.Compute();   
        }
    }

    // --- SZTUCZNA INTELIGENCJA NA SD (co 15 sekund) ---
    if (millis() - lastAITime >= 15000) {
        lastAITime = millis();
        if (systemON && ai_samples > 0) {
            float avg_Amps = ai_sumAmps / ai_samples;
            String wniosek = "";
            
            if (avg_Amps < (maxLimit - 10.0)) wniosek = "Maszyna mocno niedociazona. Sugerowane podniesienie MinLimit.";
            else if (avg_Amps < (maxLimit - 3.0)) wniosek = "Praca stabilna, ale mozna zwiekszyc posuw.";
            else wniosek = "Praca na pelnych obrotach. Parametry optymalne.";

            // Zapisz wniosek na SD
            if(SD.cardType() != CARD_NONE) {
                File f = SD.open("/AI_LOG.txt", FILE_APPEND);
                if(f) {
                    f.printf("Czas: %lu ms | Sredni prad: %.1f A | Cel: %.1f A | WNIOSEK: %s\n", millis(), avg_Amps, maxLimit, wniosek.c_str());
                    f.close();
                }
            }
            ai_sumAmps = 0.0; ai_samples = 0; // Reset próbek
        }
    }

    // --- WOLNA PĘTLA EKRANU I SERIAL LOGGERA (1000ms) ---
    if (millis() - lastUpdate >= 1000) {
        lastUpdate = millis(); 

        float u = pzem.voltage();
        float p = pzem.power();
        float pf = pzem.pf();
        float t = dht.readTemperature();
        float h = dht.readHumidity();

        if (isnan(u)) u = 0.0;
        if (isnan(p)) p = 0.0;
        if (isnan(pf)) pf = 0.0;

        if(!isnan(u) || trybTestowy) { 
            float s = u * current_Amps; 
            float q = (s > p) ? sqrt(s*s - p*p) : 0; 
            
            char buf[16]; 
            sprintf(buf, "%.3f A", current_Amps); 
            myNex.writeStr("granampery.txt", buf); myNex.writeStr("natgr.txt", buf);
            sprintf(buf, "%.1f V", u); myNex.writeStr("napgr.txt", buf);
            sprintf(buf, "%.0f W", p); myNex.writeStr("mocczy.txt", buf);
            sprintf(buf, "%.0f VA", s); myNex.writeStr("mocpoz.txt", buf);
            sprintf(buf, "%.0f Var", q); myNex.writeStr("mocbie.txt", buf);
            sprintf(buf, "%.2f", pf); myNex.writeStr("wspmoc.txt", buf);
        }

        if(!isnan(t)) { 
            myNex.writeStr("temperatura.txt", String(t, 1));
            myNex.writeStr("wilgotnosc.txt", String(h, 0));
        }

        char dacBuf[10];
        sprintf(dacBuf, "%.2f V", currentDac1); myNex.writeStr("pod1.txt", dacBuf); myNex.writeStr("dac1.txt", dacBuf);
        sprintf(dacBuf, "%.2f V", currentDac2); myNex.writeStr("pod2.txt", dacBuf); myNex.writeStr("dac2.txt", dacBuf);

        if(SD.cardType() != CARD_NONE) { 
             float gb = SD.totalBytes() / (1024.0*1024.0*1024.0); 
             myNex.writeStr("sd.txt", String(gb, 1) + " GB"); 
        } else {
            myNex.writeStr("sd.txt", "NO SD"); 
        }

        // --- ZAPIS CSV BEZPOŚREDNIO DO PORTU SZEREGOWEGO (Zamiast na SD) ---
        // Kopiujesz te linie i masz gotowego Excela!
        Serial.printf("%lu;%d;%d;%d;%.3f;%.1f;%.0f;%.2f;%.1f;%.1f;%.2f;%.2f;%.2f;%.2f\n", 
            millis(), systemON, modeAUTO, trippedByOverload, current_Amps, u, p, napiecieZadajnika, minLimit, maxLimit, Setpoint, Output, currentDac1, currentDac2);
    }
}