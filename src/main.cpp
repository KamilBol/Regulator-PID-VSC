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
// PINOLOGIA (Nadawanie przyjaznych nazw numerom pinów)
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

// --- NOWY PIN: POTENCJOMETR DO SYMULACJI PZEM ---
#define PIN_POT_SYMULACJA 3 // Wolny pin ADC na ESP32-S3 (odczyt 0-3.3V)

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

// Obiekty i zmienne do obsługi regulatora PID
double Setpoint;
double Input;
double Output;
// BARDZO ZŁAGODZONE NASTAWY (Koniec z zero-jedynkowym szarpaniem!)
// P=0.2, I=0.1, D=0.0
PID myPID(&Input, &Output, &Setpoint, 0.2, 0.1, 0.0, DIRECT);

// ================================================================
// ZMIENNE GLOBALNE
// ================================================================
float minLimit = 10.0;
float maxLimit = 40.0;
bool systemON = false;
bool modeAUTO = true;

float napiecieZadajnika = 0.0;
float currentDac1 = 0.0;
float currentDac2 = 0.0;
String logFileName = "/log_000.csv";

unsigned long lastUpdate = 0;
unsigned long lastLogTime = 0;
unsigned long lastFastUpdate = 0;
unsigned long resetPressTime = 0;
bool isResetPressed = false;

const float WSPOLCZYNNIK_DZIELNIKA = 1.982;
float filtr_waga = 0.15;

// --- ZMIENNE DO SYMULACJI (MAGIA POTENCJOMETR -> PZEM) ---
const int BUTTON_PIN = 0;
bool trybTestowy = false;
unsigned long buttonPressTime_Magia = 0; 
bool isButtonPressed_Magia = false;
const unsigned long LONG_PRESS_TIME = 3000;

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
    if (nDevices == 0) Serial.println("[I2C] Brak urzadzen! Sprawdz kable.");
    else Serial.println("[I2C] Koniec skanowania.");
}

void initSD() {
    SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS); 
    if(!SD.begin(PIN_SD_CS)) { 
        Serial.println("[SD] BLAD INICJALIZACJI! (Sprawdz format FAT32)");
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
        f.println("Czas;PID;Prad;Nap;Moc;Zadajnik;DAC1;DAC2"); 
        f.close(); 
        Serial.println("[SD] Utworzono plik: " + logFileName);
    }
}

// ================================================================
// LOGIKA STEROWANIA SYSTEMEM
// ================================================================
void startRegulator() {
    systemON = true;                      
    myNex.writeStr("pidonoff.txt", "ON"); 

    // --- WYRÓWNANIE Z ZADAJNIKIEM PRZED WŁĄCZENIEM ---
    myPID.SetMode(MANUAL);          // Chwilowo usypiamy PID
    Output = napiecieZadajnika;     // Pobieramy aktualny stan maszyny
    currentDac1 = Output;
    currentDac2 = currentDac1 * 0.90;

    // Od razu wymuszamy to napięcie na fizycznych wyjściach DAC
    uint16_t mv_dac1 = (currentDac1 <= 10.0) ? (currentDac1 * 1000) : 10000;
    uint16_t mv_dac2 = (currentDac2 <= 10.0) ? (currentDac2 * 1000) : 10000;
    dac.setDACOutVoltage(mv_dac1, 0); 
    dac.setDACOutVoltage(mv_dac2, 1);
    
    delay(50); // Czekamy 50ms, żeby prąd w elektronice się ustabilizował

    // Dopiero teraz załączamy potężne przekaźniki
    digitalWrite(PIN_RELAY_1, RELAY_ON);  
    digitalWrite(PIN_RELAY_2, RELAY_ON);  

    myPID.SetMode(AUTOMATIC);       // Budzimy PID - przejmuje kontrolę dokładnie z tego miejsca
    // -------------------------------------------------------------
    
    Serial.println("[SYS] START REGULATORA (Wyrównano z Zadajnikiem)");
}

void stopRegulator() {
    systemON = false;                      
    myNex.writeStr("pidonoff.txt", "OFF"); 
    digitalWrite(PIN_RELAY_1, RELAY_OFF);  
    digitalWrite(PIN_RELAY_2, RELAY_OFF);  
    Serial.println("[SYS] STOP REGULATORA (Powrot do trybu recznego)");
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
                        activeButtonID = cmpId; 
                        isButtonHeld = true; 
                        processButtonAction(activeButtonID); 
                        buttonHoldTimer = millis() + 400;    
                    }
                    else if (event == 0x00) { 
                        activeButtonID = 0; 
                        isButtonHeld = false; 
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
        delay(500); 
        ESP.restart(); 
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
    pinMode(PIN_POT_SYMULACJA, INPUT); // PIN potencjometru jako wejście

    delay(1000); 
    Serial.begin(115200); 
    Serial.println("\n--- SYSTEM V9.5 (Potencjometr 0-50A + Window PID) ---");

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
    if (!ads.begin(0x48)) {
        Serial.println("[ADS] Blad komunikacji! Sprawdz adres 0x48.");
    } else {
        Serial.println("[ADS] Polaczono poprawnie.");
    }

    if(dac.begin() != 0) {
        Serial.println("[DAC] Blad komunikacji! (Adres inny niz 0x58?)");
    } else {
        Serial.println("[DAC] Polaczono poprawnie.");
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
    myPID.SetOutputLimits(0.0, 10.0);   
    myPID.SetSampleTime(1000);          
    
    Serial.println("SYSTEM GOTOWY");
}

// ================================================================
// GŁÓWNA PĘTLA PROGRAMU
// ================================================================
void loop() {
    handleNextionInput();

    int buttonState = digitalRead(BUTTON_PIN);
    if (buttonState == LOW && !isButtonPressed_Magia) {
        buttonPressTime_Magia = millis();
        isButtonPressed_Magia = true;
    } else if (buttonState == HIGH && isButtonPressed_Magia) {
        isButtonPressed_Magia = false;
        if (millis() - buttonPressTime_Magia >= LONG_PRESS_TIME) {
            trybTestowy = !trybTestowy; 
            Serial.print("[MAGIA] Tryb testowy (POTENCJOMETR -> PZEM) to teraz: ");
            Serial.println(trybTestowy ? "ON" : "OFF");
        }
    }

    // --- SZYBKA PĘTLA (50ms) ---
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
            if (isnan(Output)) Output = 0.0;
            currentDac1 = Output;             
            currentDac2 = currentDac1 * 0.90; 
        }

        uint16_t mv_dac1 = (currentDac1 <= 10.0) ? (currentDac1 * 1000) : 10000;
        uint16_t mv_dac2 = (currentDac2 <= 10.0) ? (currentDac2 * 1000) : 10000;

        dac.setDACOutVoltage(mv_dac1, 0); 
        dac.setDACOutVoltage(mv_dac2, 1);
    }

    // --- WOLNA PĘTLA (1000ms) ---
    if (millis() - lastUpdate >= 1000) {
        lastUpdate = millis(); 

        float u = pzem.voltage();
        float i = pzem.current();
        float p = pzem.power();
        float pf = pzem.pf();
        float t = dht.readTemperature();
        float h = dht.readHumidity();

        if (isnan(u)) u = 0.0;
        if (isnan(i)) i = 0.0;
        if (isnan(p)) p = 0.0;
        if (isnan(pf)) pf = 0.0;

        // --- PROTEZA MAGII: NADPISYWANIE PRĄDU POTENCJOMETREM ---
        if (trybTestowy) {
            int pot_raw = analogRead(PIN_POT_SYMULACJA); // Odczyt sprzętowy 0-4095
            // Przeliczamy wartość cyfrową płynnie na Ampery od 0.000A do 50.000A
            i = (pot_raw / 4095.0) * 50.0; 
        }
        // --------------------------------------------------------

        // --- INTELIGENTNA STREFA BEZPIECZNA (Wczesne reagowanie) ---
        // Wyliczamy strefę wczesnego reagowania (np. 15% z zakresu pomiędzy Min i Max)
        float margines = (maxLimit - minLimit) * 0.15; 

        if (i > (maxLimit - margines)) {
            // Prąd zbliża się do sufitu -> PID ma łagodnie utrzymać go na granicy
            Setpoint = maxLimit - margines;
        } else if (i < (minLimit + margines)) {
            // Prąd zbliża się do podłogi -> PID ma go delikatnie podbić
            Setpoint = minLimit + margines;
        } else {
            // Prąd jest idealnie w środku. Zamrażamy PID! Niech płynie naturalnie.
            Setpoint = i; 
        }

        Input = i;         
        myPID.Compute();   
        // -----------------------------------------------------------------------

        if(!isnan(u) || trybTestowy) { 
            float s = u * i; 
            float q = (s > p) ? sqrt(s*s - p*p) : 0; 
            
            char buf[16]; 
            sprintf(buf, "%.3f A", i); 
            myNex.writeStr("granampery.txt", buf); 
            myNex.writeStr("natgr.txt", buf);
            
            sprintf(buf, "%.1f V", u); myNex.writeStr("napgr.txt", buf);
            sprintf(buf, "%.0f W", p); myNex.writeStr("mocczy.txt", buf);
            sprintf(buf, "%.0f VA", s); myNex.writeStr("mocpoz.txt", buf);
            sprintf(buf, "%.0f Var", q); myNex.writeStr("mocbie.txt", buf);
            sprintf(buf, "%.2f", pf); myNex.writeStr("wspmoc.txt", buf);
            
            Serial.printf("[PID] I=%.3fA | Cel=%.1fA | Wyjscie DAC=%.2fV\n", i, Setpoint, Output);
        } else { 
            myNex.writeStr("napgr.txt", "ERR");
        }

        if(!isnan(t)) { 
            myNex.writeStr("temperatura.txt", String(t, 1));
            myNex.writeStr("wilgotnosc.txt", String(h, 0));
        }

        char dacBuf[10];
        sprintf(dacBuf, "%.2f V", currentDac1); 
        myNex.writeStr("pod1.txt", dacBuf);
        myNex.writeStr("dac1.txt", dacBuf);
        
        sprintf(dacBuf, "%.2f V", currentDac2);
        myNex.writeStr("pod2.txt", dacBuf);
        myNex.writeStr("dac2.txt", dacBuf);

        if(SD.cardType() != CARD_NONE) { 
             float gb = SD.totalBytes() / (1024.0*1024.0*1024.0); 
             myNex.writeStr("sd.txt", String(gb, 1) + " GB"); 
             
             if(millis() - lastLogTime >= 2000) { 
                 lastLogTime = millis();
                 File f = SD.open(logFileName, FILE_APPEND); 
                 if(f) {
                     f.print(millis()); f.print(";");
                     f.print(systemON); f.print(";");
                     f.print(i); f.print(";"); 
                     f.print(u); f.print(";");
                     f.print(p); f.print(";");
                     f.print(napiecieZadajnika); f.print(";");
                     f.print(currentDac1); f.print(";");
                     f.println(currentDac2); 
                     f.close(); 
                 }
             }
        } else {
            myNex.writeStr("sd.txt", "NO SD"); 
        }
    }
}