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
#include <WiFi.h>              
#include <WebServer.h>         

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
#define PIN_POT_SYMULACJA 3 

// --- NOWY PIN: DIODA SYGNALIZACYJNA LED ---
#define PIN_LED 2 

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

WebServer server(80); 

// --- PARAMETRY PID ---
double Setpoint;
double Input;
double Output;
double Kp = 0.5, Ki = 0.1, Kd = 0.15; 
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// --- TWARDE LIMITY MASZYNY ---
const float MIN_DAC_VOLTAGE = 3.5; // 17.5 Hz
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
unsigned long lastAITime = 0;     
unsigned long resetPressTime = 0;
bool isResetPressed = false;

const float WSPOLCZYNNIK_DZIELNIKA = 1.982;
float filtr_waga = 0.15;

const int BUTTON_PIN = 0;
bool trybTestowy = false;
float current_Amps = 0.0;
float ai_sumAmps = 0.0;
int ai_samples = 0;

// Zmienne do obsługi WiFi
bool isWifiAPActive = false;

// ULEPSZONE ZMIENNE PRZYCISKU
unsigned long buttonPressTime = 0; 
unsigned long lastClickTime = 0;
int clickCount = 0;
bool buttonWasPressed = false;
const unsigned long CLICK_TIMEOUT = 800;    
const unsigned long LONG_PRESS_TIME = 3000; 

// Zmienne do płynnej sygnalizacji LED (NON-BLOCKING)
unsigned long ledTimer = 0;
int ledState = LOW;
int blinkCount = 0;
int blinkMax = 0;
int blinkDuration = 100; 

// ================================================================
// FUNKCJE POMOCNICZE I SYGNALIZACYJNE
// ================================================================
void triggerBlink(int times, int duration) {
    blinkMax = times * 2; 
    blinkCount = 0;
    blinkDuration = duration;
    ledState = HIGH;
    digitalWrite(PIN_LED, ledState);
    ledTimer = millis();
    blinkCount++;
}

void handleLED() {
    if (blinkCount > 0 && blinkCount < blinkMax) {
        if (millis() - ledTimer >= blinkDuration) {
            ledTimer = millis();
            ledState = !ledState;
            digitalWrite(PIN_LED, ledState);
            blinkCount++;
        }
    } else if (blinkCount >= blinkMax) {
        digitalWrite(PIN_LED, LOW); 
        blinkCount = 0;
    }
}

void scanI2C() {
    Serial.println("[I2C] Skanowanie magistrali...");
    byte error, address;
    int nDevices = 0; 
    for(address = 1; address < 127; address++ ) {
        Wire.beginTransmission(address); 
        error = Wire.endTransmission();  
        if (error == 0) {
            Serial.print("[I2C] Znaleziono: 0x");
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
    File f = SD.open("/AI_LOG.txt", FILE_APPEND); 
    if(f) {
        f.println("=== START SYSTEMU - ANALIZATOR TRENDOW ==="); 
        f.close(); 
        Serial.println("[SD] Aktywowano AI_LOG na karcie.");
    }
}

// ================================================================
// STRONY WWW (HTML + CSS + JS)
// ================================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pl">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Regulator PID - Panel</title>
  <style>
    body { background-color: #121212; color: #ffffff; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; text-align: center; margin: 0; padding: 10px; }
    h1 { color: #00bcd4; font-size: 24px; }
    .card { background-color: #1e1e1e; border-radius: 12px; padding: 20px; margin: 15px auto; max-width: 450px; box-shadow: 0 4px 8px rgba(0,0,0,0.3); }
    .data-row { display: flex; justify-content: space-between; font-size: 18px; margin-bottom: 10px; border-bottom: 1px solid #333; padding-bottom: 5px;}
    .data-val { font-weight: bold; color: #4caf50; }
    label { display: block; text-align: left; font-size: 14px; color: #aaa; margin-top: 15px; }
    input[type="number"] { width: 100%; padding: 12px; margin-top: 5px; background: #2a2a2a; border: 1px solid #444; color: white; border-radius: 6px; font-size: 16px; box-sizing: border-box;}
    button { background-color: #00bcd4; color: #000; border: none; padding: 15px 20px; font-size: 18px; font-weight: bold; border-radius: 8px; cursor: pointer; width: 100%; margin-top: 20px; }
    button:hover { background-color: #0097a7; }
    .btn-danger { background-color: #f44336; color: white; margin-top: 10px; }
  </style>
</head>
<body>
  <h1>🛠️ Panel Serwisowy Granulatora</h1>
  
  <div class="card">
    <div class="data-row"><span>Prąd Maszyny:</span> <span class="data-val" id="amp">-- A</span></div>
    <div class="data-row"><span>Limit (Max):</span> <span class="data-val" id="maxL">-- A</span></div>
    <div class="data-row"><span>Cel PID (Setpoint):</span> <span class="data-val" id="setp">-- A</span></div>
    <div class="data-row" style="border:none;"><span>Wyjście DAC (Falownik):</span> <span class="data-val" style="color:#ff9800;" id="dac">-- V</span></div>
  </div>

  <div class="card">
    <h3 style="margin-top:0; border-bottom: 1px solid #333; padding-bottom:10px;">Strojenie Algorytmu PID</h3>
    <form action="/update_pid" method="POST">
      <label><b>P (Proporcjonalny)</b> - Siła natychmiastowej reakcji na błąd.</label>
      <input type="number" step="0.01" name="kp" id="kp" required>
      
      <label><b>I (Całkujący)</b> - Wyrównywanie w czasie do punktu docelowego.</label>
      <input type="number" step="0.01" name="ki" id="ki" required>
      
      <label><b>D (Różniczkujący)</b> - Przewidywanie nagłych skoków (Amortyzator).</label>
      <input type="number" step="0.01" name="kd" id="kd" required>
      
      <button type="submit">💾 ZAPISZ PARAMETRY</button>
    </form>
  </div>

  <div class="card" style="background: transparent; box-shadow: none;">
    <button class="btn-danger" onclick="window.location.href='/ota'">📥 PRZEJDŹ DO AKTUALIZACJI (OTA)</button>
  </div>

  <script>
    setInterval(function() {
      fetch('/api/data').then(response => response.json()).then(data => {
        document.getElementById('amp').innerText = data.amp + " A";
        document.getElementById('maxL').innerText = data.maxL + " A";
        document.getElementById('setp').innerText = data.setp + " A";
        document.getElementById('dac').innerText = data.dac + " V";
        
        if(!document.getElementById('kp').value) document.getElementById('kp').value = data.kp;
        if(!document.getElementById('ki').value) document.getElementById('ki').value = data.ki;
        if(!document.getElementById('kd').value) document.getElementById('kd').value = data.kd;
      });
    }, 1000);
  </script>
</body>
</html>
)rawliteral";

const char OTA_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pl">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Aktualizacja OTA</title>
  <style>
    body { background-color: #121212; color: #ffffff; font-family: sans-serif; text-align: center; padding: 20px; }
    .card { background-color: #1e1e1e; border-radius: 12px; padding: 30px; margin: 20px auto; max-width: 400px; }
    button { background-color: #4caf50; color: #fff; border: none; padding: 15px; width: 100%; font-size: 16px; border-radius: 8px; margin-top:20px;}
    .back { background-color: #555; margin-top: 10px; }
  </style>
</head>
<body>
  <div class="card">
    <h2>Menedżer Aktualizacji (OTA)</h2>
    <p style="color:#aaa;">Miejsce na przyszla implementacje zrzutu pliku .bin.</p>
    <br><br>
    <button disabled style="background:#333; color:#666;">Wybierz plik .bin (Wkrotce)</button>
    <button class="back" onclick="window.location.href='/'">Powrót do panelu</button>
  </div>
</body>
</html>
)rawliteral";

// ================================================================
// FUNKCJE SERWERA WWW
// ================================================================
void handleRoot() { server.send(200, "text/html", INDEX_HTML); }
void handleOTA() { server.send(200, "text/html", OTA_HTML); }

void handleApiData() {
    String json = "{";
    json += "\"amp\":\"" + String(current_Amps, 2) + "\",";
    json += "\"maxL\":\"" + String(maxLimit, 1) + "\",";
    json += "\"setp\":\"" + String(Setpoint, 2) + "\",";
    json += "\"dac\":\"" + String(Output, 2) + "\",";
    json += "\"kp\":\"" + String(Kp, 3) + "\",";
    json += "\"ki\":\"" + String(Ki, 3) + "\",";
    json += "\"kd\":\"" + String(Kd, 3) + "\"";
    json += "}";
    server.send(200, "application/json", json);
}

void handleUpdatePID() {
    if (server.hasArg("kp") && server.hasArg("ki") && server.hasArg("kd")) {
        Kp = server.arg("kp").toFloat();
        Ki = server.arg("ki").toFloat();
        Kd = server.arg("kd").toFloat();
        
        memory.putFloat("kp", Kp);
        memory.putFloat("ki", Ki);
        memory.putFloat("kd", Kd);
        
        myPID.SetTunings(Kp, Ki, Kd);
        
        Serial.printf("[WiFi] Zmieniono parametry PID: P=%.3f, I=%.3f, D=%.3f\n", Kp, Ki, Kd);
        triggerBlink(1, 1000); // 1 sekunda sygnalizacji
    }
    server.sendHeader("Location", "/");
    server.send(303); 
}

void toggleWiFi() {
    if (!isWifiAPActive) {
        IPAddress local_ip(192, 168, 5, 1);
        IPAddress gateway(192, 168, 5, 1);
        IPAddress subnet(255, 255, 255, 0);
        WiFi.softAPConfig(local_ip, gateway, subnet);
        
        WiFi.softAP("RegulatorPID", "regpid");
        server.begin();
        isWifiAPActive = true;
        
        Serial.println("\n=======================================");
        Serial.println("[WIFI] SIEC UTWORZONA!");
        Serial.println("[WIFI] Nazwa: RegulatorPID");
        Serial.println("[WIFI] Haslo: regpid");
        Serial.println("[WIFI] Adres w przegladarce: 192.168.5.1");
        Serial.println("=======================================\n");

        triggerBlink(3, 100); // 3 migniecia = wlaczono
    } else {
        server.stop();
        WiFi.softAPdisconnect(true);
        isWifiAPActive = false;
        Serial.println("\n[WIFI] Siec WYLACZONA.\n");
        
        triggerBlink(1, 500); // 1 dlugie = wylaczono
    }
}

void onStationConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("\n[WIFI] Nowe urzadzenie (Telefon) polaczone z maszyna!");
    triggerBlink(2, 200); 
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
    if (Output < MIN_DAC_VOLTAGE) Output = MIN_DAC_VOLTAGE;
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
}

void stopRegulator() {
    systemON = false;                      
    myNex.writeStr("pidonoff.txt", "OFF"); 
    digitalWrite(PIN_RELAY_1, RELAY_OFF);  
    digitalWrite(PIN_RELAY_2, RELAY_OFF);  
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
// PARSER DOTYKU EKRANU
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
    pinMode(PIN_POT_SYMULACJA, INPUT); 
    pinMode(PIN_LED, OUTPUT); 
    digitalWrite(PIN_LED, LOW);

    // Dajemy czas na zainicjowanie wirtualnego portu USB COM 
    delay(2000); 
    Serial.begin(115200); 
    Serial.setTxTimeoutMs(0); 
    Serial.println("\n\n--- SYSTEM V11.0 (PRO WIFI + LED Feedback) ---");

    WiFi.onEvent(onStationConnected, ARDUINO_EVENT_WIFI_AP_STACONNECTED);

    memory.begin("regulator", false); 
    minLimit = memory.getFloat("minLim", 10.0); 
    maxLimit = memory.getFloat("maxLim", 40.0); 
    Kp = memory.getFloat("kp", 0.5);
    Ki = memory.getFloat("ki", 0.1);
    Kd = memory.getFloat("kd", 0.15);
    myPID.SetTunings(Kp, Ki, Kd); 

    server.on("/", HTTP_GET, handleRoot);
    server.on("/api/data", HTTP_GET, handleApiData);
    server.on("/update_pid", HTTP_POST, handleUpdatePID);
    server.on("/ota", HTTP_GET, handleOTA);

    NextionSerial.begin(9600, SERIAL_8N1, PIN_NEXT_RX, PIN_NEXT_TX);
    myNex.begin(9600);
    PzemSerial.begin(9600, SERIAL_8N1, PIN_PZEM_RX, PIN_PZEM_TX);
    dht.begin();
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL); 
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
    myPID.SetOutputLimits(MIN_DAC_VOLTAGE, MAX_DAC_VOLTAGE);   
    myPID.SetSampleTime(200); 
    
    Serial.println("SYSTEM GOTOWY - Odpal Monitor Szeregowy!");
    triggerBlink(2, 500); 
}

// ================================================================
// GŁÓWNA PĘTLA PROGRAMU
// ================================================================
void loop() {
    handleLED(); 

    if (isWifiAPActive) {
        server.handleClient();
    }

    handleNextionInput(); 

    // --- ULEPSZONY MULTI-KLIK PRZYCISKU BOOT ---
    int buttonState = digitalRead(BUTTON_PIN);
    if (buttonState == LOW && !buttonWasPressed) {
        buttonPressTime = millis();
        buttonWasPressed = true;
    } else if (buttonState == HIGH && buttonWasPressed) {
        buttonWasPressed = false;
        unsigned long pressDuration = millis() - buttonPressTime;

        if (pressDuration >= LONG_PRESS_TIME) {
            trybTestowy = !trybTestowy; 
            Serial.print("\n[MAGIA] Potencjometr -> PZEM: ");
            Serial.println(trybTestowy ? "ON" : "OFF");
            triggerBlink(8, 50); 
            clickCount = 0; 
        } else if (pressDuration > 20) { 
            clickCount++;
            lastClickTime = millis();
        }
    }

    if (clickCount > 0 && (millis() - lastClickTime) > CLICK_TIMEOUT) {
        if (clickCount >= 3) { 
            toggleWiFi();
        }
        clickCount = 0; 
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
            if (isnan(Output)) Output = MIN_DAC_VOLTAGE; 
            currentDac1 = Output;             
            currentDac2 = currentDac1 * 0.90; 
        }

        uint16_t mv_dac1 = (currentDac1 <= 10.0) ? (currentDac1 * 1000) : 10000;
        uint16_t mv_dac2 = (currentDac2 <= 10.0) ? (currentDac2 * 1000) : 10000;
        dac.setDACOutVoltage(mv_dac1, 0); 
        dac.setDACOutVoltage(mv_dac2, 1);
    }

    // --- PĘTLA PID (200ms) ---
    if (millis() - lastPIDTime >= 200) {
        lastPIDTime = millis();
        float i = pzem.current();
        if (isnan(i)) i = 0.0;
        if (trybTestowy) {
            int pot_raw = analogRead(PIN_POT_SYMULACJA); 
            i = (pot_raw / 4095.0) * 50.0; 
        }
        current_Amps = i; 
        
        if (systemON && current_Amps >= (maxLimit + 7.0)) {
            Serial.println("[ALARM] Przekroczono limit +7A! Awaryjne rozlaczenie!");
            stopRegulator();
            trippedByOverload = true;
        }

        if (!systemON && trippedByOverload && modeAUTO) {
            if (current_Amps <= (minLimit + 2.0)) {
                Serial.println("[AUTO] Prad wrocil do normy. Odpalam maszyne ponownie.");
                startRegulator(); 
            }
        }

        if (systemON) {
            Setpoint = maxLimit - 0.5; 
            Input = current_Amps;         
            myPID.Compute();   
        }
    }

    // --- WOLNA PĘTLA EKRANU I SERIAL LOGGERA (1000ms) ---
    if (millis() - lastUpdate >= 1000) {
        lastUpdate = millis(); 
        float u = pzem.voltage();
        float p = pzem.power();
        float pf = pzem.pf();

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
        }

        char dacBuf[10];
        sprintf(dacBuf, "%.2f V", currentDac1); myNex.writeStr("pod1.txt", dacBuf); myNex.writeStr("dac1.txt", dacBuf);
        sprintf(dacBuf, "%.2f V", currentDac2); myNex.writeStr("pod2.txt", dacBuf); myNex.writeStr("dac2.txt", dacBuf);

        // ZAPIS CSV BEZPOŚREDNIO DO PORTU SZEREGOWEGO
        Serial.printf("%lu;%d;%d;%d;%.3f;%.1f;%.0f;%.2f;%.1f;%.1f;%.2f;%.2f;%.2f;%.2f\n", 
            millis(), systemON, modeAUTO, trippedByOverload, current_Amps, u, p, napiecieZadajnika, minLimit, maxLimit, Setpoint, Output, currentDac1, currentDac2);
    }
}