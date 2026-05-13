// =====================================================================================
// REGULATOR PID - V16.5 (SMART OTA, CZYSTY KOD, CHUNKING, FULL COMMENTS)
// =====================================================================================
#include <Arduino.h>
#include "strona_www.h" // Załączenie naszej zewnętrznej strony HTML/CSS/JS

// --- BIBLIOTEKI SPRZĘTOWE I SENSORY ---
#include <Wire.h>               // Komunikacja I2C (Zadajnik, Falowniki)
#include <SPI.h>                // Komunikacja SPI (Karta SD)
#include <SD.h>                 // Obsługa systemu plików na karcie SD
#include <PZEM004Tv30.h>        // Miernik parametrów sieci elektrycznej
#include <EasyNextionLibrary.h> // Obsługa fizycznego ekranu dotykowego Nextion
#include <DFRobot_GP8403.h>     // Przetwornik cyfrowo-analogowy (DAC) dla falowników
#include <Adafruit_ADS1X15.h>   // Precyzyjny przetwornik analogowo-cyfrowy (ADC) zadajnika
#include <PID_v1.h>             // Biblioteka matematyczna algorytmu PID
#include <DHT.h>                // Czujnik temperatury i wilgotności

// --- BIBLIOTEKI SIECIOWE I SYSTEMOWE ---
#include <Preferences.h>        // Zapisywanie ustawień w trwałej pamięci Flash (odpowiednik EEPROM)
#include <WiFi.h>               // Obsługa rdzenia WiFi
#include <WiFiClientSecure.h>   // Bezpieczny klient HTTPS (wymagany do GitHuba)
#include <ESPmDNS.h>            // Przyjazne adresy w sieci lokalnej (np. granulator.local)
#include <WebServer.h>          // Lokalny serwer strony WWW
#include <Update.h>             // Wbudowana biblioteka obsługująca nadpisywanie Flash (OTA)
#include <HTTPClient.h>         // Klient HTTP do pobierania plików
#include <PubSubClient.h>       // Klient protokołu MQTT (HiveMQ)

// =====================================================================================
// DEFINICJE PINÓW (PINOLOGIA)
// =====================================================================================
#define PIN_PZEM_RX       4
#define PIN_PZEM_TX       5
#define PIN_I2C_SDA       2
#define PIN_I2C_SCL       1
#define PIN_RELAY_1       47
#define PIN_RELAY_2       38
#define PIN_NEXT_RX       13
#define PIN_NEXT_TX       14
#define PIN_DHT           20
#define PIN_SD_CS         15
#define PIN_SD_SCK        16
#define PIN_SD_MOSI       17
#define PIN_SD_MISO       18
#define PIN_POT_SYMULACJA 3 
#define PIN_LED           48 

// Logika przekaźników (zależna od modułu - tu stan niski załącza przekaźnik)
#define RELAY_ON          LOW
#define RELAY_OFF         HIGH

// =====================================================================================
// INICJALIZACJA OBIEKTÓW GLOBALNYCH
// =====================================================================================
HardwareSerial NextionSerial(1);
HardwareSerial PzemSerial(2);

EasyNex myNex(NextionSerial);
PZEM004Tv30 pzem(PzemSerial, PIN_PZEM_RX, PIN_PZEM_TX);
DHT dht(PIN_DHT, DHT11);
DFRobot_GP8403 dac(&Wire, 0x58); // Adres I2C układu DAC to 0x58
Adafruit_ADS1115 ads;            // Domyślny adres I2C układu ADS to 0x48

Preferences memory;
WebServer server(80); 

WiFiClientSecure espClient; 
PubSubClient mqtt(espClient);

// =====================================================================================
// ZMIENNE GLOBALNE I USTAWIENIA
// =====================================================================================
// --- PARAMETRY PID ---
double Setpoint; 
double Input; 
double Output;
double Kp = 0.5;
double Ki = 0.1;
double Kd = 0.15; 
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
// --- NOWE PARAMETRY DYNAMIKI (ZWŁOKA I DEADBAND) ---
float delayTimeSek = 2.0;    // Czas zwłoki algorytmu w sekundach
float deadbandAmps = 1.0;    // Strefa nieczułości w Amperach

// --- KONFIGURACJA SPRZĘTOWA (FALOWNIKI) ---
int outMode = 0; 
float minDacVolt = 3.5; 
float maxDacVolt = 10.0; 
float dac1Calib = 0.0; 
float dac2Calib = 0.0;

// --- DANE LOGOWANIA (WIFI I CHMURA) ---
String routerSSID = ""; 
String routerPASS = "";
String mqtt_server = ""; 
String mqtt_user = ""; 
String mqtt_pass = ""; 
String mqtt_id = "Granulator_01";

// --- WIDEŁKI I ALARMY MASZYNY ---
float minLimit = 10.0; 
float maxLimit = 40.0; 
float dac1Ratio = 100.0; 
float dac2Ratio = 90.0;  
float overloadLimit = 7.0; 
float recoveryLimit = 2.0; 

// --- STANY LOGICZNE SYSTEMU ---
bool systemON = false; 
bool modeAUTO = true; 
bool trippedByOverload = false;
bool cloudEcoMode = false; // Tryb oszczędzania danych chmury

// --- STATUS ZDALNEJ AKTUALIZACJI Z GITHUBA (OTA) ---
int remoteOtaProgress = -1; // Wartość -1 oznacza, że pobieranie się nie toczy
String remoteOtaState = "";

// --- ZMIENNE POMOCNICZE, TIMERY I FILTRY ---
float napiecieZadajnika = 0.0; 
float currentDac1 = 0.0; 
float currentDac2 = 0.0;
// --- REJESTRATOR DIAGNOSTYCZNY (DATA LOGGER) ---
bool isLoggingActive = false;
unsigned long logEndTime = 0;
unsigned long lastLogWriteTime = 0;
String currentLogFileName = "/AI_DIAG.csv"; // Będzie dynamicznie nadpisywane datą

unsigned long lastUpdate = 0; 
unsigned long lastFastUpdate = 0;
unsigned long lastPIDTime = 0; 
unsigned long lastDiagnosticTime = 0;
unsigned long lastMqttReconnect = 0; 
unsigned long lastMqttPublish = 0;

unsigned long resetPressTime = 0; 
bool isResetPressed = false;
bool resetStage1 = false; 
bool resetStage2 = false; 
bool resetStage3 = false;
unsigned long factoryResetPressTime = 0; 
bool isFactoryResetPressed = false;

const float WSPOLCZYNNIK_DZIELNIKA = 1.982; // Fizyczny przelicznik dzielnika napięcia na zadajniku
float filtr_waga = 0.15;                    // Wygładzanie skoków zadajnika (Low-Pass Filter)

const int BUTTON_PIN = 0; 
bool trybTestowy = false; 
float current_Amps = 0.0;

bool isWifiAPActive = false; 
unsigned long buttonPressTime = 0; 
unsigned long lastClickTime = 0;
int clickCount = 0; 
bool buttonWasPressed = false; 
const unsigned long CLICK_TIMEOUT = 800; 
const unsigned long LONG_PRESS_TIME = 3000; 

unsigned long ledTimer = 0; 
int ledState = LOW; 
int blinkCount = 0; 
int blinkMax = 0; 
int blinkDuration = 100; 

// --- FLAGI ZDROWIA (DIAGNOSTYKA SENSORÓW) ---
bool statusDAC = false; 
bool statusADS = false; 
bool statusSD = false; 
bool statusPZEM = false;
unsigned long lastNextionResponseTime = 0;

float pzem_u = 0, pzem_p = 0, pzem_pf = 0, pzem_s = 0, pzem_q = 0;
float dht_t = 0, dht_h = 0;

// Konfiguracja wejścia sprzętowego:
// 0 = Odczyt napięcia (0-10V) z pinu A0 (Dzielnik 10k)
// 1 = Odczyt prądu (mA) z pinu A1 (Rezystor 250 Ohm)
int typZadajnika = 0;

// =====================================================================================
// DEKLARACJE WYPRZEDZAJĄCE (Zabezpieczenie kompilatora przed brakiem referencji)
// =====================================================================================
void startRegulator(); 
void stopRegulator(); 
void updateSettingsScreen(); 
bool isResetStage1Active(); 
void handleRestoreDefaults(); 
void toggleLocalWiFi(); 
void performRemoteOTA(String url); 
void publishSDList();
void handleNextionInput(); 
void processButtonAction(int id); 
void updateNextionEcoText();

// =====================================================================================
// FUNKCJE POMOCNICZE I SYSTEMOWE
// =====================================================================================

// Czyści wklejany adres serwera z niepotrzebnych przedrostków i portów
String cleanHostAddress(String host) {
    String clean = host; 
    clean.replace("http://", ""); 
    clean.replace("https://", "");
    int colonIndex = clean.indexOf(':'); 
    if (colonIndex > 0) {
        clean = clean.substring(0, colonIndex); 
    }
    clean.trim(); 
    return clean;
}

// Uruchamia sekwencję mignięć wbudowanej diody LED
void triggerBlink(int times, int duration) {
    blinkMax = times * 2; 
    blinkCount = 0; 
    blinkDuration = duration;
    ledState = HIGH; 
    digitalWrite(PIN_LED, ledState); 
    ledTimer = millis(); 
    blinkCount++;
}

// Obsługa nieblokującego migania diody LED w głównej pętli
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
// Funkcja pobierająca aktualny czas z internetu (Zegar Atomowy NTP)
String getTimeString(bool forFileName = false) {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo, 2000)) { // Czeka max 2 sekundy
        return forFileName ? "Brak_Czasu" : "Brak synchronizacji z siecia";
    }
    char buffer[30];
    if (forFileName) {
        strftime(buffer, sizeof(buffer), "%d_%m_%Y_%H_%M", &timeinfo);
    } else {
        strftime(buffer, sizeof(buffer), "%d.%m.%Y %H:%M:%S", &timeinfo);
    }
    return String(buffer);
}
// 1. Inicjalizacja sprzetowa karty (wywoływana na początku setup)
void initSD() {
    SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS); 
    if (SD.begin(PIN_SD_CS)) { 
        statusSD = true; 
        Serial.println("[SYSTEM] Karta SD zainicjalizowana poprawnie.");
    } else { 
        statusSD = false; 
        myNex.writeStr("sd.txt", "ERR"); 
        Serial.println("[BLAD] Nie odnaleziono karty SD!");
    }
}

// 2. Inteligentny raport startowy (wywoływany na samym koncu setup)
void logBootEvent() {
    // Jeśli karta nie wstała sprzetowo, nie próbujemy nawet otwierać pliku
    if (!statusSD) return;
    
    Serial.println("[NTP] Czekam na synchronizację czasu dla logów...");
    // Czekamy max 5 sekund na czas z sieci
    for(int i = 0; i < 50; i++) {
        if (getTimeString() != "Brak synchronizacji z siecia") break;
        delay(100);
    }
    
    // Wymuszone odczyty kontrolne czujników
    float boot_pzem_v = pzem.voltage();
    bool boot_pzem_ok = !isnan(boot_pzem_v);
    
    float boot_temp = dht.readTemperature();
    float boot_hum = dht.readHumidity();
    bool boot_dht_ok = !isnan(boot_temp);
    
    bool boot_iso_ok = (statusDAC || statusADS);

    File f = SD.open("/AI_LOG.txt", FILE_APPEND); 
    if (f) { 
        f.println("\n========================================");
        f.println("BOOT MASZYNY: " + getTimeString());
        f.println("System: Granulator Pro V16.5");
        f.println("Adres IP LAN: " + (WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "Brak-Tryb(AP)"));
        f.println("--- STATUS SPRZETU ---");
        f.println("Zasilanie (PZEM-004T): " + String(boot_pzem_ok ? "ONLINE (" + String(boot_pzem_v, 1) + " V)" : "OFFLINE / BLAD"));
        f.println("Ekran HMI (Nextion): ONLINE (Port UART aktywny)");
        f.println("Zadajnik (ADS1115): " + String(statusADS ? "ONLINE" : "OFFLINE / BLAD"));
        f.println("Falowniki (GP8403): " + String(statusDAC ? "ONLINE" : "OFFLINE / BLAD"));
        f.println("Izolator I2C (ISO1540): " + String(boot_iso_ok ? "ONLINE" : "OFFLINE / BLAD"));
        f.println("Klimat (DHT11): " + String(boot_dht_ok ? "ONLINE (" + String(boot_temp, 1) + " st.C / " + String(boot_hum, 0) + " %)" : "OFFLINE / BLAD"));
        f.println("Logi (Karta SD): ONLINE");
        f.println("========================================");
        f.close(); 
        Serial.println("[SYSTEM] Raport startowy zapisany w AI_LOG.txt");
    } else {
        Serial.println("[BLAD] Nie udalo sie otworzyc pliku AI_LOG.txt do zapisu!");
    }
}

// Aplikuje limity dolne i górne dla układu DAC oraz algorytmu PID
void applyOutputMode() {
    myPID.SetOutputLimits(minDacVolt, maxDacVolt); 
    memory.putInt("outMode", outMode);
}

// Przełącza maszynę z trybu Router (Client) na tryb Sieci Lokalnej (Access Point)
void toggleLocalWiFi() {
    int apState = memory.getInt("apState", 1); 
    apState = (apState == 1) ? 0 : 1; 
    
    // Zabezpieczenie: Nie można wyłączyć AP, jeśli nie wpisano danych do domowego routera
    if (apState == 0 && routerSSID == "") {
        apState = 1;
    }
    
    memory.putInt("apState", apState);
    
    if (apState == 1) {
        WiFi.mode(routerSSID != "" ? WIFI_AP_STA : WIFI_AP);
        IPAddress local_ip(192, 168, 5, 1); 
        IPAddress gateway(192, 168, 5, 1); 
        IPAddress subnet(255, 255, 255, 0);
        WiFi.softAPConfig(local_ip, gateway, subnet); 
        WiFi.softAP("RegulatorPID");
    } else { 
        WiFi.softAPdisconnect(true); 
        WiFi.mode(WIFI_STA); 
    }
}

// Tworzy JSON z listą plików na karcie SD i wysyła przez MQTT
void publishSDList() {
    if (SD.cardType() == CARD_NONE) {
        return;
    }
    
    File root = SD.open("/"); 
    String json = "["; 
    File file = root.openNextFile(); 
    bool first = true;
    
    while(file) {
        if (!file.isDirectory()) {
            if (!first) {
                json += ",";
            }
            json += "{\"name\":\"" + String(file.name()) + "\",\"size\":" + String(file.size() / 1024) + "}";
            first = false;
        }
        file = root.openNextFile();
    }
    json += "]";
    
    String topic = "biuro/" + mqtt_id + "/sdlist";
    mqtt.beginPublish(topic.c_str(), json.length(), false); 
    mqtt.print(json); 
    mqtt.endPublish();
}

// Zmienia napis na ekranie Nextion informujący o trybie danych do chmury
void updateNextionEcoText() {
    if (cloudEcoMode) {
        myNex.writeStr("page4.mqtttext.txt", "Eco");
    } else {
        myNex.writeStr("page4.mqtttext.txt", "Max");
    }
}

// =====================================================================================
// INTELIGENTNY SILNIK OTA (ZDALNA AKTUALIZACJA Z GITHUBA)
// =====================================================================================
void performRemoteOTA(String url) {
    Serial.println("[OTA] Otrzymano rozkaz aktualizacji z chmury!");
    Serial.println("[OTA] URL: " + url);
    stopRegulator(); // Ze względów bezpieczeństwa natychmiast zatrzymujemy maszynę
    
    remoteOtaProgress = 0;
    remoteOtaState = "Nawiązywanie połączenia i szukanie pliku...";
    
    // Konfiguracja bezpiecznego klienta omijającego weryfikację certyfikatu SSL (niezbędne dla GitHuba)
    WiFiClientSecure otaClient;
    otaClient.setInsecure(); 
    
    HTTPClient http; 
    http.begin(otaClient, url); 
    
    // Kluczowa linijka: Rozwiązuje problem "HTTP 302 Redirect" narzucany przez serwery GitHub
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    
    int httpCode = http.GET();
    
    // Kody 200 (OK) lub 206 (Partial Content) oznaczają, że plik fizycznie tam jest
    if (httpCode == 200 || httpCode == 206) {
        int contentLength = http.getSize(); 
        Serial.println("[OTA] Rozmiar pliku: " + String(contentLength) + " bajtow.");
        remoteOtaState = "Pobieranie pliku (Flashowanie)...";
        
        bool canBegin = Update.begin(contentLength);
        if (canBegin) {
            WiFiClient& client = http.getStream(); 
            size_t written = 0;
            uint8_t buff[512] = { 0 }; // Ustawiamy bufor na małe paczki 512-bajtowe
            
            while (http.connected() && (contentLength > 0 || contentLength == -1)) {
                size_t size = client.available();
                if (size) {
                    // Odczytujemy paczkę z sieci
                    int c = client.readBytes(buff, ((size > sizeof(buff)) ? sizeof(buff) : size));
                    // Wgrywamy paczkę bezpośrednio do pamięci Flash mikrokontrolera
                    Update.write(buff, c);
                    written += c;
                    
                    // Aktualizujemy globalną zmienną postępu
                    if (contentLength > 0) {
                        remoteOtaProgress = (written * 100) / contentLength;
                    }
                    
                    // MAGIA: W międzyczasie serwer obsługuje żądania z przeglądarki (pokazuje pasek!)
                    server.handleClient(); 
                    // Obsługujemy również ekran fizyczny Nextion, by się nie zawiesił
                    handleNextionInput();
                }
                delay(1);
                
                // Przerwanie pętli, gdy całość zostanie pobrana
                if (contentLength > 0 && written >= contentLength) {
                    break;
                }
            }
            
            // Weryfikacja integralności wgranego pliku
            if (Update.end()) { 
                remoteOtaState = "Zakończono sukcesem! Restartowanie...";
                remoteOtaProgress = 100;
                server.handleClient(); // Pchamy ostatnie żądanie do przeglądarki na 100%
                
                Serial.println("[OTA] Sukces. Maszyna zrestartuje sie za 3 sekundy.");
                delay(3000); 
                ESP.restart(); // Restart procesora z nowym firmware!
            } else {
                remoteOtaState = "Błąd zapisu do pamięci Flash!";
                remoteOtaProgress = -1;
                Serial.println("[OTA] Blad Update.end()");
            }
        } else {
            remoteOtaState = "Błąd: Zbyt mała pamięć uC.";
            remoteOtaProgress = -1;
            Serial.println("[OTA] Blad Update.begin()");
        }
    } else {
        remoteOtaState = "Błąd pobierania! (Kod HTTP: " + String(httpCode) + ")";
        remoteOtaProgress = -1;
        Serial.println("[OTA] Blad pobierania! Kod: " + String(httpCode));
    }
    
    http.end();
    
    // Jeśli nastąpił błąd (-1), wyświetlamy komunikat przez 5 sek, a potem gasimy okno na WWW
    if (remoteOtaProgress == -1) {
        delay(5000); 
        remoteOtaState = "";
    }
}

// =====================================================================================
// OBSŁUGA PROTOKOŁU MQTT Z CHMURY (Odbieranie rozkazów)
// =====================================================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String msg = ""; 
    for (int i = 0; i < length; i++) {
        msg += (char)payload[i];
    }
    Serial.println("[MQTT] Otrzymano rozkaz: " + msg);

    // Reakcje na polecenia dyspozytorskie
    if (msg == "SYSTEM=ON") { 
        startRegulator(); 
    }
    else if (msg == "SYSTEM=OFF") { 
        stopRegulator(); 
    }
    else if (msg == "MODE=AUTO") { 
        modeAUTO = true; 
        myNex.writeStr("pracaautoman.txt", "AUT"); 
    }
    else if (msg == "MODE=MAN") { 
        modeAUTO = false; 
        myNex.writeStr("pracaautoman.txt", "MAN"); 
    }
    else if (msg == "ECO=TOGGLE") { 
        cloudEcoMode = !cloudEcoMode; 
        memory.putBool("cloudEco", cloudEcoMode); 
        updateNextionEcoText();
    }
    else if (msg == "CMD:RESTART") { 
        ESP.restart(); 
    }
    else if (msg == "CMD:SDLIST") { 
        publishSDList(); 
    }
    else if (msg.startsWith("OTA=")) { 
        performRemoteOTA(msg.substring(4)); 
    }
    // Reakcje na polecenia nastawień z formularzy
    else if (msg.startsWith("CMD:LIMITS:")) {
        int p1 = msg.indexOf(':', 11); 
        minLimit = msg.substring(11, p1).toFloat(); 
        maxLimit = msg.substring(p1 + 1).toFloat();
        memory.putFloat("minLim", minLimit); 
        memory.putFloat("maxLim", maxLimit); 
        updateSettingsScreen();
    }
    else if (msg.startsWith("CMD:RATIOS:")) {
        int p1 = msg.indexOf(':', 11); 
        dac1Ratio = msg.substring(11, p1).toFloat(); 
        dac2Ratio = msg.substring(p1 + 1).toFloat();
        memory.putFloat("dac1Ratio", dac1Ratio); 
        memory.putFloat("dac2Ratio", dac2Ratio);
    }
    else if (msg.startsWith("CMD:PID:")) {
        int p1 = msg.indexOf(':', 8); 
        int p2 = msg.indexOf(':', p1 + 1);
        int p3 = msg.indexOf(':', p2 + 1);
        int p4 = msg.indexOf(':', p3 + 1);
        
        Kp = msg.substring(8, p1).toFloat(); 
        Ki = msg.substring(p1 + 1, p2).toFloat(); 
        Kd = msg.substring(p2 + 1, p3).toFloat();
        
        if (p3 != -1 && p4 != -1) {
            delayTimeSek = msg.substring(p3 + 1, p4).toFloat();
            deadbandAmps = msg.substring(p4 + 1).toFloat();
            
            // Zabezpieczenie przed wpisaniem wartości ujemnych z palca
            if(delayTimeSek < 0.0) delayTimeSek = 0.0;
            if(deadbandAmps < 0.0) deadbandAmps = 0.0;

            memory.putFloat("delayTime", delayTimeSek);
            memory.putFloat("deadBand", deadbandAmps);
            
            // Trik "pod maską" chroniący układ:
            int bezpiecznyCzas = (delayTimeSek <= 0.0) ? 100 : (int)(delayTimeSek * 1000);
            myPID.SetSampleTime(bezpiecznyCzas);
        }

        myPID.SetTunings(Kp, Ki, Kd); 
        memory.putFloat("kp", Kp); 
        memory.putFloat("ki", Ki); 
        memory.putFloat("kd", Kd);
    }
    else if (msg.startsWith("CMD:ALARMS:")) {
        int p1 = msg.indexOf(':', 11); 
        overloadLimit = msg.substring(11, p1).toFloat(); 
        recoveryLimit = msg.substring(p1 + 1).toFloat();
        memory.putFloat("ovrLimit", overloadLimit); 
        memory.putFloat("recLimit", recoveryLimit);
    }
    else if (msg.startsWith("CMD:VOLT:")) {
        int p1 = msg.indexOf(':', 9); 
        minDacVolt = msg.substring(9, p1).toFloat(); 
        maxDacVolt = msg.substring(p1 + 1).toFloat();
        memory.putFloat("minDacVolt", minDacVolt); 
        memory.putFloat("maxDacVolt", maxDacVolt); 
        applyOutputMode();
    }
    else if (msg.startsWith("CMD:OUTMODE:")) { 
        outMode = msg.substring(12).toInt(); 
        applyOutputMode(); 
    }
    else if (msg.startsWith("CMD:CALIB:")) {
        int p1 = msg.indexOf(':', 10); 
        dac1Calib = msg.substring(10, p1).toFloat(); 
        dac2Calib = msg.substring(p1 + 1).toFloat();
        memory.putFloat("dac1Calib", dac1Calib); 
        memory.putFloat("dac2Calib", dac2Calib);
    }
    else if (msg == "CMD:SAVEDEF") {
        memory.putFloat("d_minL", minLimit); 
        memory.putFloat("d_maxL", maxLimit); 
        memory.putFloat("d_kp", Kp);
        memory.putFloat("d_ki", Ki); 
        memory.putFloat("d_kd", Kd); 
        memory.putFloat("d_ovL", overloadLimit);
        memory.putFloat("d_recL", recoveryLimit); 
        memory.putFloat("d_d1R", dac1Ratio); 
        memory.putFloat("d_d2R", dac2Ratio);
        memory.putFloat("d_minV", minDacVolt); 
        memory.putFloat("d_maxV", maxDacVolt); 
        memory.putInt("d_outM", outMode);
    }
    else if (msg == "CMD:RESTOREDEF") { 
        handleRestoreDefaults(); 
    }
    else if (msg.startsWith("CMD:WIFI:")) {
        int p1 = msg.indexOf(':', 9); 
        routerSSID = msg.substring(9, p1); 
        memory.putString("ssid", routerSSID);
        String p = msg.substring(p1 + 1); 
        if (p != "") { 
            routerPASS = p; 
            memory.putString("pass", routerPASS); 
        } 
        ESP.restart();
    }
    else if (msg.startsWith("CMD:MQTT:")) {
        int p1 = msg.indexOf(':', 9); 
        int p2 = msg.indexOf(':', p1 + 1); 
        int p3 = msg.indexOf(':', p2 + 1);
        mqtt_server = cleanHostAddress(msg.substring(9, p1)); 
        memory.putString("mq_srv", mqtt_server);
        mqtt_user = msg.substring(p1 + 1, p2); 
        memory.putString("mq_usr", mqtt_user);
        String pas = msg.substring(p2 + 1, p3); 
        if (pas != "") { 
            mqtt_pass = pas; 
            memory.putString("mq_pas", mqtt_pass); 
        }
        mqtt_id = msg.substring(p3 + 1); 
        memory.putString("mq_id", mqtt_id); 
        ESP.restart();
    }
    
    // Potwierdzenie odebrania komunikatu mignięciem LED
    triggerBlink(2, 100); 
}

// Funkcja obsługująca utrzymanie połączenia z serwerem HiveMQ
void handleMQTT() {
    // Przerywamy, jeśli brak konfiguracji lub brak połączenia z siecią
    if (mqtt_server == "" || routerSSID == "") return;
    if (WiFi.status() != WL_CONNECTED || WiFi.localIP().toString() == "0.0.0.0") return;

    if (!mqtt.connected()) {
        // Próba wznowienia połączenia co 15 sekund
        if (millis() - lastMqttReconnect > 15000) {
            lastMqttReconnect = millis();
            espClient.stop(); 
            espClient.setInsecure();
            
            String clientId = mqtt_id + "-" + String(random(0xffff), HEX);
            Serial.print("[MQTT] Łączenie z brokerem...");
            
            if (mqtt.connect(clientId.c_str(), mqtt_user.c_str(), mqtt_pass.c_str())) {
                Serial.println(" POŁĄCZONO!");
                String subTopic = "biuro/" + mqtt_id + "/rozkazy";
                mqtt.subscribe(subTopic.c_str());
            } else {
                Serial.print(" BŁĄD! Kod=");
                Serial.println(mqtt.state());
            }
        }
    } else {
        // Podtrzymanie nasłuchu
        mqtt.loop();
        
        // Zależnie od Trybu ECO, ładunek wysyłamy co 8 sekund (cienko) lub co 3 sekundy (grubo)
        unsigned long interwalWysylki = cloudEcoMode ? 8000 : 3000;

        if (millis() - lastMqttPublish > interwalWysylki) {
            lastMqttPublish = millis();
            String json; 
            json.reserve(1200); // Rezerwacja pamięci w celu uniknięcia fragmentacji RAM
            
            if (cloudEcoMode) {
                // ==========================
                // TRYB ECO: Pakiet Thin JSON
                // Oszczędza aż 95% danych transferu
                // ==========================
                json = "{";
                json += "\"eco\":1,";
                json += "\"amp\":" + String(current_Amps, 2) + ",";
                json += "\"sysON\":" + String(systemON ? 1 : 0) + ",";
                json += "\"trip\":" + String(trippedByOverload ? 1 : 0) + ",";
                json += "\"autoM\":" + String(modeAUTO ? 1 : 0);
                json += "}";
            } else {
                // ==========================
                // TRYB MAX: Pakiet Thick JSON
                // Pełen raport diagnostyczny systemu
                // ==========================
                float safe_temp = isnan(dht_t) ? 0.0 : dht_t; 
                float safe_hum = isnan(dht_h) ? 0.0 : dht_h; 
                float safe_pf = isnan(pzem_pf) ? 0.0 : pzem_pf;
                
                json = "{";
                json += "\"eco\":0,";
                json += "\"amp\":" + String(current_Amps, 2) + ",\"setp\":" + String(Setpoint, 2) + ",";
                json += "\"sysON\":" + String(systemON ? 1 : 0) + ",\"autoM\":" + String(modeAUTO ? 1 : 0) + ",";
                json += "\"trip\":" + String(trippedByOverload ? 1 : 0) + ",\"volt\":" + String(pzem_u, 1) + ",";
                json += "\"pow\":" + String(pzem_p, 0) + ",\"ap_pow\":" + String(pzem_s, 0) + ",\"re_pow\":" + String(pzem_q, 0) + ",";
                json += "\"pf\":" + String(safe_pf, 2) + ",\"temp\":" + String(safe_temp, 1) + ",\"hum\":" + String(safe_hum, 0) + ",";
                json += "\"dac\":" + String(currentDac1, 2) + ",\"dac2v\":" + String(currentDac2, 2) + ",";
                json += "\"minL\":" + String(minLimit, 1) + ",\"maxL\":" + String(maxLimit, 1) + ",\"kp\":" + String(Kp, 3) + ",\"ki\":" + String(Ki, 3) + ",\"kd\":" + String(Kd, 3) + ",";
                json += "\"dac1R\":" + String(dac1Ratio, 0) + ",\"dac2R\":" + String(dac2Ratio, 0) + ",\"ovL\":" + String(overloadLimit, 1) + ",\"recL\":" + String(recoveryLimit, 1) + ",";
                json += "\"dac1C\":" + String(dac1Calib, 2) + ",\"dac2C\":" + String(dac2Calib, 2) + ",\"minV\":" + String(minDacVolt, 2) + ",\"maxV\":" + String(maxDacVolt, 2) + ",\"outM\":" + String(outMode) + ",";
                json += "\"pzem\":" + String(statusPZEM ? 1 : 0) + ",\"nex\":" + String((millis() - lastNextionResponseTime < 5000) ? 1 : 0) + ",\"ads\":" + String(statusADS ? 1 : 0) + ",\"dac_st\":" + String(statusDAC ? 1 : 0) + ",";
                json += "\"dht_st\":" + String(!isnan(dht_t) ? 1 : 0) + ",\"sd\":" + String(statusSD ? 1 : 0) + ",\"iso\":" + String((statusDAC || statusADS) ? 1 : 0) + ",";
                json += "\"up_s\":" + String(millis() / 1000) + ",\"heap_pct\":" + String(((float)ESP.getFreeHeap() / ESP.getHeapSize()) * 100.0, 1) + ",";
                json += "\"cpu\":" + String(ESP.getCpuFreqMHz()) + ",\"chip\":\"" + String(ESP.getChipModel()) + "\",\"sketch_k\":" + String(ESP.getSketchSize() / 1024) + ",\"cli\":" + String(WiFi.softAPgetStationNum()) + ",";
                json += "\"wifi_s\":\"" + routerSSID + "\",\"mq_srv\":\"" + mqtt_server + "\",\"mq_usr\":\"" + mqtt_user + "\",\"mq_id\":\"" + mqtt_id + "\",\"ip\":\"" + WiFi.localIP().toString() + "\"}";
            }
            
            // Wysłanie zbudowanego ładunku do chmury
            String pubTopic = "biuro/" + mqtt_id + "/dane";
            mqtt.beginPublish(pubTopic.c_str(), json.length(), false); 
            mqtt.print(json); 
            mqtt.endPublish();
        }
    }
}

// =====================================================================================
// BRAMKA LOKALNEGO SERWERA WWW I API
// =====================================================================================
bool checkAuth() { 
    if (!server.authenticate("admin", "regpid12")) { 
        server.requestAuthentication(); 
        return false; 
    } 
    return true; 
}

// Obsługa nakładki graficznej OTA na stronie WWW
void handleOtaStatus() {
    if (!checkAuth()) return;
    
    String json = "{";
    json += "\"progress\":" + String(remoteOtaProgress) + ",";
    json += "\"state\":\"" + remoteOtaState + "\"";
    json += "}";
    
    server.send(200, "application/json", json);
}

// Generowanie paczki JSON dla lokalnego wyświetlania WWW (Co sekundę)
void handleApiData() {
    if (!checkAuth()) return;
    
    String json = "{";
    json += "\"eco\":\"" + String(cloudEcoMode ? 1 : 0) + "\",";
    json += "\"amp\":\"" + String(current_Amps, 2) + "\",\"setp\":\"" + String(Setpoint, 2) + "\",\"dac\":\"" + String(currentDac1, 2) + "\",\"dac2v\":\"" + String(currentDac2, 2) + "\",";
    json += "\"trip\":\"" + String(trippedByOverload ? 1 : 0) + "\",\"sysON\":\"" + String(systemON ? 1 : 0) + "\",\"autoM\":\"" + String(modeAUTO ? 1 : 0) + "\",";
    json += "\"volt\":\"" + String(pzem_u, 1) + "\",\"pow\":\"" + String(pzem_p, 0) + "\",\"ap_pow\":\"" + String(pzem_s, 0) + "\",\"re_pow\":\"" + String(pzem_q, 0) + "\",\"pf\":\"" + String(pzem_pf, 2) + "\",";
    json += "\"temp\":\"" + String(dht_t, 1) + "\",\"hum\":\"" + String(dht_h, 0) + "\",\"minL\":\"" + String(minLimit, 1) + "\",\"maxL\":\"" + String(maxLimit, 1) + "\",";
    json += "\"kp\":\"" + String(Kp, 3) + "\",\"ki\":\"" + String(Ki, 3) + "\",\"kd\":\"" + String(Kd, 3) + "\",\"dt\":\"" + String(delayTimeSek, 1) + "\",\"db\":\"" + String(deadbandAmps, 1) + "\",\"outM\":\"" + String(outMode) + "\",";
    json += "\"dac1C\":\"" + String(dac1Calib, 2) + "\",\"dac2C\":\"" + String(dac2Calib, 2) + "\",\"minV\":\"" + String(minDacVolt, 2) + "\",\"maxV\":\"" + String(maxDacVolt, 2) + "\",";
    json += "\"wifi_s\":\"" + routerSSID + "\",\"mq_srv\":\"" + mqtt_server + "\",\"mq_usr\":\"" + mqtt_user + "\",\"mq_id\":\"" + mqtt_id + "\"}";
    
    server.send(200, "application/json", json);
}

// Generowanie diagnostyki zdrowia podzespołów maszyny
void handleApiHealth() {
    if (!checkAuth()) return;
    
    bool statusNex = (millis() - lastNextionResponseTime < 5000); 
    
    String json = "{";
    json += "\"pzem\":\"" + String(statusPZEM ? 1 : 0) + "\",\"dac\":\"" + String(statusDAC ? 1 : 0) + "\",\"ads\":\"" + String(statusADS ? 1 : 0) + "\",";
    json += "\"dht\":\"" + String(!isnan(dht_t) ? 1 : 0) + "\",\"sd\":\"" + String(statusSD ? 1 : 0) + "\",\"nex\":\"" + String(statusNex ? 1 : 0) + "\",";
    
    unsigned long sec = millis() / 1000; 
    unsigned long d = sec / 86400; 
    unsigned long h = (sec % 86400) / 3600; 
    unsigned long m = (sec % 3600) / 60; 
    unsigned long s = sec % 60;
    
    String upStr = "";
    if (d > 0) upStr = String(d) + "d " + String(h) + "h " + String(m) + "m";
    else if (h > 0) upStr = String(h) + "h " + String(m) + "m " + String(s) + "s";
    else if (m > 0) upStr = String(m) + "m " + String(s) + "s";
    else upStr = String(s) + "s";
    
    json += "\"up\":\"" + upStr + "\",\"heap_pct\":\"" + String(((float)ESP.getFreeHeap() / ESP.getHeapSize()) * 100.0, 1) + "\",\"cpu\":\"" + String(ESP.getCpuFreqMHz()) + " MHz\",";
    json += "\"chip\":\"" + String(ESP.getChipModel()) + " (" + String(ESP.getChipCores()) + " Core)\",\"sketch\":\"" + String(ESP.getSketchSize() / 1024) + " KB\",";
    int logRem = isLoggingActive ? (logEndTime - millis()) / 1000 : 0;
    json += "\"clients\":\"" + String(WiFi.softAPgetStationNum()) + "\",\"router_ip\":\"" + ((WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "Brak (AP)") + "\",";
    json += "\"log_rem\":\"" + String(logRem) + "\"}";
    
    server.send(200, "application/json", json);
}

// Funkcje wywoływane wciśnięciem przycisków na panelu WWW
void handleToggleEco() { 
    if (!checkAuth()) return; 
    cloudEcoMode = !cloudEcoMode; 
    memory.putBool("cloudEco", cloudEcoMode); 
    updateNextionEcoText(); 
    server.send(200, "text/plain", "OK"); 
}

void handleToggleSys() { 
    if (!checkAuth()) return; 
    if (systemON) {
        stopRegulator(); 
    } else {
        startRegulator(); 
    }
    server.send(200, "text/plain", "OK"); 
}

void handleToggleMode() { 
    if (!checkAuth()) return; 
    modeAUTO = !modeAUTO; 
    myNex.writeStr("pracaautoman.txt", modeAUTO ? "AUT" : "MAN"); 
    server.send(200, "text/plain", "OK"); 
}

// Funkcje odbierające dane z formularzy WWW i zapisujące w pamięci urządzenia
void handleSetOutMode() { 
    if (!checkAuth()) return; 
    if (server.hasArg("m")) { 
        outMode = server.arg("m").toInt(); 
        applyOutputMode(); 
        triggerBlink(1, 1000); 
    } 
    server.send(200, "text/plain", "OK"); 
}

void handleSetRatios() { 
    if (!checkAuth()) return; 
    if (server.hasArg("r1") && server.hasArg("r2")) { 
        dac1Ratio = server.arg("r1").toFloat(); 
        dac2Ratio = server.arg("r2").toFloat(); 
        memory.putFloat("dac1Ratio", dac1Ratio); 
        memory.putFloat("dac2Ratio", dac2Ratio); 
        triggerBlink(1, 1000); 
    } 
    server.send(200, "text/plain", "OK"); 
}

void handleSetAlarms() { 
    if (!checkAuth()) return; 
    if (server.hasArg("ov") && server.hasArg("rec")) { 
        overloadLimit = server.arg("ov").toFloat(); 
        recoveryLimit = server.arg("rec").toFloat(); 
        memory.putFloat("ovrLimit", overloadLimit); 
        memory.putFloat("recLimit", recoveryLimit); 
        triggerBlink(1, 1000); 
    } 
    server.send(200, "text/plain", "OK"); 
}

void handleSetLimits() { 
    if (!checkAuth()) return; 
    if (server.hasArg("min") && server.hasArg("max")) { 
        minLimit = server.arg("min").toFloat(); 
        maxLimit = server.arg("max").toFloat(); 
        updateSettingsScreen(); 
        memory.putFloat("minLim", minLimit); 
        memory.putFloat("maxLim", maxLimit); 
        triggerBlink(1, 1000); 
    } 
    server.send(200, "text/plain", "OK"); 
}

void handleSetPID() { 
    if (!checkAuth()) return; 
    if (server.hasArg("kp") && server.hasArg("ki") && server.hasArg("kd")) { 
        Kp = server.arg("kp").toFloat(); 
        Ki = server.arg("ki").toFloat(); 
        Kd = server.arg("kd").toFloat(); 
        memory.putFloat("kp", Kp); 
        memory.putFloat("ki", Ki); 
        memory.putFloat("kd", Kd); 
        myPID.SetTunings(Kp, Ki, Kd); 

        // Odbiór nowych parametrów Czasu Zwłoki i Strefy Nieczułości
        if(server.hasArg("dt")) {
            delayTimeSek = server.arg("dt").toFloat();
            memory.putFloat("delayTime", delayTimeSek);
            myPID.SetSampleTime((int)(delayTimeSek * 1000)); // Wrzucenie czasu w milisekundach do biblioteki
        }
        if(server.hasArg("db")) {
            deadbandAmps = server.arg("db").toFloat();
            memory.putFloat("deadBand", deadbandAmps);
        }

        triggerBlink(1, 1000); 
    } 
    server.send(200, "text/plain", "OK"); 
}

void handleSetCalib() { 
    if (!checkAuth()) return; 
    if (server.hasArg("c1") && server.hasArg("c2")) { 
        dac1Calib = server.arg("c1").toFloat(); 
        dac2Calib = server.arg("c2").toFloat(); 
        memory.putFloat("dac1Calib", dac1Calib); 
        memory.putFloat("dac2Calib", dac2Calib); 
        triggerBlink(1, 1000); 
    } 
    server.send(200, "text/plain", "OK"); 
}

void handleSetVoltLimits() { 
    if (!checkAuth()) return; 
    if (server.hasArg("min") && server.hasArg("max")) { 
        minDacVolt = server.arg("min").toFloat(); 
        maxDacVolt = server.arg("max").toFloat(); 
        memory.putFloat("minDacVolt", minDacVolt); 
        memory.putFloat("maxDacVolt", maxDacVolt); 
        myPID.SetOutputLimits(minDacVolt, maxDacVolt); 
        triggerBlink(1, 1000); 
    } 
    server.send(200, "text/plain", "OK"); 
}

void handleSetWiFi() { 
    if (!checkAuth()) return; 
    if (server.hasArg("s")) { 
        routerSSID = server.arg("s"); 
        memory.putString("ssid", routerSSID); 
    } 
    if (server.hasArg("p") && server.arg("p") != "") { 
        routerPASS = server.arg("p"); 
        memory.putString("pass", routerPASS); 
    } 
    server.send(200, "text/plain", "OK"); 
    delay(500); 
    ESP.restart(); 
}

void handleSetMQTT() { 
    if (!checkAuth()) return; 
    if (server.hasArg("srv")) {
        memory.putString("mq_srv", cleanHostAddress(server.arg("srv"))); 
    }
    if (server.hasArg("usr")) {
        memory.putString("mq_usr", server.arg("usr")); 
    }
    if (server.hasArg("pas") && server.arg("pas") != "") {
        memory.putString("mq_pas", server.arg("pas")); 
    }
    if (server.hasArg("id")) {
        memory.putString("mq_id", server.arg("id")); 
    }
    server.send(200, "text/plain", "OK"); 
    delay(500); 
    ESP.restart(); 
}
void handleStartLog() {
    if (!checkAuth()) return;
    if (server.hasArg("min")) {
        int mins = server.arg("min").toInt();
        if (mins > 0 && SD.cardType() != CARD_NONE) {
            isLoggingActive = true;
            logEndTime = millis() + (mins * 60000UL);
            
            // Generowanie pięknej nazwy z datą i godziną (np. /DIAG_14_05_2026_15_30.csv)
            currentLogFileName = "/DIAG_" + getTimeString(true) + ".csv";
            
            File f = SD.open(currentLogFileName.c_str(), FILE_APPEND);
            if (f) {
                // Polskie nagłówki Excela - separator to średnik (;)
                f.println("\nCzas_ms;SysON;Auto;Awaria;Prad_A;Cel_A;U_V;P_W;S_VA;Q_VAR;CosFi;DAC1_V;DAC2_V;Temp_C;Wilg_%;Kp;Ki;Kd;Min_A;Max_A");
                f.close();
            }
        } else {
            isLoggingActive = false;
        }
    }
    server.send(200, "text/plain", "OK");
}

void handleRestart() { 
    if (!checkAuth()) return; 
    server.send(200, "text/plain", "OK"); 
    delay(500); 
    ESP.restart(); 
}
// =====================================================================================
// KROK 3: ZAPIS W PAMIĘCI I OBSŁUGA ZMIANY Z POZIOMU WWW
// =====================================================================================

void handleSetTypZad() { 
    if (!checkAuth()) return; 
    if (server.hasArg("t")) { 
        typZadajnika = server.arg("t").toInt(); 
        memory.putInt("typZad", typZadajnika); 
        triggerBlink(1, 1000); 
    } 
    server.send(200, "text/plain", "OK"); 
}

void handleSaveDefaults() { 
    if (!checkAuth()) return; 
    memory.putFloat("d_minL", minLimit); 
    memory.putFloat("d_maxL", maxLimit); 
    memory.putFloat("d_kp", Kp); 
    memory.putFloat("d_ki", Ki); 
    memory.putFloat("d_kd", Kd); 
    memory.putFloat("d_ovL", overloadLimit); 
    memory.putFloat("d_recL", recoveryLimit); 
    memory.putFloat("d_d1R", dac1Ratio); 
    memory.putFloat("d_d2R", dac2Ratio); 
    memory.putFloat("d_minV", minDacVolt); 
    memory.putFloat("d_maxV", maxDacVolt); 
    memory.putInt("d_outM", outMode); 
    
    // Zapisujemy domyślny typ zadajnika do pamięci trwałej
    memory.putInt("d_typZ", typZadajnika); 
    
    server.send(200, "text/plain", "OK"); 
}

void handleRestoreDefaults() { 
    if (!checkAuth()) return; 
    minLimit = memory.getFloat("d_minL", 10.0); 
    maxLimit = memory.getFloat("d_maxL", 40.0); 
    Kp = memory.getFloat("d_kp", 0.5); 
    Ki = memory.getFloat("d_ki", 0.1); 
    Kd = memory.getFloat("d_kd", 0.15); 
    overloadLimit = memory.getFloat("d_ovL", 7.0); 
    recoveryLimit = memory.getFloat("d_recL", 2.0); 
    dac1Ratio = memory.getFloat("d_d1R", 100.0); 
    dac2Ratio = memory.getFloat("d_d2R", 90.0); 
    minDacVolt = memory.getFloat("d_minV", 3.5); 
    maxDacVolt = memory.getFloat("d_maxV", 10.0); 
    outMode = memory.getInt("d_outM", 0); 
    
    // Wczytujemy zresetowany typ zadajnika
    typZadajnika = memory.getInt("d_typZ", 0); 
    
    memory.putFloat("minLim", minLimit); 
    memory.putFloat("maxLim", maxLimit); 
    memory.putFloat("kp", Kp); 
    memory.putFloat("ki", Ki); 
    memory.putFloat("kd", Kd); 
    memory.putFloat("ovrLimit", overloadLimit); 
    memory.putFloat("recLimit", recoveryLimit); 
    memory.putFloat("dac1Ratio", dac1Ratio); 
    memory.putFloat("dac2Ratio", dac2Ratio); 
    memory.putFloat("minDacVolt", minDacVolt); 
    memory.putFloat("maxDacVolt", maxDacVolt); 
    memory.putInt("outMode", outMode); 
    
    // Twardy zapis odzyskanego typu zadajnika do działającej pamięci
    memory.putInt("typZad", typZadajnika); 
    
    memory.putString("ssid", ""); 
    memory.putString("pass", ""); 
    memory.putString("mq_srv", ""); 
    memory.putString("mq_usr", ""); 
    memory.putString("mq_pas", ""); 
    memory.putInt("apState", 1); 
    
    myPID.SetTunings(Kp, Ki, Kd); 
    applyOutputMode(); 
    updateSettingsScreen(); 
    server.send(200, "text/plain", "OK"); 
    delay(500); 
    ESP.restart(); 
}

void handleSDList() { 
    if (!checkAuth()) return; 
    if (SD.cardType() == CARD_NONE) { 
        server.send(200, "application/json", "[]"); 
        return; 
    } 
    
    File root = SD.open("/"); 
    String json = "["; 
    File file = root.openNextFile(); 
    bool first = true; 
    
    while(file) { 
        if (!file.isDirectory()) { 
            if (!first) {
                json += ","; 
            }
            json += "{\"name\":\"" + String(file.name()) + "\",\"size\":" + String(file.size() / 1024) + "}"; 
            first = false; 
        } 
        file = root.openNextFile(); 
    } 
    json += "]"; 
    server.send(200, "application/json", json); 
}

void handleSDRead() { 
    if (!checkAuth()) return; 
    if (!server.hasArg("f")) { 
        server.send(400, "text/plain", "Brak pliku"); 
        return; 
    } 
    File file = SD.open("/" + server.arg("f"), FILE_READ); 
    if (!file) { 
        server.send(404, "text/plain", "Nie odnaleziono"); 
        return; 
    } 
    server.streamFile(file, "text/plain"); 
    file.close(); 
}

// Konfiguracja i uruchomienie serwera WWW maszyny
void setupWiFi() {
    int apState = memory.getInt("apState", 1); 
    WiFi.disconnect(true); 
    WiFi.softAPdisconnect(true); 
    delay(100);
    
    if (routerSSID != "") { 
        WiFi.mode(apState == 1 ? WIFI_AP_STA : WIFI_STA); 
        WiFi.begin(routerSSID.c_str(), routerPASS.c_str()); 
    } else { 
        WiFi.mode(WIFI_AP); 
        apState = 1; 
        memory.putInt("apState", 1); 
    }
    
    if (apState == 1) { 
        IPAddress local_ip(192, 168, 5, 1); 
        IPAddress gateway(192, 168, 5, 1); 
        IPAddress subnet(255, 255, 255, 0); 
        WiFi.softAPConfig(local_ip, gateway, subnet); 
        
        String apName = "Regulator_" + mqtt_id;
        WiFi.softAP(apName.c_str()); 
    }
    
    String mdnsName = mqtt_id;
    mdnsName.toLowerCase(); 
    MDNS.begin(mdnsName.c_str());
    
    server.on("/", HTTP_GET, []() { 
        if (checkAuth()) {
            server.send(200, "text/html", INDEX_HTML); 
        }
    });
    
    server.on("/api/data", HTTP_GET, handleApiData); 
    server.on("/api/health", HTTP_GET, handleApiHealth);
    server.on("/api/ota_status", HTTP_GET, handleOtaStatus);
    server.on("/api/toggle_sys", HTTP_POST, handleToggleSys); 
    server.on("/api/toggle_mode", HTTP_POST, handleToggleMode); 
    server.on("/api/toggle_eco", HTTP_POST, handleToggleEco);
    server.on("/api/set_limits", HTTP_POST, handleSetLimits); 
    server.on("/api/set_outmode", HTTP_POST, handleSetOutMode); 
    server.on("/api/set_ratios", HTTP_POST, handleSetRatios);
    server.on("/api/set_alarms", HTTP_POST, handleSetAlarms); 
    server.on("/api/set_pid", HTTP_POST, handleSetPID); 
    server.on("/api/set_calib", HTTP_POST, handleSetCalib);
    server.on("/api/set_volt_limits", HTTP_POST, handleSetVoltLimits); 
    server.on("/api/set_wifi", HTTP_POST, handleSetWiFi); 
    server.on("/api/set_mqtt", HTTP_POST, handleSetMQTT);
    server.on("/api/restart", HTTP_POST, handleRestart);
    server.on("/api/start_log", HTTP_POST, handleStartLog);
    server.on("/api/save_defaults", HTTP_POST, handleSaveDefaults); 
    server.on("/api/restore_defaults", HTTP_POST, handleRestoreDefaults);
    
    // Nowy endpoint do obsługi wyboru Zadajnika z poziomu HTML
    server.on("/api/set_typzad", HTTP_POST, handleSetTypZad);
    
    server.on("/api/sd_list", HTTP_GET, handleSDList); 
    server.on("/sd_read", HTTP_GET, handleSDRead);
    
    server.on("/update", HTTP_POST, []() { 
        if (checkAuth()) { 
            server.sendHeader("Connection", "close"); 
            server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK"); 
            ESP.restart(); 
        } 
    }, []() { 
        if (!checkAuth()) return; 
        HTTPUpload& upload = server.upload(); 
        if (upload.status == UPLOAD_FILE_START) { 
            if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
                Update.printError(Serial); 
            }
        } else if (upload.status == UPLOAD_FILE_WRITE) { 
            if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
                Update.printError(Serial); 
            }
        } else if (upload.status == UPLOAD_FILE_END) { 
            if (!Update.end(true)) {
                Update.printError(Serial); 
            }
        } 
    });
  
    server.begin(); 
    isWifiAPActive = true; 
    triggerBlink(3, 100); 
}

void onStationConnected(WiFiEvent_t event, WiFiEventInfo_t info) { 
    triggerBlink(2, 200); 
}

// =====================================================================================
// LOGIKA STEROWANIA SYSTEMEM PID I FALOWNIKAMI
// =====================================================================================
void startRegulator() {
    systemON = true; 
    trippedByOverload = false; 
    myNex.writeStr("pidonoff.txt", "ON"); 
    
    // Przechodzimy na chwilę w tryb manualny by bezpiecznie wystartować maszynę
    myPID.SetMode(MANUAL);          
    
    if (napiecieZadajnika < minDacVolt) {
        Output = minDacVolt; 
    } else if (napiecieZadajnika > maxDacVolt) {
        Output = maxDacVolt; 
    } else {
        Output = napiecieZadajnika;
    }
    
    currentDac1 = Output * (dac1Ratio / 100.0); 
    currentDac2 = Output * (dac2Ratio / 100.0);
    
    if (currentDac1 < minDacVolt) currentDac1 = minDacVolt; 
    if (currentDac2 < minDacVolt) currentDac2 = minDacVolt;
    
    float finalDac1 = currentDac1 + dac1Calib; 
    float finalDac2 = currentDac2 + dac2Calib;
    
    if (finalDac1 < 0.0) finalDac1 = 0.0; 
    if (finalDac1 > maxDacVolt) finalDac1 = maxDacVolt;
    if (finalDac2 < 0.0) finalDac2 = 0.0; 
    if (finalDac2 > maxDacVolt) finalDac2 = maxDacVolt;
    
    uint16_t mv_dac1 = (uint16_t)(finalDac1 * 1000.0); 
    uint16_t mv_dac2 = (uint16_t)(finalDac2 * 1000.0);
    
    dac.setDACOutVoltage(mv_dac1, 0); 
    dac.setDACOutVoltage(mv_dac2, 1);
    delay(50); 
    
    digitalWrite(PIN_RELAY_1, RELAY_ON); 
    digitalWrite(PIN_RELAY_2, RELAY_ON); 
    
    // Wracamy do obliczeń zamkniętej pętli sprzężenia zwrotnego
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
    if (id == 1) { 
        minLimit += 0.1; 
        if (minLimit > maxLimit) minLimit = maxLimit; 
    }
    if (id == 2) { 
        minLimit -= 0.1; 
        if (minLimit < 0) minLimit = 0; 
    }
    if (id == 3) { 
        maxLimit += 0.1; 
        if (maxLimit > 100) maxLimit = 100; 
    }
    if (id == 4) { 
        maxLimit -= 0.1; 
        if (maxLimit < minLimit) maxLimit = minLimit; 
    }
    updateSettingsScreen(); 
    memory.putFloat("minLim", minLimit); 
    memory.putFloat("maxLim", maxLimit);
}

// Zmienne obsługujące płynne podtrzymanie przycisku nastaw na Nextionie
int activeButtonID = 0; 
unsigned long buttonHoldTimer = 0; 
bool isButtonHeld = false;       

// Główne sprzężenie z wyświetlaczem fizycznym (Odbieranie poleceń połączone z parserem biblioteki)
void handleNextionInput() {
    while (NextionSerial.available()) { 
        byte b = NextionSerial.read(); 
        lastNextionResponseTime = millis(); 
        
        // Magiczny bajt poczatkowy protokołu Nextion (0x65 to kod zdarzenia wciśnięcia)
        if (b == 0x65) {                
            delay(15);                   
            if (NextionSerial.available() >= 6) {
                byte pageId = NextionSerial.read(); 
                byte cmpId  = NextionSerial.read(); 
                byte event  = NextionSerial.read(); 
                // Odczyt i zignorowanie końcówki ramki
                NextionSerial.read(); 
                NextionSerial.read(); 
                NextionSerial.read(); 
                
                // Wywołania dla Strony 0 (Panel Główny)
                if (pageId == 0) {
                    if (cmpId == 11 && event == 0x01) { 
                        if (systemON) stopRegulator(); else startRegulator(); 
                    }
                    if (cmpId == 12 && event == 0x01) { 
                        modeAUTO = !modeAUTO; 
                        myNex.writeStr("pracaautoman.txt", modeAUTO ? "AUT" : "MAN"); 
                    }
                    if (cmpId == 8) { 
                        // Zliczanie czasu wciśnięcia dla Resetu Zabezpieczeń
                        if (event == 0x01) { 
                            isResetPressed = true; 
                            resetPressTime = millis(); 
                            resetStage1 = false; 
                            resetStage2 = false; 
                            resetStage3 = false; 
                        }
                        else if (event == 0x00) { 
                            isResetPressed = false; 
                            resetStage1 = false; 
                            resetStage2 = false; 
                            resetStage3 = false; 
                            myNex.writeNum("pod2.bco", 65535); 
                            myNex.writeNum("pod1.bco", 65535); 
                            myNex.writeNum("granampery.bco", 65535); 
                        }
                    }
                }
                
                // Wywołania dla Strony 2 (Konfiguracja Limtów)
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
                
                // Wywołania dla Strony 4 (Menu Inżynierskie)
                if (pageId == 4) {
                    if (cmpId == 7 && event == 0x01) { 
                        toggleLocalWiFi(); 
                    }
                    if (cmpId == 8) {
                        if (event == 0x01) { 
                            isFactoryResetPressed = true; 
                            factoryResetPressTime = millis(); 
                        } 
                        else if (event == 0x00) { 
                            isFactoryResetPressed = false; 
                            myNex.writeNum("page4.bco", 65535); 
                        }
                    }
                    // Obsługa wciśnięcia przycisku Trybu ECO Chmury
                    if (cmpId == 9 && event == 0x01) {
                        cloudEcoMode = !cloudEcoMode; 
                        memory.putBool("cloudEco", cloudEcoMode);
                        updateNextionEcoText();
                    }
                }
            }
        }
    }
    
    // Automatyczne, ciągłe inkrementowanie wartości wciśniętego klawisza (Hold Action)
    if (isButtonHeld && activeButtonID > 0 && millis() > buttonHoldTimer) { 
        processButtonAction(activeButtonID); 
        buttonHoldTimer = millis() + 100; 
    }
}

// =====================================================================================
// SEKCJA SETUP (Uruchamiana raz po podłączeniu zasilania)
// =====================================================================================
void setup() {
    pinMode(PIN_RELAY_1, OUTPUT); 
    pinMode(PIN_RELAY_2, OUTPUT); 
    digitalWrite(PIN_RELAY_1, RELAY_OFF); 
    digitalWrite(PIN_RELAY_2, RELAY_OFF);
    
    pinMode(BUTTON_PIN, INPUT_PULLUP); 
    pinMode(PIN_POT_SYMULACJA, INPUT); 
    pinMode(PIN_LED, OUTPUT); 
    digitalWrite(PIN_LED, LOW);
    
    delay(2000); 
    Serial.begin(115200); 
    Serial.println("\n\n--- SYSTEM V16.5 (PRO OTA & LOGO FRONTEND) ---");

    // Montowanie pamięci masowej układu (Wczytywanie zapisanych ustawień do zmiennych RAM)
    memory.begin("regulator", false); 
    routerSSID = memory.getString("ssid", ""); 
    routerPASS = memory.getString("pass", "");
    mqtt_server = cleanHostAddress(memory.getString("mq_srv", "")); 
    mqtt_user = memory.getString("mq_usr", ""); 
    mqtt_pass = memory.getString("mq_pas", ""); 
    mqtt_id = memory.getString("mq_id", "Granulator_01");

    cloudEcoMode = memory.getBool("cloudEco", false);

    minLimit = memory.getFloat("minLim", 10.0); 
    maxLimit = memory.getFloat("maxLim", 40.0); 
    Kp = memory.getFloat("kp", 0.5); 
    Ki = memory.getFloat("ki", 0.1); 
    Kd = memory.getFloat("kd", 0.15); 
    myPID.SetTunings(Kp, Ki, Kd);
    // Wczytanie Zwłoki i Strefy Nieczułości
    delayTimeSek = memory.getFloat("delayTime", 2.0);
    deadbandAmps = memory.getFloat("deadBand", 1.0);
    
    outMode = memory.getInt("outMode", 0);
    
    // --- INTELIGENTNY MODUŁ: Wczytanie typu zadajnika na obiekcie ---
    typZadajnika = memory.getInt("typZad", 0); // 0=Volty(A0), 1=Ampery(A1)
    // -----------------------------------------------------------------
    
    minDacVolt = memory.getFloat("minDacVolt", 3.5); 
    if (isnan(minDacVolt) || minDacVolt < 0.0) minDacVolt = 0.0; 
    if (minDacVolt > 10.0) minDacVolt = 10.0;
    
    maxDacVolt = memory.getFloat("maxDacVolt", 10.0); 
    if (isnan(maxDacVolt) || maxDacVolt < 0.0) maxDacVolt = 10.0; 
    if (maxDacVolt > 10.0) maxDacVolt = 10.0; 
    if (maxDacVolt < minDacVolt) maxDacVolt = minDacVolt;
    
    applyOutputMode(); 
    
    dac1Ratio = memory.getFloat("dac1Ratio", 100.0); 
    dac2Ratio = memory.getFloat("dac2Ratio", 90.0); 
    if (isnan(dac1Ratio) || dac1Ratio < 0.0) dac1Ratio = 0.0; 
    if (dac1Ratio > 100.0) dac1Ratio = 100.0; 
    if (isnan(dac2Ratio) || dac2Ratio < 0.0) dac2Ratio = 0.0; 
    if (dac2Ratio > 100.0) dac2Ratio = 100.0;
    
    overloadLimit = memory.getFloat("ovrLimit", 7.0); 
    recoveryLimit = memory.getFloat("recLimit", 2.0); 
    if (isnan(overloadLimit) || overloadLimit < 0.0) overloadLimit = 7.0; 
    if (isnan(recoveryLimit) || recoveryLimit < 0.0) recoveryLimit = 2.0;
    
    dac1Calib = memory.getFloat("dac1Calib", 0.0); 
    dac2Calib = memory.getFloat("dac2Calib", 0.0); 
    if (isnan(dac1Calib)) dac1Calib = 0.0; 
    if (isnan(dac2Calib)) dac2Calib = 0.0;

    espClient.setInsecure(); 
    mqtt.setServer(mqtt_server.c_str(), 8883); 
    mqtt.setCallback(mqttCallback);
    // Kluczowe rozszerzenie przestrzeni roboczej bufora ładunków JSON
    mqtt.setBufferSize(4096); 

    WiFi.onEvent(onStationConnected, ARDUINO_EVENT_WIFI_AP_STACONNECTED); 
    setupWiFi();

    // Synchronizacja czasu polskiego z serwerami NTP
    configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", "pool.ntp.org", "time.nist.gov");
    delay(500); // Daj procesorowi pół sekundy na chwycenie zasięgu z satelity

    NextionSerial.begin(9600, SERIAL_8N1, PIN_NEXT_RX, PIN_NEXT_TX);
    
    PzemSerial.begin(9600, SERIAL_8N1, PIN_PZEM_RX, PIN_PZEM_TX); 
    dht.begin();
    
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL); 
    
    // Pingowanie sprzętu na magistrali I2C (Zabezpieczenie przed zwarciem pinów)
    Wire.beginTransmission(0x48); 
    statusADS = (Wire.endTransmission() == 0); 
    if (statusADS) { 
        ads.setGain(GAIN_TWOTHIRDS); 
        ads.begin(0x48); 
    }
    
    Wire.beginTransmission(0x58); 
    statusDAC = (Wire.endTransmission() == 0); 
    if (statusDAC) { 
        dac.begin(); 
        dac.setDACOutRange(dac.eOutputRange10V); 
        dac.setDACOutVoltage(0, 0); 
        dac.setDACOutVoltage(0, 1); 
    }
    
    initSD(); 
    updateSettingsScreen(); 
    myNex.writeStr("pidonoff.txt", "OFF"); 
    myNex.writeStr("pracaautoman.txt", "AUT"); 
    
    // Inicjalizacja ekranu Nextion wymuszająca zgodność tekstów
    updateNextionEcoText(); 
    stopRegulator(); 
    
    myPID.SetMode(AUTOMATIC); 
    
    // Ochrona startowa PID z pamięci Flash
    int bezpiecznyCzas = (delayTimeSek <= 0.0) ? 100 : (int)(delayTimeSek * 1000);
    myPID.SetSampleTime(bezpiecznyCzas);
    
    // Inteligentny raport startowy (Czeka na sieć i czujniki)
    logBootEvent();
    
    triggerBlink(2, 500); 
}

// =====================================================================================
// GŁÓWNA PĘTLA PROGRAMU (Wykonywana dziesiątki tysięcy razy na sekundę)
// =====================================================================================
void loop() {
    handleLED(); 
    handleMQTT();
    
    if (isWifiAPActive) {
        server.handleClient();
    }
    
    handleNextionInput(); 

    if (isResetPressed) {
        unsigned long holdTime = millis() - resetPressTime;
        if (holdTime > 1000 && !resetStage1) { 
            myNex.writeNum("pod2.bco", 0); 
            myNex.writeStr("pod2.txt", ""); 
            resetStage1 = true; 
        }
        if (holdTime > 2000 && !resetStage2) { 
            myNex.writeNum("pod1.bco", 0); 
            myNex.writeStr("pod1.txt", ""); 
            resetStage2 = true; 
        }
        if (holdTime > 3000 && !resetStage3) { 
            myNex.writeNum("granampery.bco", 0); 
            myNex.writeStr("granampery.txt", ""); 
            resetStage3 = true; 
        }
        if (holdTime > 4000) { 
            myNex.writeStr("rest"); 
            delay(500); 
            ESP.restart(); 
        }
    }

    if (isFactoryResetPressed) {
        if (millis() - factoryResetPressTime > 3000) { 
            myNex.writeNum("page4.bco", 0); 
            delay(500); 
            handleRestoreDefaults(); 
        }
    }

    int buttonState = digitalRead(BUTTON_PIN);
    if (buttonState == LOW && !buttonWasPressed) { 
        buttonPressTime = millis(); 
        buttonWasPressed = true; 
    } 
    else if (buttonState == HIGH && buttonWasPressed) {
        buttonWasPressed = false; 
        unsigned long pressDuration = millis() - buttonPressTime;
        if (pressDuration >= LONG_PRESS_TIME) { 
            trybTestowy = !trybTestowy; 
            triggerBlink(8, 50); 
            clickCount = 0; 
        } 
        else if (pressDuration > 20) { 
            clickCount++; 
            lastClickTime = millis(); 
        }
    }

    if (clickCount > 0 && (millis() - lastClickTime) > CLICK_TIMEOUT) { 
        if (clickCount >= 3) {
            setupWiFi(); 
        }
        clickCount = 0; 
    }

    if (millis() - lastDiagnosticTime >= 2000) {
        lastDiagnosticTime = millis();
        Wire.beginTransmission(0x58); 
        statusDAC = (Wire.endTransmission() == 0);
        
        Wire.beginTransmission(0x48); 
        statusADS = (Wire.endTransmission() == 0);
        
        statusSD = (SD.cardType() != CARD_NONE); 
        float diag_u = pzem.voltage(); 
        statusPZEM = !isnan(diag_u);
        
        NextionSerial.print("sendme"); 
        NextionSerial.write(0xFF); 
        NextionSerial.write(0xFF); 
        NextionSerial.write(0xFF);
    }

    // ====================================================================
    // KROK 4: PĘTLA STERUJĄCA - INTELIGENTNY ODCZYT ADC
    // ====================================================================
    if (millis() - lastFastUpdate >= 50) {
        lastFastUpdate = millis(); 
        int16_t adc_surowe = 0; 
        
        if (statusADS) {
            // Abstrakcja sprzętowa - wybór wejścia na podstawie ustawień
            if (typZadajnika == 0) {
                adc_surowe = ads.readADC_SingleEnded(0); // Pin A0 (Dzielnik napięcia 0-10V)
            } else {
                adc_surowe = ads.readADC_SingleEnded(1); // Pin A1 (Rezystor prądowy mA)
            }
        }
        
        float napiecie_na_pinie = ads.computeVolts(adc_surowe); 
        float aktualny_odczyt = napiecie_na_pinie * WSPOLCZYNNIK_DZIELNIKA; 
        
        if (isnan(aktualny_odczyt)) aktualny_odczyt = 0.0; 
        if (aktualny_odczyt < 0.05) aktualny_odczyt = 0.0; 
        if (aktualny_odczyt > 10.5) aktualny_odczyt = 10.5;
        
        if (napiecieZadajnika == 0.0 && aktualny_odczyt > 0.0) {
            napiecieZadajnika = aktualny_odczyt; 
        } else {
            napiecieZadajnika = (aktualny_odczyt * filtr_waga) + (napiecieZadajnika * (1.0 - filtr_waga));
        }

        if (!systemON) { 
            currentDac1 = napiecieZadajnika * (dac1Ratio / 100.0); 
            currentDac2 = napiecieZadajnika * (dac2Ratio / 100.0); 
        } else {
            if (isnan(Output)) {
                Output = minDacVolt; 
            }
            currentDac1 = Output * (dac1Ratio / 100.0); 
            currentDac2 = Output * (dac2Ratio / 100.0); 
            
            if (currentDac1 < minDacVolt) currentDac1 = minDacVolt; 
            if (currentDac2 < minDacVolt) currentDac2 = minDacVolt;
        }

        float finalDac1 = currentDac1 + dac1Calib; 
        float finalDac2 = currentDac2 + dac2Calib;
        
        if (finalDac1 < 0.0) finalDac1 = 0.0; 
        if (finalDac1 > maxDacVolt) finalDac1 = maxDacVolt;
        if (finalDac2 < 0.0) finalDac2 = 0.0; 
        if (finalDac2 > maxDacVolt) finalDac2 = maxDacVolt;

        if (statusDAC) {
            uint16_t mv_dac1 = (uint16_t)(finalDac1 * 1000.0); 
            uint16_t mv_dac2 = (uint16_t)(finalDac2 * 1000.0);
            dac.setDACOutVoltage(mv_dac1, 0); 
            dac.setDACOutVoltage(mv_dac2, 1);
        }
    }

    if (millis() - lastPIDTime >= 200) {
        lastPIDTime = millis(); 
        float i = pzem.current(); 
        
        if (isnan(i)) i = 0.0;
        
        if (trybTestowy) { 
            int pot_raw = analogRead(PIN_POT_SYMULACJA); 
            i = (pot_raw / 4095.0) * 50.0; 
        }
        
        if (current_Amps == 0.0) {
            current_Amps = i; 
        } else {
            current_Amps = (i * 0.4) + (current_Amps * 0.6); 
        }
        
        if (systemON && current_Amps >= (maxLimit + overloadLimit)) { 
            stopRegulator(); 
            trippedByOverload = true; 
        }
        
        if (!systemON && trippedByOverload && modeAUTO) { 
            if (current_Amps <= (minLimit + recoveryLimit)) {
                startRegulator(); 
            }
        }
        
        if (systemON) { 
            Setpoint = maxLimit - 1.0; 
            
            // Wyliczamy absolutny błąd (odchylenie od celu)
            float error = abs(Setpoint - current_Amps);
            
            // INTELIGENTNY DEADBAND
            if (error <= deadbandAmps) {
                // Prąd w granicach tolerancji. Oszukujemy PID, że wszystko jest idealnie.
                // Dzięki temu PID zatrzyma się i "zamrozi" obecne napięcie falownika.
                Input = Setpoint; 
            } else {
                // Prąd uciekł za daleko - PID widzi rzeczywistość i oblicza korektę napięcia.
                Input = current_Amps; 
            }

            myPID.Compute(); 
        }
    }

    if (millis() - lastUpdate >= 1000) {
        lastUpdate = millis(); 
        pzem_u = pzem.voltage(); 
        pzem_p = pzem.power(); 
        pzem_pf = pzem.pf(); 
        dht_t = dht.readTemperature(); 
        dht_h = dht.readHumidity();
        
        if (isnan(pzem_u)) pzem_u = 0.0; 
        if (isnan(pzem_p)) pzem_p = 0.0; 
        if (isnan(pzem_pf)) pzem_pf = 0.0;

        if (pzem_u > 0 || trybTestowy) { 
            pzem_s = pzem_u * current_Amps; 
            pzem_q = (pzem_s > pzem_p) ? sqrt(pzem_s * pzem_s - pzem_p * pzem_p) : 0; 
            
            char buf[16]; 
            sprintf(buf, "%.3f A", current_Amps); 
            
            if (!isResetStage1Active()) {
                myNex.writeStr("granampery.txt", buf); 
            }
            
            myNex.writeStr("natgr.txt", buf);
            sprintf(buf, "%.1f V", pzem_u); 
            myNex.writeStr("napgr.txt", buf);
            sprintf(buf, "%.0f W", pzem_p); 
            myNex.writeStr("mocczy.txt", buf);
            sprintf(buf, "%.0f VA", pzem_s); 
            myNex.writeStr("mocpoz.txt", buf);
            sprintf(buf, "%.0f Var", pzem_q); 
            myNex.writeStr("mocbie.txt", buf);
            sprintf(buf, "%.2f", pzem_pf); 
            myNex.writeStr("wspmoc.txt", buf);
        } else { 
            pzem_s = 0; 
            pzem_q = 0; 
        }

        if (!isnan(dht_t)) { 
            myNex.writeStr("temperatura.txt", String(dht_t, 1)); 
            myNex.writeStr("wilgotnosc.txt", String(dht_h, 0)); 
        }
        
        char dacBuf[10];
        if (!isResetPressed) {
            sprintf(dacBuf, "%.2f V", currentDac1); 
            myNex.writeStr("pod1.txt", dacBuf); 
            myNex.writeStr("dac1.txt", dacBuf);
            
            sprintf(dacBuf, "%.2f V", currentDac2); 
            myNex.writeStr("pod2.txt", dacBuf); 
            myNex.writeStr("dac2.txt", dacBuf);
        }

        if (SD.cardType() != CARD_NONE) { 
            float gb = SD.totalBytes() / (1024.0 * 1024.0 * 1024.0); 
            myNex.writeStr("sd.txt", String(gb, 1) + " GB"); 
        } else {
            myNex.writeStr("sd.txt", "NO SD"); 
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            myNex.writeStr("page4.ip.txt", WiFi.localIP().toString());
        } else {
            myNex.writeStr("page4.ip.txt", "192.168.5.1");
        }
        
        int apState = memory.getInt("apState", 1); 
        myNex.writeStr("page4.wifilokalonoff.txt", apState == 1 ? "ON" : "OFF");
    }

    // ====================================================================
    // REJESTRATOR DIAGNOSTYCZNY (Zapis wszystkiego w 1 linii na SD)
    // ====================================================================
    if (isLoggingActive) {
        if (millis() > logEndTime) {
            isLoggingActive = false; // Zegar wybił koniec
        } else if (millis() - lastLogWriteTime >= 1000) { // Zapis równo co 1 sekundę
            lastLogWriteTime = millis();
            
            if (SD.cardType() != CARD_NONE) {
                // Użycie wygenerowanej, dynamicznej nazwy pliku!
                File f = SD.open(currentLogFileName.c_str(), FILE_APPEND);
                if (f) {
                    // Budowa paczki z użyciem średników zamiast przecinków
                    String logLine = String(millis()) + ";" + String(systemON) + ";" + String(modeAUTO) + ";" + 
                                     String(trippedByOverload) + ";" + String(current_Amps, 2) + ";" + 
                                     String(Setpoint, 2) + ";" + String(pzem_u, 1) + ";" + String(pzem_p, 0) + ";" + 
                                     String(pzem_s, 0) + ";" + String(pzem_q, 0) + ";" + String(pzem_pf, 2) + ";" + 
                                     String(currentDac1, 2) + ";" + String(currentDac2, 2) + ";" + 
                                     String(dht_t, 1) + ";" + String(dht_h, 0) + ";" + String(Kp, 2) + ";" + 
                                     String(Ki, 2) + ";" + String(Kd, 2) + ";" + String(minLimit, 1) + ";" + String(maxLimit, 1);
                    
                    // PRO TRIK: Zamiana kropki na przecinek, żeby polski Excel widział to jako liczby
                    logLine.replace(".", ",");
                    
                    f.println(logLine);
                    f.close();
                }
            }
        }
    }
}

bool isResetStage1Active() { 
    return resetStage3; 
}