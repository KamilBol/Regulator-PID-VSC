// ================================================================
// REGULATOR PID - V16.2 (THICK JSON, PEŁNY HTML, CLOUD COMMANDS)
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
#include <WiFiClientSecure.h>
#include <ESPmDNS.h>           
#include <WebServer.h>         
#include <Update.h>            
#include <HTTPClient.h>
#include <PubSubClient.h>
#include "strona_www.h"

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

WiFiClientSecure espClient; 
PubSubClient mqtt(espClient);

// --- PARAMETRY PID ---
double Setpoint; double Input; double Output;
double Kp = 0.5, Ki = 0.1, Kd = 0.15; 
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// --- ZMIENNE KONFIGURACJI SPRZĘTOWEJ ---
int outMode = 0; float minDacVolt = 3.5; float maxDacVolt = 10.0; 
float dac1Calib = 0.0; float dac2Calib = 0.0;

// --- ZMIENNE SIECIOWE I CHMUROWE ---
String routerSSID = ""; String routerPASS = "";
String mqtt_server = ""; String mqtt_user = ""; String mqtt_pass = ""; String mqtt_id = "Granulator_01";

// ================================================================
// ZMIENNE GLOBALNE
// ================================================================
float minLimit = 10.0; float maxLimit = 40.0; float dac1Ratio = 100.0; float dac2Ratio = 90.0;  
float overloadLimit = 7.0; float recoveryLimit = 2.0; 

bool systemON = false; bool modeAUTO = true; bool trippedByOverload = false;
float napiecieZadajnika = 0.0; float currentDac1 = 0.0; float currentDac2 = 0.0;

unsigned long lastUpdate = 0; unsigned long lastFastUpdate = 0;
unsigned long lastPIDTime = 0; unsigned long lastDiagnosticTime = 0;
unsigned long lastMqttReconnect = 0; unsigned long lastMqttPublish = 0;

unsigned long resetPressTime = 0; bool isResetPressed = false;
bool resetStage1 = false; bool resetStage2 = false; bool resetStage3 = false;
unsigned long factoryResetPressTime = 0; bool isFactoryResetPressed = false;

const float WSPOLCZYNNIK_DZIELNIKA = 1.982; float filtr_waga = 0.15;
const int BUTTON_PIN = 0; bool trybTestowy = false; float current_Amps = 0.0;

bool isWifiAPActive = false; unsigned long buttonPressTime = 0; unsigned long lastClickTime = 0;
int clickCount = 0; bool buttonWasPressed = false; const unsigned long CLICK_TIMEOUT = 800; const unsigned long LONG_PRESS_TIME = 3000; 

unsigned long ledTimer = 0; int ledState = LOW; int blinkCount = 0; int blinkMax = 0; int blinkDuration = 100; 

bool statusDAC = false; bool statusADS = false; bool statusSD = false; bool statusPZEM = false;
unsigned long lastNextionResponseTime = 0;

float pzem_u = 0, pzem_p = 0, pzem_pf = 0, pzem_s = 0, pzem_q = 0;
float dht_t = 0, dht_h = 0;

// ================================================================
// DEKLARACJE WYPRZEDZAJĄCE (Zabezpieczenie przed błędami kompilatora)
// ================================================================
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

// ================================================================
// FUNKCJE POMOCNICZE
// ================================================================
String cleanHostAddress(String host) {
    String clean = host; clean.replace("http://", ""); clean.replace("https://", "");
    int colonIndex = clean.indexOf(':'); if (colonIndex > 0) clean = clean.substring(0, colonIndex); 
    clean.trim(); return clean;
}

void triggerBlink(int times, int duration) {
    blinkMax = times * 2; blinkCount = 0; blinkDuration = duration;
    ledState = HIGH; digitalWrite(PIN_LED, ledState); ledTimer = millis(); blinkCount++;
}

void handleLED() {
    if (blinkCount > 0 && blinkCount < blinkMax) {
        if (millis() - ledTimer >= blinkDuration) { ledTimer = millis(); ledState = !ledState; digitalWrite(PIN_LED, ledState); blinkCount++; }
    } else if (blinkCount >= blinkMax) { digitalWrite(PIN_LED, LOW); blinkCount = 0; }
}

void initSD() {
    SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS); 
    if(SD.begin(PIN_SD_CS)) { 
        statusSD = true; File f = SD.open("/AI_LOG.txt", FILE_APPEND); 
        if(f) { f.println("=== START SYSTEMU - V16.2 ==="); f.close(); }
        Serial.println("[SD] Karta aktywna.");
    } else { 
        statusSD = false; Serial.println("[SD] BLAD KARTY!"); myNex.writeStr("sd.txt", "ERR"); 
    }
}

void applyOutputMode() {
    myPID.SetOutputLimits(minDacVolt, maxDacVolt); memory.putInt("outMode", outMode);
    Serial.printf("[SYS] Konfiguracja: Tryb %d | Podloga: %.2fV | Sufit: %.2fV\n", outMode, minDacVolt, maxDacVolt);
}

void toggleLocalWiFi() {
    int apState = memory.getInt("apState", 1); apState = (apState == 1) ? 0 : 1; 
    if (apState == 0 && routerSSID == "") apState = 1;
    memory.putInt("apState", apState);
    if (apState == 1) {
        WiFi.mode(routerSSID != "" ? WIFI_AP_STA : WIFI_AP);
        IPAddress local_ip(192, 168, 5, 1); IPAddress gateway(192, 168, 5, 1); IPAddress subnet(255, 255, 255, 0);
        WiFi.softAPConfig(local_ip, gateway, subnet); WiFi.softAP("RegulatorPID");
    } else { WiFi.softAPdisconnect(true); WiFi.mode(WIFI_STA); }
}

void performRemoteOTA(String url) {
    Serial.println("[OTA] Otrzymano rozkaz aktualizacji z chmury!"); stopRegulator(); 
    HTTPClient http; http.begin(url); int httpCode = http.GET();
    if (httpCode == 200) {
        int contentLength = http.getSize(); bool canBegin = Update.begin(contentLength);
        if (canBegin) {
            WiFiClient& client = http.getStream(); size_t written = Update.writeStream(client);
            if (written == contentLength) { Update.end(); delay(3000); ESP.restart(); }
        }
    }
    http.end();
}

void publishSDList() {
    if(SD.cardType() == CARD_NONE) return;
    File root = SD.open("/"); String json = "["; File file = root.openNextFile(); bool first = true;
    while(file){
        if (!file.isDirectory()) {
            if(!first) json += ",";
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

// ================================================================
// OBSŁUGA MQTT Z CHMURY (FAZA 4)
// ================================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String msg = ""; for (int i = 0; i < length; i++) msg += (char)payload[i];
    Serial.println("[MQTT] Rozkaz: " + msg);

    if (msg == "SYSTEM=ON") { startRegulator(); }
    else if (msg == "SYSTEM=OFF") { stopRegulator(); }
    else if (msg == "MODE=AUTO") { modeAUTO = true; myNex.writeStr("pracaautoman.txt", "AUT"); }
    else if (msg == "MODE=MAN") { modeAUTO = false; myNex.writeStr("pracaautoman.txt", "MAN"); }
    else if (msg == "CMD:RESTART") { ESP.restart(); }
    else if (msg == "CMD:SDLIST") { publishSDList(); }
    else if (msg.startsWith("OTA=")) { performRemoteOTA(msg.substring(4)); }
    
    // Parsowanie zaawansowanych ustawien z formularzy Serwera HUB
    else if (msg.startsWith("CMD:LIMITS:")) {
        int p1 = msg.indexOf(':', 11);
        minLimit = msg.substring(11, p1).toFloat(); maxLimit = msg.substring(p1 + 1).toFloat();
        memory.putFloat("minLim", minLimit); memory.putFloat("maxLim", maxLimit); updateSettingsScreen();
    }
    else if (msg.startsWith("CMD:RATIOS:")) {
        int p1 = msg.indexOf(':', 11);
        dac1Ratio = msg.substring(11, p1).toFloat(); dac2Ratio = msg.substring(p1 + 1).toFloat();
        memory.putFloat("dac1Ratio", dac1Ratio); memory.putFloat("dac2Ratio", dac2Ratio);
    }
    else if (msg.startsWith("CMD:PID:")) {
        int p1 = msg.indexOf(':', 8); int p2 = msg.indexOf(':', p1 + 1);
        Kp = msg.substring(8, p1).toFloat(); Ki = msg.substring(p1 + 1, p2).toFloat(); Kd = msg.substring(p2 + 1).toFloat();
        myPID.SetTunings(Kp, Ki, Kd); memory.putFloat("kp", Kp); memory.putFloat("ki", Ki); memory.putFloat("kd", Kd);
    }
    else if (msg.startsWith("CMD:ALARMS:")) {
        int p1 = msg.indexOf(':', 11);
        overloadLimit = msg.substring(11, p1).toFloat(); recoveryLimit = msg.substring(p1 + 1).toFloat();
        memory.putFloat("ovrLimit", overloadLimit); memory.putFloat("recLimit", recoveryLimit);
    }
    else if (msg.startsWith("CMD:VOLT:")) {
        int p1 = msg.indexOf(':', 9);
        minDacVolt = msg.substring(9, p1).toFloat(); maxDacVolt = msg.substring(p1 + 1).toFloat();
        memory.putFloat("minDacVolt", minDacVolt); memory.putFloat("maxDacVolt", maxDacVolt); applyOutputMode();
    }
    else if (msg.startsWith("CMD:OUTMODE:")) {
        outMode = msg.substring(12).toInt(); applyOutputMode();
    }
    else if (msg.startsWith("CMD:CALIB:")) {
        int p1 = msg.indexOf(':', 10);
        dac1Calib = msg.substring(10, p1).toFloat(); dac2Calib = msg.substring(p1 + 1).toFloat();
        memory.putFloat("dac1Calib", dac1Calib); memory.putFloat("dac2Calib", dac2Calib);
    }
    else if (msg == "CMD:SAVEDEF") {
        memory.putFloat("d_minL", minLimit); memory.putFloat("d_maxL", maxLimit); memory.putFloat("d_kp", Kp);
        memory.putFloat("d_ki", Ki); memory.putFloat("d_kd", Kd); memory.putFloat("d_ovL", overloadLimit);
        memory.putFloat("d_recL", recoveryLimit); memory.putFloat("d_d1R", dac1Ratio); memory.putFloat("d_d2R", dac2Ratio);
        memory.putFloat("d_minV", minDacVolt); memory.putFloat("d_maxV", maxDacVolt); memory.putInt("d_outM", outMode);
    }
    else if (msg == "CMD:RESTOREDEF") { handleRestoreDefaults(); }
    else if (msg.startsWith("CMD:WIFI:")) {
        int p1 = msg.indexOf(':', 9);
        routerSSID = msg.substring(9, p1); memory.putString("ssid", routerSSID);
        String p = msg.substring(p1 + 1); if(p != "") { routerPASS = p; memory.putString("pass", routerPASS); }
        ESP.restart();
    }
    else if (msg.startsWith("CMD:MQTT:")) {
        int p1 = msg.indexOf(':', 9); int p2 = msg.indexOf(':', p1 + 1); int p3 = msg.indexOf(':', p2 + 1);
        mqtt_server = cleanHostAddress(msg.substring(9, p1)); memory.putString("mq_srv", mqtt_server);
        mqtt_user = msg.substring(p1 + 1, p2); memory.putString("mq_usr", mqtt_user);
        String pas = msg.substring(p2 + 1, p3); if(pas != "") { mqtt_pass = pas; memory.putString("mq_pas", mqtt_pass); }
        mqtt_id = msg.substring(p3 + 1); memory.putString("mq_id", mqtt_id);
        ESP.restart();
    }
    triggerBlink(2, 100); 
}

void handleMQTT() {
    if (mqtt_server == "" || routerSSID == "") return;
    if (WiFi.status() != WL_CONNECTED || WiFi.localIP().toString() == "0.0.0.0") return;

    if (!mqtt.connected()) {
        if (millis() - lastMqttReconnect > 15000) {
            lastMqttReconnect = millis();
            espClient.stop(); espClient.setInsecure();
            Serial.print("[MQTT] Proba logowania do chmury...");
            String clientId = mqtt_id + "-" + String(random(0xffff), HEX);
            if (mqtt.connect(clientId.c_str(), mqtt_user.c_str(), mqtt_pass.c_str())) {
                Serial.println(" SUKCES!");
                String subTopic = "biuro/" + mqtt_id + "/rozkazy";
                mqtt.subscribe(subTopic.c_str());
            } else {
                Serial.print(" BLAD, kod=");
                Serial.println(mqtt.state());
            }
        }
    } else {
        mqtt.loop();
        
        // PUBLIKACJA GRUBEJ PACZKI JSON CO 3 SEKUNDY
        if (millis() - lastMqttPublish > 3000) {
            lastMqttPublish = millis();
            
            float safe_temp = isnan(dht_t) ? 0.0 : dht_t;
            float safe_hum = isnan(dht_h) ? 0.0 : dht_h;
            float safe_pf = isnan(pzem_pf) ? 0.0 : pzem_pf;

            String json;
            json.reserve(1200); 
            
            json = "{";
            json += "\"amp\":" + String(current_Amps, 2) + ",\"setp\":" + String(Setpoint, 2) + ",";
            json += "\"sysON\":" + String(systemON ? 1 : 0) + ",\"autoM\":" + String(modeAUTO ? 1 : 0) + ",";
            json += "\"trip\":" + String(trippedByOverload ? 1 : 0) + ",\"volt\":" + String(pzem_u, 1) + ",";
            json += "\"pow\":" + String(pzem_p, 0) + ",\"ap_pow\":" + String(pzem_s, 0) + ",\"re_pow\":" + String(pzem_q, 0) + ",";
            json += "\"pf\":" + String(safe_pf, 2) + ",\"temp\":" + String(safe_temp, 1) + ",\"hum\":" + String(safe_hum, 0) + ",";
            json += "\"dac\":" + String(currentDac1, 2) + ",\"dac2v\":" + String(currentDac2, 2) + ",";
            
            json += "\"minL\":" + String(minLimit, 1) + ",\"maxL\":" + String(maxLimit, 1) + ",";
            json += "\"kp\":" + String(Kp, 3) + ",\"ki\":" + String(Ki, 3) + ",\"kd\":" + String(Kd, 3) + ",";
            json += "\"dac1R\":" + String(dac1Ratio, 0) + ",\"dac2R\":" + String(dac2Ratio, 0) + ",";
            json += "\"ovL\":" + String(overloadLimit, 1) + ",\"recL\":" + String(recoveryLimit, 1) + ",";
            json += "\"dac1C\":" + String(dac1Calib, 2) + ",\"dac2C\":" + String(dac2Calib, 2) + ",";
            json += "\"minV\":" + String(minDacVolt, 2) + ",\"maxV\":" + String(maxDacVolt, 2) + ",\"outM\":" + String(outMode) + ",";
            
            json += "\"pzem\":" + String(statusPZEM ? 1 : 0) + ",\"nex\":" + String((millis() - lastNextionResponseTime < 5000) ? 1 : 0) + ",";
            json += "\"ads\":" + String(statusADS ? 1 : 0) + ",\"dac_st\":" + String(statusDAC ? 1 : 0) + ",";
            json += "\"dht_st\":" + String(!isnan(dht_t) ? 1 : 0) + ",\"sd\":" + String(statusSD ? 1 : 0) + ",";
            json += "\"iso\":" + String((statusDAC || statusADS) ? 1 : 0) + ",";
            
            json += "\"up_s\":" + String(millis() / 1000) + ",\"heap_pct\":" + String(((float)ESP.getFreeHeap() / ESP.getHeapSize()) * 100.0, 1) + ",";
            json += "\"cpu\":" + String(ESP.getCpuFreqMHz()) + ",\"chip\":\"" + String(ESP.getChipModel()) + "\",";
            json += "\"sketch_k\":" + String(ESP.getSketchSize() / 1024) + ",\"cli\":" + String(WiFi.softAPgetStationNum()) + ",";
            json += "\"wifi_s\":\"" + routerSSID + "\",\"mq_srv\":\"" + mqtt_server + "\",\"mq_usr\":\"" + mqtt_user + "\",\"mq_id\":\"" + mqtt_id + "\",";
            json += "\"ip\":\"" + WiFi.localIP().toString() + "\"}";
            
            String pubTopic = "biuro/" + mqtt_id + "/dane";
            mqtt.beginPublish(pubTopic.c_str(), json.length(), false);
            mqtt.print(json);
            mqtt.endPublish();
        }
    }
}

// ================================================================
// BRAMKA AUTORYZACJI WWW (Ochroniarz LOKALNY)
// ================================================================
bool checkAuth() {
    if (!server.authenticate("admin", "regpid12")) {
        server.requestAuthentication();
        return false;
    }
    return true;
}



// ================================================================
// FUNKCJE SERWERA WWW LOKALNEGO API
// ================================================================
void handleRoot() { 
    if(!checkAuth()) return;
    server.send(200, "text/html", INDEX_HTML); 
}

void handleApiData() {
    if(!checkAuth()) return;
    String json = "{";
    json += "\"amp\":\"" + String(current_Amps, 2) + "\",";
    json += "\"setp\":\"" + String(Setpoint, 2) + "\",";
    json += "\"dac\":\"" + String(currentDac1, 2) + "\",";
    json += "\"dac2v\":\"" + String(currentDac2, 2) + "\",";
    json += "\"trip\":\"" + String(trippedByOverload ? 1 : 0) + "\",";
    json += "\"sysON\":\"" + String(systemON ? 1 : 0) + "\",";
    json += "\"autoM\":\"" + String(modeAUTO ? 1 : 0) + "\",";
    json += "\"volt\":\"" + String(pzem_u, 1) + "\",";
    json += "\"pow\":\"" + String(pzem_p, 0) + "\",";
    json += "\"ap_pow\":\"" + String(pzem_s, 0) + "\",";
    json += "\"re_pow\":\"" + String(pzem_q, 0) + "\",";
    json += "\"pf\":\"" + String(pzem_pf, 2) + "\",";
    json += "\"temp\":\"" + String(dht_t, 1) + "\",";
    json += "\"hum\":\"" + String(dht_h, 0) + "\",";
    json += "\"minL\":\"" + String(minLimit, 1) + "\",";
    json += "\"maxL\":\"" + String(maxLimit, 1) + "\",";
    json += "\"kp\":\"" + String(Kp, 3) + "\",";
    json += "\"ki\":\"" + String(Ki, 3) + "\",";
    json += "\"kd\":\"" + String(Kd, 3) + "\",";
    json += "\"outM\":\"" + String(outMode) + "\",";
    json += "\"dac1R\":\"" + String(dac1Ratio, 0) + "\",";
    json += "\"dac2R\":\"" + String(dac2Ratio, 0) + "\",";
    json += "\"ovL\":\"" + String(overloadLimit, 1) + "\",";
    json += "\"recL\":\"" + String(recoveryLimit, 1) + "\",";
    json += "\"dac1C\":\"" + String(dac1Calib, 2) + "\",";
    json += "\"dac2C\":\"" + String(dac2Calib, 2) + "\",";
    json += "\"minV\":\"" + String(minDacVolt, 2) + "\",";
    json += "\"maxV\":\"" + String(maxDacVolt, 2) + "\",";
    json += "\"wifi_s\":\"" + routerSSID + "\",";
    json += "\"mq_srv\":\"" + mqtt_server + "\",";
    json += "\"mq_usr\":\"" + mqtt_user + "\",";
    json += "\"mq_id\":\"" + mqtt_id + "\"";
    json += "}";
    server.send(200, "application/json", json);
}

void handleApiHealth() {
    if(!checkAuth()) return;
    bool statusNex = (millis() - lastNextionResponseTime < 5000); 
    float heapPct = ((float)ESP.getFreeHeap() / ESP.getHeapSize()) * 100.0;
    
    String json = "{";
    json += "\"pzem\":\"" + String(statusPZEM ? 1 : 0) + "\",";
    json += "\"dac\":\"" + String(statusDAC ? 1 : 0) + "\",";
    json += "\"ads\":\"" + String(statusADS ? 1 : 0) + "\",";
    json += "\"dht\":\"" + String(!isnan(dht_t) ? 1 : 0) + "\",";
    json += "\"sd\":\"" + String(statusSD ? 1 : 0) + "\",";
    json += "\"nex\":\"" + String(statusNex ? 1 : 0) + "\",";
    
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
    
    json += "\"up\":\"" + upStr + "\",";
    json += "\"heap_pct\":\"" + String(heapPct, 1) + "\",";
    json += "\"cpu\":\"" + String(ESP.getCpuFreqMHz()) + " MHz\",";
    json += "\"chip\":\"" + String(ESP.getChipModel()) + " (" + String(ESP.getChipCores()) + " Core)\",";
    json += "\"sketch\":\"" + String(ESP.getSketchSize() / 1024) + " KB / " + String((ESP.getSketchSize() + ESP.getFreeSketchSpace()) / 1024) + " KB\",";
    json += "\"clients\":\"" + String(WiFi.softAPgetStationNum()) + "\",";
    
    String routerIP = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "Brak (Tylko AP)";
    json += "\"router_ip\":\"" + routerIP + "\"";
    json += "}";
    server.send(200, "application/json", json);
}

void handleToggleSys() {
    if(!checkAuth()) return;
    if(systemON) stopRegulator(); else startRegulator();
    server.send(200, "text/plain", "OK");
}

void handleToggleMode() {
    if(!checkAuth()) return;
    modeAUTO = !modeAUTO;
    myNex.writeStr("pracaautoman.txt", modeAUTO ? "AUT" : "MAN");
    server.send(200, "text/plain", "OK");
}

void handleSetOutMode() {
    if(!checkAuth()) return;
    if (server.hasArg("m")) {
        outMode = server.arg("m").toInt();
        applyOutputMode();
        triggerBlink(1, 1000); 
    }
    server.send(200, "text/plain", "OK");
}

void handleSetRatios() {
    if(!checkAuth()) return;
    if (server.hasArg("r1") && server.hasArg("r2")) {
        dac1Ratio = server.arg("r1").toFloat();
        dac2Ratio = server.arg("r2").toFloat();
        if (dac1Ratio < 0.0) dac1Ratio = 0.0; if (dac1Ratio > 100.0) dac1Ratio = 100.0;
        if (dac2Ratio < 0.0) dac2Ratio = 0.0; if (dac2Ratio > 100.0) dac2Ratio = 100.0;
        memory.putFloat("dac1Ratio", dac1Ratio);
        memory.putFloat("dac2Ratio", dac2Ratio);
        triggerBlink(1, 1000);
    }
    server.send(200, "text/plain", "OK");
}

void handleSetAlarms() {
    if(!checkAuth()) return;
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
    if(!checkAuth()) return;
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
    if(!checkAuth()) return;
    if (server.hasArg("kp") && server.hasArg("ki") && server.hasArg("kd")) {
        Kp = server.arg("kp").toFloat();
        Ki = server.arg("ki").toFloat();
        Kd = server.arg("kd").toFloat();
        memory.putFloat("kp", Kp);
        memory.putFloat("ki", Ki);
        memory.putFloat("kd", Kd);
        myPID.SetTunings(Kp, Ki, Kd);
        triggerBlink(1, 1000);
    }
    server.send(200, "text/plain", "OK");
}

void handleSetCalib() {
    if(!checkAuth()) return;
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
    if(!checkAuth()) return;
    if (server.hasArg("min") && server.hasArg("max")) {
        minDacVolt = server.arg("min").toFloat();
        maxDacVolt = server.arg("max").toFloat();
        if (minDacVolt < 0.0) minDacVolt = 0.0;
        if (minDacVolt > 10.0) minDacVolt = 10.0;
        if (maxDacVolt < minDacVolt) maxDacVolt = minDacVolt;
        if (maxDacVolt > 10.0) maxDacVolt = 10.0;
        memory.putFloat("minDacVolt", minDacVolt);
        memory.putFloat("maxDacVolt", maxDacVolt);
        myPID.SetOutputLimits(minDacVolt, maxDacVolt);
        triggerBlink(1, 1000);
    }
    server.send(200, "text/plain", "OK");
}

void handleSetWiFi() {
    if(!checkAuth()) return;
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
    if(!checkAuth()) return;
    if (server.hasArg("srv")) {
        mqtt_server = cleanHostAddress(server.arg("srv"));
        memory.putString("mq_srv", mqtt_server);
    }
    if (server.hasArg("usr")) {
        mqtt_user = server.arg("usr");
        memory.putString("mq_usr", mqtt_user);
    }
    if (server.hasArg("pas") && server.arg("pas") != "") {
        mqtt_pass = server.arg("pas");
        memory.putString("mq_pas", mqtt_pass);
    }
    if (server.hasArg("id")) {
        mqtt_id = server.arg("id");
        memory.putString("mq_id", mqtt_id);
    }
    
    server.send(200, "text/plain", "OK");
    delay(500);
    ESP.restart();
}

void handleRestart() {
    if(!checkAuth()) return;
    server.send(200, "text/plain", "OK");
    delay(500);
    ESP.restart();
}

void handleSaveDefaults() {
    if(!checkAuth()) return;
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
    server.send(200, "text/plain", "OK");
}

void handleRestoreDefaults() {
    if(!checkAuth()) return;
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
    if(!checkAuth()) return;
    if(SD.cardType() == CARD_NONE) { 
        server.send(200, "application/json", "[]"); 
        return; 
    }
    File root = SD.open("/");
    String json = "[";
    File file = root.openNextFile();
    bool first = true;
    while(file){
        if (!file.isDirectory()) {
            if(!first) {
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
    if(!checkAuth()) return;
    if (!server.hasArg("f")) { 
        server.send(400, "text/plain", "Brak pliku"); 
        return; 
    }
    String path = "/" + server.arg("f");
    File file = SD.open(path, FILE_READ);
    if (!file) { 
        server.send(404, "text/plain", "Nie znaleziono"); 
        return; 
    }
    server.streamFile(file, "text/plain"); 
    file.close();
}

void setupWiFi() {
    int apState = memory.getInt("apState", 1);
    
    WiFi.disconnect(true);
    WiFi.softAPdisconnect(true);
    delay(100);

    if (routerSSID != "") {
        WiFi.mode(apState == 1 ? WIFI_AP_STA : WIFI_STA);
        WiFi.begin(routerSSID.c_str(), routerPASS.c_str());
        Serial.printf("[WIFI] Laczenie z routerem: %s\n", routerSSID.c_str());
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
        
        WiFi.softAP("RegulatorPID"); 
        Serial.println("[WIFI] Otwarto OTWARTA wewnetrzna siec maszyny (AP).");
    }

    if (MDNS.begin("granulator")) {
        Serial.println("[mDNS] Adres maszyny to: http://granulator.local");
    }
    
    server.on("/", HTTP_GET, handleRoot);
    server.on("/api/data", HTTP_GET, handleApiData);
    server.on("/api/health", HTTP_GET, handleApiHealth);
    server.on("/api/toggle_sys", HTTP_POST, handleToggleSys);
    server.on("/api/toggle_mode", HTTP_POST, handleToggleMode);
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
    server.on("/api/save_defaults", HTTP_POST, handleSaveDefaults);
    server.on("/api/restore_defaults", HTTP_POST, handleRestoreDefaults);
    server.on("/api/sd_list", HTTP_GET, handleSDList);
    server.on("/sd_read", HTTP_GET, handleSDRead);
    
    server.on("/update", HTTP_POST, []() {
        if(!checkAuth()) return;
        server.sendHeader("Connection", "close");
        server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
        ESP.restart();
    }, []() {
        if(!checkAuth()) return;
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
    Serial.println("\n[WIFI] Telefon polaczyl sie z maszyna!");
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
    
    if (napiecieZadajnika < minDacVolt) Output = minDacVolt;
    else if (napiecieZadajnika > maxDacVolt) Output = maxDacVolt;
    else Output = napiecieZadajnika;
    
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
// PARSER DOTYKU EKRANU I MECHANIZM RESETU EKRANOWEGO
// ================================================================
int activeButtonID = 0;          
unsigned long buttonHoldTimer = 0; 
bool isButtonHeld = false;       

void handleNextionInput() {
    while (NextionSerial.available()) { 
        byte b = NextionSerial.read();  
        lastNextionResponseTime = millis(); 
        
        if (b == 0x65) {                
            delay(15);                   
            if (NextionSerial.available() >= 6) {
                byte pageId = NextionSerial.read(); 
                byte cmpId  = NextionSerial.read(); 
                byte event  = NextionSerial.read(); 
                NextionSerial.read(); 
                NextionSerial.read(); 
                NextionSerial.read(); 

                if (pageId == 0) {
                    if (cmpId == 11 && event == 0x01) { 
                        if(systemON) stopRegulator(); else startRegulator(); 
                    }
                    if (cmpId == 12 && event == 0x01) { 
                        modeAUTO = !modeAUTO; 
                        myNex.writeStr("pracaautoman.txt", modeAUTO ? "AUT" : "MAN"); 
                    }
                    if (cmpId == 8) { 
                        if (event == 0x01) { 
                            isResetPressed = true; 
                            resetPressTime = millis(); 
                            resetStage1 = false; resetStage2 = false; resetStage3 = false;
                        }
                        else if (event == 0x00) { 
                            isResetPressed = false; 
                            resetStage1 = false; resetStage2 = false; resetStage3 = false; 
                            myNex.writeNum("pod2.bco", 65535);
                            myNex.writeNum("pod1.bco", 65535);
                            myNex.writeNum("granampery.bco", 65535);
                        }
                    }
                }
                
                if (pageId == 2) {
                    if (event == 0x01) { 
                        activeButtonID = cmpId; isButtonHeld = true; 
                        processButtonAction(activeButtonID); buttonHoldTimer = millis() + 400; 
                    } else if (event == 0x00) { 
                        activeButtonID = 0; isButtonHeld = false; 
                    }
                }

                if (pageId == 4) {
                    if (cmpId == 7 && event == 0x01) { 
                        toggleLocalWiFi();
                    }
                    if (cmpId == 8) {
                        if (event == 0x01) {
                            isFactoryResetPressed = true;
                            factoryResetPressTime = millis();
                        } else if (event == 0x00) {
                            isFactoryResetPressed = false;
                            myNex.writeNum("page4.bco", 65535); 
                        }
                    }
                }
            }
        }
    }
    if (isButtonHeld && activeButtonID > 0 && millis() > buttonHoldTimer) {
        processButtonAction(activeButtonID); buttonHoldTimer = millis() + 100; 
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

    delay(2000); 
    Serial.begin(115200); 
    Serial.println("\n\n--- SYSTEM V16.2 (CLOUD CORE STABLE) ---");

    memory.begin("regulator", false); 
    routerSSID = memory.getString("ssid", "");
    routerPASS = memory.getString("pass", "");
    mqtt_server = memory.getString("mq_srv", "");
    mqtt_user = memory.getString("mq_usr", "");
    mqtt_pass = memory.getString("mq_pas", "");
    mqtt_id = memory.getString("mq_id", "Granulator_01");

    mqtt_server = cleanHostAddress(mqtt_server);

    minLimit = memory.getFloat("minLim", 10.0); 
    maxLimit = memory.getFloat("maxLim", 40.0); 
    Kp = memory.getFloat("kp", 0.5);
    Ki = memory.getFloat("ki", 0.1);
    Kd = memory.getFloat("kd", 0.15);
    myPID.SetTunings(Kp, Ki, Kd); 
    
    outMode = memory.getInt("outMode", 0);
    
    minDacVolt = memory.getFloat("minDacVolt", 3.5);
    if(isnan(minDacVolt) || minDacVolt < 0.0) minDacVolt = 0.0;
    if(minDacVolt > 10.0) minDacVolt = 10.0;

    maxDacVolt = memory.getFloat("maxDacVolt", 10.0);
    if(isnan(maxDacVolt) || maxDacVolt < 0.0) maxDacVolt = 10.0;
    if(maxDacVolt > 10.0) maxDacVolt = 10.0;
    if(maxDacVolt < minDacVolt) maxDacVolt = minDacVolt;

    applyOutputMode(); 

    dac1Ratio = memory.getFloat("dac1Ratio", 100.0);
    dac2Ratio = memory.getFloat("dac2Ratio", 90.0);
    if (isnan(dac1Ratio) || dac1Ratio < 0.0) dac1Ratio = 0.0; if (dac1Ratio > 100.0) dac1Ratio = 100.0;
    if (isnan(dac2Ratio) || dac2Ratio < 0.0) dac2Ratio = 0.0; if (dac2Ratio > 100.0) dac2Ratio = 100.0;

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

    // ================================================================
    // FIX: POSZERZENIE BUFORA MQTT DLA "GRUBEJ PACZKI JSON" (V16.2)
    // ================================================================
    mqtt.setBufferSize(4096);

    WiFi.onEvent(onStationConnected, ARDUINO_EVENT_WIFI_AP_STACONNECTED);
    setupWiFi();

    NextionSerial.begin(9600, SERIAL_8N1, PIN_NEXT_RX, PIN_NEXT_TX);
    myNex.begin(9600);
    PzemSerial.begin(9600, SERIAL_8N1, PIN_PZEM_RX, PIN_PZEM_TX);
    dht.begin();
    
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL); 
    
    Wire.beginTransmission(0x48);
    statusADS = (Wire.endTransmission() == 0);
    if(statusADS) { ads.setGain(GAIN_TWOTHIRDS); ads.begin(0x48); }

    Wire.beginTransmission(0x58);
    statusDAC = (Wire.endTransmission() == 0);
    if(statusDAC) {
        dac.begin();
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
    myPID.SetSampleTime(200); 
    
    triggerBlink(2, 500); 
}

// ================================================================
// GŁÓWNA PĘTLA PROGRAMU
// ================================================================
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
        unsigned long holdTime = millis() - factoryResetPressTime;
        if (holdTime > 3000) {
            myNex.writeNum("page4.bco", 0); 
            delay(500);
            handleRestoreDefaults(); 
        }
    }

    int buttonState = digitalRead(BUTTON_PIN);
    if (buttonState == LOW && !buttonWasPressed) {
        buttonPressTime = millis();
        buttonWasPressed = true;
    } else if (buttonState == HIGH && buttonWasPressed) {
        buttonWasPressed = false;
        unsigned long pressDuration = millis() - buttonPressTime;

        if (pressDuration >= LONG_PRESS_TIME) {
            trybTestowy = !trybTestowy; 
            triggerBlink(8, 50); 
            clickCount = 0; 
        } else if (pressDuration > 20) { 
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

    if (millis() - lastFastUpdate >= 50) {
        lastFastUpdate = millis(); 

        int16_t adc_surowe = 0;
        if(statusADS) adc_surowe = ads.readADC_SingleEnded(0); 
        float napiecie_na_pinie = ads.computeVolts(adc_surowe); 
        float aktualny_odczyt = napiecie_na_pinie * WSPOLCZYNNIK_DZIELNIKA; 
        
        if(isnan(aktualny_odczyt)) aktualny_odczyt = 0.0;
        if(aktualny_odczyt < 0.05) aktualny_odczyt = 0.0;
        if(aktualny_odczyt > 10.5) aktualny_odczyt = 10.5;

        if (napiecieZadajnika == 0.0 && aktualny_odczyt > 0.0) {
            napiecieZadajnika = aktualny_odczyt; 
        } else {
            napiecieZadajnika = (aktualny_odczyt * filtr_waga) + (napiecieZadajnika * (1.0 - filtr_waga));
        }

        if (!systemON) {
            currentDac1 = napiecieZadajnika * (dac1Ratio / 100.0);
            currentDac2 = napiecieZadajnika * (dac2Ratio / 100.0); 
        } else {
            if (isnan(Output)) Output = minDacVolt; 
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

        if(statusDAC) {
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
            Serial.printf("[ALARM] Przekroczono limit +%.1fA! Awaryjne rozlaczenie!\n", overloadLimit);
            stopRegulator();
            trippedByOverload = true;
        }

        if (!systemON && trippedByOverload && modeAUTO) {
            if (current_Amps <= (minLimit + recoveryLimit)) {
                Serial.printf("[AUTO] Prad zmalal o +%.1fA. Odpalam maszyne ponownie.\n", recoveryLimit);
                startRegulator(); 
            }
        }

        if (systemON) {
            Setpoint = maxLimit - 1.0; 
            Input = current_Amps;         
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

        if(pzem_u > 0 || trybTestowy) { 
            pzem_s = pzem_u * current_Amps; 
            pzem_q = (pzem_s > pzem_p) ? sqrt(pzem_s*pzem_s - pzem_p*pzem_p) : 0; 
            
            char buf[16]; 
            sprintf(buf, "%.3f A", current_Amps); 
            
            if(!isResetStage1Active()) {
                myNex.writeStr("granampery.txt", buf); 
            }
            myNex.writeStr("natgr.txt", buf);
            
            sprintf(buf, "%.1f V", pzem_u); myNex.writeStr("napgr.txt", buf);
            sprintf(buf, "%.0f W", pzem_p); myNex.writeStr("mocczy.txt", buf);
            
            sprintf(buf, "%.0f VA", pzem_s); myNex.writeStr("mocpoz.txt", buf);
            sprintf(buf, "%.0f Var", pzem_q); myNex.writeStr("mocbie.txt", buf);
            sprintf(buf, "%.2f", pzem_pf); myNex.writeStr("wspmoc.txt", buf);
        } else {
            pzem_s = 0; pzem_q = 0;
        }

        if(!isnan(dht_t)) { 
            myNex.writeStr("temperatura.txt", String(dht_t, 1));
            myNex.writeStr("wilgotnosc.txt", String(dht_h, 0));
        }

        char dacBuf[10];
        if(!isResetPressed) {
            sprintf(dacBuf, "%.2f V", currentDac1); myNex.writeStr("pod1.txt", dacBuf); myNex.writeStr("dac1.txt", dacBuf);
            sprintf(dacBuf, "%.2f V", currentDac2); myNex.writeStr("pod2.txt", dacBuf); myNex.writeStr("dac2.txt", dacBuf);
        }

        if(SD.cardType() != CARD_NONE) { 
             float gb = SD.totalBytes() / (1024.0*1024.0*1024.0); 
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

        Serial.printf("%lu;%d;%d;%d;%.3f;%.1f;%.0f;%.2f;%.1f;%.1f;%.2f;%.2f;%.2f;%.2f\n", 
            millis(), systemON, modeAUTO, trippedByOverload, current_Amps, pzem_u, pzem_p, napiecieZadajnika, minLimit, maxLimit, Setpoint, Output, currentDac1, currentDac2);
    }
}

bool isResetStage1Active() {
    return resetStage3;
}