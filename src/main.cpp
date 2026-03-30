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
#include <Update.h>            

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

// --- PARAMETRY PID ---
double Setpoint;
double Input;
double Output;
double Kp = 0.5, Ki = 0.1, Kd = 0.15; 
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// --- ZMIENNE KONFIGURACJI SPRZĘTOWEJ ---
int outMode = 0; 
float currentMinDac = 3.5; 
const float MAX_DAC_VOLTAGE = 10.0;

// ================================================================
// ZMIENNE GLOBALNE
// ================================================================
float minLimit = 10.0;
float maxLimit = 40.0;
float dac2Ratio = 90.0; 

bool systemON = false;
bool modeAUTO = true;
bool trippedByOverload = false;

float napiecieZadajnika = 0.0;
float currentDac1 = 0.0;
float currentDac2 = 0.0;

unsigned long lastUpdate = 0;
unsigned long lastFastUpdate = 0;
unsigned long lastPIDTime = 0;
unsigned long resetPressTime = 0;
bool isResetPressed = false;

const float WSPOLCZYNNIK_DZIELNIKA = 1.982;
float filtr_waga = 0.15;

const int BUTTON_PIN = 0;
bool trybTestowy = false;
float current_Amps = 0.0;

// Zmienne do obsługi WiFi
bool isWifiAPActive = false;

// ZMIENNE PRZYCISKU
unsigned long buttonPressTime = 0; 
unsigned long lastClickTime = 0;
int clickCount = 0;
bool buttonWasPressed = false;
const unsigned long CLICK_TIMEOUT = 800;    
const unsigned long LONG_PRESS_TIME = 3000; 

// Zmienne LED
unsigned long ledTimer = 0;
int ledState = LOW;
int blinkCount = 0;
int blinkMax = 0;
int blinkDuration = 100; 

// Zmienne Mocy i Srodowiska
float pzem_u = 0, pzem_p = 0, pzem_pf = 0, pzem_s = 0, pzem_q = 0;
float dht_t = 0, dht_h = 0;

// Deklaracje
void startRegulator();
void stopRegulator();
void updateSettingsScreen();

// ================================================================
// FUNKCJE POMOCNICZE
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
        f.println("=== START SYSTEMU - V12.4 ==="); 
        f.close(); 
    }
}

void applyOutputMode() {
    if (outMode == 0 || outMode == 1) {
        currentMinDac = 3.5;
    } else if (outMode == 2) {
        currentMinDac = 4.8;
    }
    myPID.SetOutputLimits(currentMinDac, MAX_DAC_VOLTAGE);
    memory.putInt("outMode", outMode);
    Serial.printf("[SYS] Konfiguracja: Tryb %d | Podloga PID (17.5Hz): %.2fV\n", outMode, currentMinDac);
}

// ================================================================
// STRONA WWW (HTML + CSS + JS) 
// ================================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pl">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Regulator PID - Centrum Kontroli</title>
  <style>
    :root { --bg: #121212; --card: #1e1e1e; --text: #fff; --accent: #00bcd4; --green: #4caf50; --red: #f44336; --orange: #ff9800; --purple: #9c27b0; }
    body { background-color: var(--bg); color: var(--text); font-family: 'Segoe UI', sans-serif; margin: 0; padding: 0; }
    .header { background: #000; padding: 15px; text-align: center; border-bottom: 2px solid var(--accent); }
    h1 { margin: 0; font-size: 22px; color: var(--accent); }
    .nav { display: flex; justify-content: space-around; background: #222; padding: 10px 0; overflow-x: auto;}
    .nav button { background: none; border: none; color: #aaa; font-size: 14px; font-weight: bold; cursor: pointer; padding: 10px; white-space: nowrap; }
    .nav button.active { color: var(--accent); border-bottom: 2px solid var(--accent); }
    .tab-content { display: none; padding: 15px; max-width: 500px; margin: 0 auto; }
    .tab-content.active { display: block; }
    .card { background: var(--card); border-radius: 12px; padding: 15px; margin-bottom: 15px; box-shadow: 0 4px 8px rgba(0,0,0,0.5); }
    .row { display: flex; justify-content: space-between; font-size: 16px; padding: 10px 0; border-bottom: 1px solid #333; }
    .row:last-child { border: none; }
    .val { font-weight: bold; color: var(--green); }
    .ctrl-btn { width: 48%; padding: 15px; font-size: 16px; font-weight: bold; border-radius: 8px; border: none; cursor: pointer; color: #fff; transition: 0.2s;}
    .btn-on { background: var(--green); }
    .btn-off { background: var(--red); }
    .btn-auto { background: var(--accent); color: #000; }
    .btn-man { background: #555; }
    label { display: block; margin-top: 10px; font-size: 14px; color: #aaa; }
    input, select { width: 100%; padding: 10px; margin-top: 5px; background: #2a2a2a; border: 1px solid #444; color: #fff; border-radius: 6px; box-sizing: border-box; }
    .submit-btn { width: 100%; padding: 15px; margin-top: 15px; background: var(--accent); color: #000; border: none; font-weight: bold; border-radius: 8px; cursor: pointer; }
    .file-item { display: flex; justify-content: space-between; background: #2a2a2a; padding: 10px; margin-bottom: 5px; border-radius: 6px; }
    .file-item a { color: var(--accent); text-decoration: none; font-weight: bold; }
    #prog-container { width: 100%; background: #333; border-radius: 5px; display: none; margin-top: 15px;}
    #prog-bar { width: 0%; height: 20px; background: var(--green); border-radius: 5px; text-align: center; color: white; line-height: 20px; font-size: 12px;}
  </style>
</head>
<body>
  <div class="header"><h1>⚙️ Granulator Pro V12.4</h1></div>
  
  <div class="nav">
    <button class="tablinks active" onclick="openTab(event, 'Panel')">📊 Panel</button>
    <button class="tablinks" onclick="openTab(event, 'Sensory')">📡 Sensory</button>
    <button class="tablinks" onclick="openTab(event, 'Nastawy')">⚙️ Nastawy</button>
    <button class="tablinks" onclick="openTab(event, 'SD')">💾 SD Logi</button>
    <button class="tablinks" onclick="openTab(event, 'OTA')">📥 OTA</button>
  </div>

  <div id="Panel" class="tab-content active">
    <div class="card" style="display: flex; justify-content: space-between;">
      <button id="btnSys" class="ctrl-btn btn-off" onclick="toggleSys()">Zasilanie: OFF</button>
      <button id="btnMode" class="ctrl-btn btn-man" onclick="toggleMode()">Tryb: MAN</button>
    </div>
    <div class="card">
      <div class="row"><span>Prąd Maszyny:</span> <span class="val" id="amp">-- A</span></div>
      <div class="row"><span>Cel PID (Limit):</span> <span class="val" id="setp">-- A</span></div>
      <div class="row"><span>Awaria (Przeciążenie):</span> <span class="val" id="trip" style="color:var(--red);">NIE</span></div>
      <div class="row"><span>Wyjście na Falownik 1:</span> <span class="val" id="dac" style="color:var(--orange);">-- V</span></div>
      <div class="row"><span>Wyjście na Falownik 2:</span> <span class="val" id="dac2v" style="color:var(--purple);">-- V</span></div>
    </div>
  </div>

  <div id="Sensory" class="tab-content">
    <div class="card">
      <h3 style="margin-top:0; color:var(--accent);">Odczyt PZEM</h3>
      <div class="row"><span>Napięcie Sieci:</span> <span class="val" id="volt">-- V</span></div>
      <div class="row"><span>Moc Czynna (P):</span> <span class="val" id="pow">-- W</span></div>
      <div class="row"><span>Moc Pozorna (S):</span> <span class="val" id="ap_pow">-- VA</span></div>
      <div class="row"><span>Moc Bierna (Q):</span> <span class="val" id="re_pow">-- Var</span></div>
      <div class="row"><span>Cosinus Fi (PF):</span> <span class="val" id="pf">--</span></div>
    </div>
    <div class="card">
      <h3 style="margin-top:0; color:var(--accent);">Warunki DHT</h3>
      <div class="row"><span>Temperatura:</span> <span class="val" id="temp">-- °C</span></div>
      <div class="row"><span>Wilgotność:</span> <span class="val" id="hum">-- %</span></div>
    </div>
  </div>

  <div id="Nastawy" class="tab-content">
    <div class="card">
      <h3 style="margin-top:0; color:var(--purple);">Konfiguracja Sygnału (Fizyczna)</h3>
      <form onsubmit="saveOutMode(event)">
        <label>Wybierz typ falowników na obiekcie:</label>
        <select id="outMode">
          <option value="0">Napięciowe 0 - 10V</option>
          <option value="1">Prądowe 0 - 20mA (Moduł zewnętrzny)</option>
          <option value="2">Prądowe 4 - 20mA (Moduł zewnętrzny)</option>
        </select>
        <button type="submit" class="submit-btn" style="background:var(--purple); color:#fff;">ZAPISZ PROFIL FALOWNIKA</button>
      </form>
    </div>
    <div class="card">
      <h3 style="margin-top:0; color:#4caf50;">Proporcja Falownika 2 (DAC 2)</h3>
      <form onsubmit="saveDacRatio(event)">
        <label>Ustal stosunek prędkości względem DAC 1 (0-100%)</label>
        <input type="number" step="1" min="0" max="100" id="dac2r" required>
        <button type="submit" class="submit-btn" style="background:#4caf50; color:#fff;">ZAPISZ PROPORCJĘ (%)</button>
      </form>
    </div>
    <div class="card">
      <h3 style="margin-top:0;">Widełki Pracy (Ampery)</h3>
      <form onsubmit="saveLimits(event)">
        <label>Limit Minimalny</label>
        <input type="number" step="0.1" id="minL" required>
        <label>Limit Maksymalny (Sufit)</label>
        <input type="number" step="0.1" id="maxL" required>
        <button type="submit" class="submit-btn">ZAPISZ WIDEŁKI</button>
      </form>
    </div>
    <div class="card">
      <h3 style="margin-top:0;">Strojenie Algorytmu PID</h3>
      <form onsubmit="savePID(event)">
        <label><b>P</b> - Reakcja natychmiastowa</label>
        <input type="number" step="0.01" id="kp" required>
        <label><b>I</b> - Wyrównywanie w czasie</label>
        <input type="number" step="0.01" id="ki" required>
        <label><b>D</b> - Przewidywanie (Amortyzator)</label>
        <input type="number" step="0.01" id="kd" required>
        <button type="submit" class="submit-btn" style="background:var(--orange);">ZAPISZ PID</button>
      </form>
    </div>
  </div>

  <div id="SD" class="tab-content">
    <div class="card">
      <h3 style="margin-top:0;">Pliki na karcie SD</h3>
      <button onclick="loadSD()" style="padding:10px; background:#444; color:#fff; border:none; border-radius:5px; margin-bottom:15px; width:100%;">🔄 Odśwież listę</button>
      <div id="sd-list">Brak danych... kliknij Odśwież.</div>
    </div>
  </div>

  <div id="OTA" class="tab-content">
    <div class="card">
      <h3 style="margin-top:0; color:var(--red);">Aktualizacja Systemu (OTA)</h3>
      <p style="font-size:14px; color:#aaa;">Wybierz plik .bin z najnowszą wersją oprogramowania.</p>
      <form method="POST" action="#" enctype="multipart/form-data" id="upload_form">
        <input type="file" name="update" id="file" accept=".bin" required style="padding: 10px 0;">
        <button type="submit" class="submit-btn" style="background:var(--red); color:#fff;">🚀 WGRAJ AKTUALIZACJĘ</button>
      </form>
      <div id="prog-container">
        <div id="prog-bar">0%</div>
      </div>
      <p id="ota-status" style="margin-top:10px; font-weight:bold;"></p>
    </div>
  </div>

  <script>
    function openTab(evt, tabName) {
      var i, tabcontent, tablinks;
      tabcontent = document.getElementsByClassName("tab-content");
      for (i = 0; i < tabcontent.length; i++) tabcontent[i].style.display = "none";
      tablinks = document.getElementsByClassName("tablinks");
      for (i = 0; i < tablinks.length; i++) tablinks[i].className = tablinks[i].className.replace(" active", "");
      document.getElementById(tabName).style.display = "block";
      evt.currentTarget.className += " active";
    }

    setInterval(function() {
      fetch('/api/data').then(res => res.json()).then(data => {
        document.getElementById('amp').innerText = data.amp + " A";
        document.getElementById('setp').innerText = data.setp + " A";
        document.getElementById('dac').innerText = data.dac + " V";
        document.getElementById('dac2v').innerText = data.dac2v + " V";
        document.getElementById('trip').innerText = data.trip == "1" ? "TAK" : "NIE";
        
        let btnSys = document.getElementById('btnSys');
        if(data.sysON == "1") { btnSys.className = "ctrl-btn btn-on"; btnSys.innerText = "Zasilanie: ON"; }
        else { btnSys.className = "ctrl-btn btn-off"; btnSys.innerText = "Zasilanie: OFF"; }
        
        let btnMode = document.getElementById('btnMode');
        if(data.autoM == "1") { btnMode.className = "ctrl-btn btn-auto"; btnMode.innerText = "Tryb: AUTO"; }
        else { btnMode.className = "ctrl-btn btn-man"; btnMode.innerText = "Tryb: MAN"; }

        document.getElementById('volt').innerText = data.volt + " V";
        document.getElementById('pow').innerText = data.pow + " W";
        document.getElementById('ap_pow').innerText = data.ap_pow + " VA";
        document.getElementById('re_pow').innerText = data.re_pow + " Var";
        document.getElementById('pf').innerText = data.pf;
        document.getElementById('temp').innerText = data.temp + " °C";
        document.getElementById('hum').innerText = data.hum + " %";

        if(document.getElementById('outMode').value == "") document.getElementById('outMode').value = data.outM;
        if(!document.getElementById('dac2r').value) document.getElementById('dac2r').value = data.dac2R;
        if(!document.getElementById('minL').value) document.getElementById('minL').value = data.minL;
        if(!document.getElementById('maxL').value) document.getElementById('maxL').value = data.maxL;
        if(!document.getElementById('kp').value) document.getElementById('kp').value = data.kp;
        if(!document.getElementById('ki').value) document.getElementById('ki').value = data.ki;
        if(!document.getElementById('kd').value) document.getElementById('kd').value = data.kd;
      });
    }, 1000);

    function toggleSys() { fetch('/api/toggle_sys', {method: 'POST'}); }
    function toggleMode() { fetch('/api/toggle_mode', {method: 'POST'}); }

    function saveOutMode(e) {
      e.preventDefault();
      let m = document.getElementById('outMode').value;
      fetch('/api/set_outmode?m='+m, {method: 'POST'}).then(() => alert("Konfiguracja sprzętowa zaktualizowana!"));
    }
    function saveDacRatio(e) {
      e.preventDefault();
      let r = document.getElementById('dac2r').value;
      fetch('/api/set_dac2_ratio?r='+r, {method: 'POST'}).then(() => alert("Proporcja zapisana!"));
    }
    function saveLimits(e) {
      e.preventDefault();
      let m1 = document.getElementById('minL').value;
      let m2 = document.getElementById('maxL').value;
      fetch('/api/set_limits?min='+m1+'&max='+m2, {method: 'POST'}).then(() => alert("Widełki zapisane!"));
    }
    function savePID(e) {
      e.preventDefault();
      let p = document.getElementById('kp').value;
      let i = document.getElementById('ki').value;
      let d = document.getElementById('kd').value;
      fetch('/api/set_pid?kp='+p+'&ki='+i+'&kd='+d, {method: 'POST'}).then(() => alert("PID zaktualizowany!"));
    }

    function loadSD() {
      document.getElementById('sd-list').innerHTML = "Ładowanie...";
      fetch('/api/sd_list').then(res => res.json()).then(data => {
        let html = "";
        data.forEach(f => {
          html += `<div class='file-item'><a href='/sd_read?f=${f.name}' target='_blank'>📄 ${f.name}</a> <span>${f.size} KB</span></div>`;
        });
        if(html === "") html = "Brak plików na karcie.";
        document.getElementById('sd-list').innerHTML = html;
      });
    }

    document.getElementById('upload_form').addEventListener('submit', function(e) {
      e.preventDefault();
      var file = document.getElementById('file').files[0];
      if(!file) return;
      var data = new FormData();
      data.append('update', file, file.name);
      document.getElementById('prog-container').style.display = 'block';
      document.getElementById('ota-status').innerText = "Wgrywanie...";
      var xhr = new XMLHttpRequest();
      xhr.open('POST', '/update', true);
      xhr.upload.addEventListener('progress', function(e) {
        if (e.lengthComputable) {
          var percent = Math.round((e.loaded / e.total) * 100);
          document.getElementById('prog-bar').style.width = percent + '%';
          document.getElementById('prog-bar').innerText = percent + '%';
        }
      });
      xhr.onload = function() {
        if(xhr.status === 200) {
          document.getElementById('ota-status').style.color = "var(--green)";
          document.getElementById('ota-status').innerText = "Sukces! Trwa restart maszyny...";
          setTimeout(() => location.reload(), 5000);
        } else {
          document.getElementById('ota-status').style.color = "var(--red)";
          document.getElementById('ota-status').innerText = "Błąd aktualizacji!";
        }
      };
      xhr.send(data);
    });
  </script>
</body>
</html>
)rawliteral";

// ================================================================
// FUNKCJE SERWERA WWW I API
// ================================================================
void handleRoot() { server.send(200, "text/html", INDEX_HTML); }

void handleApiData() {
    String json = "{";
    json += "\"amp\":\"" + String(current_Amps, 2) + "\",";
    json += "\"setp\":\"" + String(Setpoint, 2) + "\",";
    json += "\"dac\":\"" + String(Output, 2) + "\",";
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
    json += "\"dac2R\":\"" + String(dac2Ratio, 0) + "\"";
    json += "}";
    server.send(200, "application/json", json);
}

void handleToggleSys() {
    if(systemON) stopRegulator(); else startRegulator();
    server.send(200, "text/plain", "OK");
}
void handleToggleMode() {
    modeAUTO = !modeAUTO;
    myNex.writeStr("pracaautoman.txt", modeAUTO ? "AUT" : "MAN");
    server.send(200, "text/plain", "OK");
}
void handleSetOutMode() {
    if (server.hasArg("m")) {
        outMode = server.arg("m").toInt();
        applyOutputMode();
        triggerBlink(1, 1000); 
    }
    server.send(200, "text/plain", "OK");
}
void handleSetDac2Ratio() {
    if (server.hasArg("r")) {
        dac2Ratio = server.arg("r").toFloat();
        if (dac2Ratio < 0.0) dac2Ratio = 0.0;
        if (dac2Ratio > 100.0) dac2Ratio = 100.0;
        memory.putFloat("dac2Ratio", dac2Ratio);
        Serial.printf("[SYS] Nowa proporcja DAC2: %.0f%%\n", dac2Ratio);
        triggerBlink(1, 1000);
    }
    server.send(200, "text/plain", "OK");
}
void handleSetLimits() {
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
    if (server.hasArg("kp") && server.hasArg("ki") && server.hasArg("kd")) {
        Kp = server.arg("kp").toFloat();
        Ki = server.arg("ki").toFloat();
        Kd = server.arg("kd").toFloat();
        memory.putFloat("kp", Kp);
        memory.putFloat("ki", Ki);
        memory.putFloat("kd", Kd);
        myPID.SetTunings(Kp, Ki, Kd);
        Serial.printf("[WiFi] Zmieniono PID: P=%.3f, I=%.3f, D=%.3f\n", Kp, Ki, Kd);
        triggerBlink(1, 1000);
    }
    server.send(200, "text/plain", "OK");
}
void handleSDList() {
    if(SD.cardType() == CARD_NONE) { server.send(200, "application/json", "[]"); return; }
    File root = SD.open("/");
    String json = "[";
    File file = root.openNextFile();
    while(file){
        if (!file.isDirectory()) {
            if(json != "[") json += ",";
            json += "{\"name\":\"" + String(file.name()) + "\",\"size\":" + String(file.size() / 1024) + "}";
        }
        file = root.openNextFile();
    }
    json += "]";
    server.send(200, "application/json", json);
}
void handleSDRead() {
    if (!server.hasArg("f")) { server.send(400, "text/plain", "Brak pliku"); return; }
    String path = "/" + server.arg("f");
    File file = SD.open(path, FILE_READ);
    if (!file) { server.send(404, "text/plain", "Nie znaleziono"); return; }
    server.streamFile(file, "text/plain"); 
    file.close();
}

void toggleWiFi() {
    if (!isWifiAPActive) {
        IPAddress local_ip(192, 168, 5, 1);
        IPAddress gateway(192, 168, 5, 1);
        IPAddress subnet(255, 255, 255, 0);
        WiFi.softAPConfig(local_ip, gateway, subnet);
        WiFi.softAP("RegulatorPID", "regpid12"); 
        server.begin();
        isWifiAPActive = true;
        Serial.println("\n[WIFI] SIEC UTWORZONA! Adres: 192.168.5.1\n");
        triggerBlink(3, 100); 
    } else {
        server.stop();
        WiFi.softAPdisconnect(true);
        isWifiAPActive = false;
        Serial.println("\n[WIFI] Siec WYLACZONA.\n");
        triggerBlink(1, 500); 
    }
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
    Output = napiecieZadajnika;     
    
    // Zabezpieczenie startowe
    if (Output < currentMinDac) Output = currentMinDac;
    if (Output > MAX_DAC_VOLTAGE) Output = MAX_DAC_VOLTAGE;
    
    currentDac1 = Output;
    currentDac2 = currentDac1 * (dac2Ratio / 100.0);
    
    // Twardy Kaganiec
    if (currentDac1 < 0.0) currentDac1 = 0.0;
    if (currentDac1 > 10.0) currentDac1 = 10.0;
    if (currentDac2 < 0.0) currentDac2 = 0.0;
    if (currentDac2 > 10.0) currentDac2 = 10.0;

    uint16_t mv_dac1 = (uint16_t)(currentDac1 * 1000.0);
    uint16_t mv_dac2 = (uint16_t)(currentDac2 * 1000.0);
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
                        activeButtonID = cmpId; isButtonHeld = true; 
                        processButtonAction(activeButtonID); buttonHoldTimer = millis() + 400; 
                    } else if (event == 0x00) { 
                        activeButtonID = 0; isButtonHeld = false; 
                    }
                }
            }
        }
    }
    if (isButtonHeld && activeButtonID > 0 && millis() > buttonHoldTimer) {
        processButtonAction(activeButtonID); buttonHoldTimer = millis() + 100; 
    }
    if (isResetPressed && (millis() - resetPressTime > 5000)) {
        myNex.writeStr("logi0.txt", "RESTART!"); delay(500); ESP.restart(); 
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
    Serial.println("\n\n--- SYSTEM V12.4 (Hard Clamp + Twardy Kaganiec) ---");

    WiFi.onEvent(onStationConnected, ARDUINO_EVENT_WIFI_AP_STACONNECTED);

    memory.begin("regulator", false); 
    minLimit = memory.getFloat("minLim", 10.0); 
    maxLimit = memory.getFloat("maxLim", 40.0); 
    Kp = memory.getFloat("kp", 0.5);
    Ki = memory.getFloat("ki", 0.1);
    Kd = memory.getFloat("kd", 0.15);
    myPID.SetTunings(Kp, Ki, Kd); 
    
    outMode = memory.getInt("outMode", 0);
    
    // Zabezpieczenie odczytu proporcji z EEPROM
    dac2Ratio = memory.getFloat("dac2Ratio", 90.0);
    if (isnan(dac2Ratio) || dac2Ratio < 0.0) dac2Ratio = 0.0;
    if (dac2Ratio > 100.0) dac2Ratio = 100.0;
    
    applyOutputMode();

    server.on("/", HTTP_GET, handleRoot);
    server.on("/api/data", HTTP_GET, handleApiData);
    server.on("/api/toggle_sys", HTTP_POST, handleToggleSys);
    server.on("/api/toggle_mode", HTTP_POST, handleToggleMode);
    server.on("/api/set_limits", HTTP_POST, handleSetLimits);
    server.on("/api/set_outmode", HTTP_POST, handleSetOutMode);
    server.on("/api/set_dac2_ratio", HTTP_POST, handleSetDac2Ratio);
    server.on("/api/set_pid", HTTP_POST, handleSetPID);
    server.on("/api/sd_list", HTTP_GET, handleSDList);
    server.on("/sd_read", HTTP_GET, handleSDRead);

    server.on("/update", HTTP_POST, []() {
        server.sendHeader("Connection", "close");
        server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
        ESP.restart();
    }, []() {
        HTTPUpload& upload = server.upload();
        if (upload.status == UPLOAD_FILE_START) {
            Serial.printf("[OTA] Wgrywanie: %s\n", upload.filename.c_str());
            if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { Update.printError(Serial); }
        } else if (upload.status == UPLOAD_FILE_WRITE) {
            if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) { Update.printError(Serial); }
        } else if (upload.status == UPLOAD_FILE_END) {
            if (Update.end(true)) { Serial.printf("[OTA] Sukces! Wielkosc: %u B. Restart...\n", upload.totalSize); } 
            else { Update.printError(Serial); }
        }
    });

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
    myPID.SetSampleTime(200); 
    
    Serial.println("SYSTEM GOTOWY - Odpal Monitor Szeregowy!");
    triggerBlink(2, 500); 

    // ODPALENIE WIFI AUTOMATYCZNIE PO STARCIE
    toggleWiFi();
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
        
        if(isnan(aktualny_odczyt)) aktualny_odczyt = 0.0;
        if(aktualny_odczyt < 0.05) aktualny_odczyt = 0.0;
        if(aktualny_odczyt > 10.5) aktualny_odczyt = 10.5;

        if (napiecieZadajnika == 0.0 && aktualny_odczyt > 0.0) {
            napiecieZadajnika = aktualny_odczyt; 
        } else {
            napiecieZadajnika = (aktualny_odczyt * filtr_waga) + (napiecieZadajnika * (1.0 - filtr_waga));
        }

        if (!systemON) {
            currentDac1 = napiecieZadajnika;
            currentDac2 = currentDac1 * (dac2Ratio / 100.0); 
        } else {
            if (isnan(Output)) Output = currentMinDac; 
            currentDac1 = Output;             
            currentDac2 = currentDac1 * (dac2Ratio / 100.0); 
        }

        // --- TWARDY KAGANIEC (Ochrona przed wybuchem DAC) ---
        if (currentDac1 < 0.0) currentDac1 = 0.0;
        if (currentDac1 > 10.0) currentDac1 = 10.0;
        if (currentDac2 < 0.0) currentDac2 = 0.0;
        if (currentDac2 > 10.0) currentDac2 = 10.0;

        uint16_t mv_dac1 = (uint16_t)(currentDac1 * 1000.0);
        uint16_t mv_dac2 = (uint16_t)(currentDac2 * 1000.0);
        
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
        
        // Delikatny filtr uśredniający prąd z maszyny
        if (current_Amps == 0.0) {
            current_Amps = i;
        } else {
            current_Amps = (i * 0.4) + (current_Amps * 0.6); 
        }
        
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
            // Odtworzona inteligentna Strefa Martwa (Window Control)
            float margines = (maxLimit - minLimit) * 0.15; 
            if (current_Amps > (maxLimit - margines)) {
                Setpoint = maxLimit - margines;
            } else if (current_Amps < (minLimit + margines)) {
                Setpoint = minLimit + margines;
            } else {
                Setpoint = current_Amps; // Zamraża PID - brak błędu
            }

            Input = current_Amps;         
            myPID.Compute();   
        }
    }

    // --- WOLNA PĘTLA EKRANU I SERIAL LOGGERA (1000ms) ---
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
            myNex.writeStr("granampery.txt", buf); myNex.writeStr("natgr.txt", buf);
            
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
        sprintf(dacBuf, "%.2f V", currentDac1); myNex.writeStr("pod1.txt", dacBuf); myNex.writeStr("dac1.txt", dacBuf);
        sprintf(dacBuf, "%.2f V", currentDac2); myNex.writeStr("pod2.txt", dacBuf); myNex.writeStr("dac2.txt", dacBuf);

        if(SD.cardType() != CARD_NONE) { 
             float gb = SD.totalBytes() / (1024.0*1024.0*1024.0); 
             myNex.writeStr("sd.txt", String(gb, 1) + " GB"); 
        } else {
             myNex.writeStr("sd.txt", "NO SD"); 
        }

        Serial.printf("%lu;%d;%d;%d;%.3f;%.1f;%.0f;%.2f;%.1f;%.1f;%.2f;%.2f;%.2f;%.2f\n", 
            millis(), systemON, modeAUTO, trippedByOverload, current_Amps, pzem_u, pzem_p, napiecieZadajnika, minLimit, maxLimit, Setpoint, Output, currentDac1, currentDac2);
    }
}