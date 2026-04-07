// ================================================================
// SERWER DYSPOZYTORSKI (MULTI-HUB) - V2.1 PEŁNY KLON
// ================================================================
#include <Arduino.h>
#include <WiFi.h>              
#include <WiFiClientSecure.h>
#include <ESPmDNS.h>           
#include <WebServer.h>         
#include <Preferences.h>
#include <PubSubClient.h>

// --- OBIEKTY SIECIOWE ---
WebServer server(80); 
WiFiClientSecure espClient; 
PubSubClient mqtt(espClient);
Preferences memory;

#define PIN_LED 2 

// --- ZMIENNE KONFIGURACYJNE ---
String routerSSID = "";
String routerPASS = "";
String mqtt_server = "";
String mqtt_user = "";
String mqtt_pass = "";

unsigned long lastMqttReconnect = 0;

// --- STRUKTURA FLOTY MASZYN ---
#define MAX_MACHINES 10
struct Machine {
    String id;
    String json;
    unsigned long lastSeen;
};
Machine machines[MAX_MACHINES];

// ================================================================
// FUNKCJE POMOCNICZE
// ================================================================
String cleanHostAddress(String host) {
    String clean = host;
    clean.replace("http://", "");
    clean.replace("https://", "");
    int colonIndex = clean.indexOf(':');
    if (colonIndex > 0) clean = clean.substring(0, colonIndex); 
    clean.trim();
    return clean;
}

// ================================================================
// BRAMKA AUTORYZACJI
// ================================================================
bool checkAuth() {
    if (!server.authenticate("admin", "regpid12")) {
        server.requestAuthentication();
        return false;
    }
    return true;
}

// ================================================================
// INTERFEJS GRAFICZNY SERWERA (HTML + JS)
// ================================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pl">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>HUB Dowodzenia - Flota</title>
  <style>
    :root { --bg: #121212; --card: #1e1e1e; --text: #fff; --accent: #00bcd4; --green: #4caf50; --red: #f44336; --orange: #ff9800; --purple: #9c27b0; --yellow: #ffeb3b; --pink: #e91e63; }
    body { background-color: var(--bg); color: var(--text); font-family: 'Segoe UI', sans-serif; margin: 0; padding: 0; }
    .header { background: #000; padding: 15px; text-align: center; border-bottom: 2px solid var(--accent); position: relative;}
    h1 { margin: 0; font-size: 22px; color: var(--accent); }
    
    .nav { display: flex; justify-content: space-around; background: #222; padding: 10px 0; overflow-x: auto;}
    .nav button { background: none; border: none; color: #aaa; font-size: 14px; font-weight: bold; cursor: pointer; padding: 10px; white-space: nowrap; }
    .nav button.active { color: var(--accent); border-bottom: 2px solid var(--accent); }
    
    .container { padding: 15px; max-width: 600px; margin: 0 auto; }
    .tab-content { display: none; }
    .tab-content.active { display: block; }
    
    .card { background: var(--card); border-radius: 12px; padding: 15px; margin-bottom: 15px; box-shadow: 0 4px 8px rgba(0,0,0,0.5); }
    .row { display: flex; justify-content: space-between; font-size: 16px; padding: 10px 0; border-bottom: 1px solid #333; }
    .row:last-child { border: none; }
    .val { font-weight: bold; color: var(--green); }
    
    .ctrl-btn { width: 48%; padding: 15px; font-size: 16px; font-weight: bold; border-radius: 8px; border: none; cursor: pointer; color: #fff; text-align:center; transition: 0.2s;}
    .btn-on { background: var(--green); }
    .btn-off { background: var(--red); }
    .btn-auto { background: var(--accent); color: #000; }
    .btn-man { background: #555; }
    
    label { display: block; margin-top: 10px; font-size: 14px; color: #aaa; }
    input, select { width: 100%; padding: 10px; margin-top: 5px; background: #2a2a2a; border: 1px solid #444; color: #fff; border-radius: 6px; box-sizing: border-box; }
    .submit-btn { width: 100%; padding: 15px; margin-top: 15px; background: var(--accent); color: #000; border: none; font-weight: bold; border-radius: 8px; cursor: pointer; }
    
    .machine-btn { display: block; width: 100%; background: #222; border: 2px solid #444; padding: 20px; border-radius: 10px; margin-bottom: 15px; cursor: pointer; text-align: left; transition: 0.3s;}
    .machine-btn:hover { border-color: var(--accent); background: #2a2a2a;}
    .m-title { font-size: 20px; font-weight: bold; color: var(--accent); margin-bottom: 5px;}
    .m-stat { font-size: 14px; color: #888; display:flex; justify-content: space-between;}
    .dot { height: 12px; width: 12px; background-color: var(--red); border-radius: 50%; display: inline-block;}
    .dot.ok { background-color: var(--green); box-shadow: 0 0 8px var(--green);}
    .badge-ok { color: var(--green); font-weight: bold; text-shadow: 0 0 5px rgba(76, 175, 80, 0.5); }
    .badge-err { color: var(--red); font-weight: bold; text-shadow: 0 0 5px rgba(244, 67, 54, 0.5); }
    
    #btn-back { position: absolute; left: 15px; top: 15px; background: none; border: 1px solid var(--accent); color: var(--accent); padding: 5px 15px; border-radius: 5px; cursor: pointer; display: none;}
  </style>
</head>
<body>
  <div class="header">
    <button id="btn-back" onclick="showDashboard()">🔙 Wróć do Floty</button>
    <h1>📡 HUB DOWODZENIA</h1>
    <div style="margin-top:5px; font-size:12px; color:#888;">IP Serwera HUB: <span id="srvIP">--</span> | <span id="mqStat">Szukam chmury...</span></div>
  </div>

  <div id="view-dashboard" class="container" style="display:block;">
    <h3 style="color:#aaa; border-bottom:1px solid #333; padding-bottom:10px;">Dostępne Maszyny (Live):</h3>
    <div id="machine-list">Ładowanie sygnału z chmury...</div>
    
    <div class="card" style="margin-top: 40px; border: 1px solid #555;">
      <h3 style="margin-top:0; color:var(--yellow);">⚙️ Konfiguracja samego Serwera HUB</h3>
      <p style="font-size:12px; color:#aaa;">Ustawienia łącza internetowego dla tego ekranu na biurku.</p>
      <form onsubmit="saveConfig(event)">
        <label>WiFi SSID (Router domowy/biurowy)</label>
        <input type="text" id="c_ssid">
        <label>WiFi Hasło</label>
        <input type="password" id="c_pass" placeholder="[Zapisane - pozostaw puste aby nie zmieniac]">
        <label>MQTT Adres Brokera (Cluster URL)</label>
        <input type="text" id="c_msrv">
        <label>MQTT Użytkownik</label>
        <input type="text" id="c_musr">
        <label>MQTT Hasło</label>
        <input type="password" id="c_mpas" placeholder="[Zapisane - pozostaw puste aby nie zmieniac]">
        <button type="submit" class="submit-btn" style="background:var(--yellow); color:#000;">ZAPISZ I RESTARTUJ HUB</button>
      </form>
    </div>
  </div>

  <div id="view-machine" style="display:none;">
    <div class="nav">
      <button class="tablinks active" onclick="openTab(event, 'Panel')">📊 Panel</button>
      <button class="tablinks" onclick="openTab(event, 'Sensory')">📡 Sensory</button>
      <button class="tablinks" onclick="openTab(event, 'Nastawy')">⚙️ Nastawy</button>
      <button class="tablinks" onclick="openTab(event, 'SD')">🩺 Diagnostyka</button>
      <button class="tablinks" onclick="openTab(event, 'OTA')">📥 OTA</button>
    </div>

    <div class="container">
      <h2 style="text-align:center; color:var(--text); margin-top:0; border-bottom:1px dashed #444; padding-bottom:10px;">Maszyna: <span id="active-m-id" style="color:var(--accent);">---</span></h2>
      
      <div id="Panel" class="tab-content active">
        <div class="card" style="display: flex; justify-content: space-between;">
          <div id="btnSys" class="ctrl-btn btn-off" onclick="toggleSys()">Zasilanie: OFF</div>
          <div id="btnMode" class="ctrl-btn btn-man" onclick="toggleMode()">Tryb: MAN</div>
        </div>
        <div class="card">
          <div class="row"><span>Prąd Maszyny:</span> <span class="val" id="amp">-- A</span></div>
          <div class="row"><span>Cel PID (Limit):</span> <span class="val" id="setp">-- A</span></div>
          <div class="row"><span>Awaria (Przeciążenie):</span> <span class="val" id="trip" style="color:var(--red);">NIE</span></div>
          <div class="row"><span>Wyjście na Falownik 1:</span> <span class="val" id="dac" style="color:var(--orange);">-- V</span></div>
          <div class="row"><span>Wyjście na Falownik 2:</span> <span class="val" id="dac2v" style="color:var(--purple);">-- V</span></div>
        </div>
        <button class="submit-btn" style="background:var(--red); color:#fff;" onclick="if(confirm('Zrestartować maszynę? Zostanie zatrzymana!')) sendCmd('RESTART=1')">🔄 ZDALNY RESTART MASZYNY (ESP32)</button>
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
        <p style="color:var(--orange); font-size:12px; text-align:center;">Wprowadzane tu zmiany zostaną bezzwłocznie przesłane przez chmurę MQTT do maszyny.</p>
        
        <div class="card">
          <h3 style="margin-top:0;">1. Widełki Pracy (Ampery)</h3>
          <form onsubmit="cmdLimits(event)">
            <label>Limit Minimalny</label><input type="number" step="0.1" id="minL" required>
            <label>Limit Maksymalny (Sufit)</label><input type="number" step="0.1" id="maxL" required>
            <button type="submit" class="submit-btn">WYŚLIJ WIDEŁKI DO MASZYNY</button>
          </form>
        </div>
        
        <div class="card">
          <h3 style="margin-top:0; color:var(--orange);">2. Proporcje Falowników (0-100%)</h3>
          <form onsubmit="cmdRatios(event)">
            <label>DAC 1 (Główny)</label><input type="number" step="1" min="0" max="100" id="dac1r" required>
            <label>DAC 2 (Pomocniczy)</label><input type="number" step="1" min="0" max="100" id="dac2r" required>
            <button type="submit" class="submit-btn" style="background:var(--orange); color:#fff;">WYŚLIJ PROPORCJE DO MASZYNY</button>
          </form>
        </div>

        <div class="card">
          <h3 style="margin-top:0;">3. Strojenie Algorytmu PID</h3>
          <form onsubmit="cmdPID(event)">
            <label>P - Reakcja natychmiastowa</label><input type="number" step="0.01" id="kp" required>
            <label>I - Wyrównywanie w czasie</label><input type="number" step="0.01" id="ki" required>
            <label>D - Przewidywanie</label><input type="number" step="0.01" id="kd" required>
            <button type="submit" class="submit-btn" style="background:#555; color:#fff;">WYŚLIJ PID DO MASZYNY</button>
          </form>
        </div>

        <div class="card">
          <h3 style="margin-top:0; color:var(--green);">4. Ustawienia Bezpieczeństwa (Odcięcia)</h3>
          <form onsubmit="cmdAlarms(event)">
            <label>Próg Odcięcia Awaryjnego (+ Ampery ponad Max Limit)</label><input type="number" step="0.1" id="ovL" required>
            <label>Próg Wznowienia Pracy (+ Ampery powyżej Min Limit)</label><input type="number" step="0.1" id="recL" required>
            <button type="submit" class="submit-btn" style="background:var(--green); color:#fff;">WYŚLIJ ALARMY DO MASZYNY</button>
          </form>
        </div>

        <div class="card">
          <h3 style="margin-top:0; color:var(--pink);">5. Ochrona Falownika (Limity Napięcia DAC)</h3>
          <form onsubmit="cmdVoltLimits(event)">
            <label>Dolna podłoga napięcia [V]</label><input type="number" step="0.01" id="minV" required>
            <label>Górny sufit napięcia [V]</label><input type="number" step="0.01" id="maxV" required>
            <button type="submit" class="submit-btn" style="background:var(--pink); color:#fff;">WYŚLIJ LIMITY NAPIĘCIA</button>
          </form>
        </div>

        <div class="card">
          <h3 style="margin-top:0; color:var(--purple);">6. Konfiguracja Sygnału (Informacyjna)</h3>
          <form onsubmit="cmdOutMode(event)">
            <label>Wybierz typ falowników na obiekcie:</label>
            <select id="outMode">
              <option value="0">Standard Napięciowy 0-10V</option>
              <option value="1">Izolator Optyczny GLK (0 - 20mA)</option>
              <option value="2">Izolator Optyczny GLK (4 - 20mA)</option>
            </select>
            <button type="submit" class="submit-btn" style="background:var(--purple); color:#fff;">WYŚLIJ PROFIL FALOWNIKA</button>
          </form>
        </div>

        <div class="card">
          <h3 style="margin-top:0; color:#00bcd4;">7. Kalibracja Napięcia DAC (Offset)</h3>
          <form onsubmit="cmdCalib(event)">
            <label>Korekta DAC 1 [Volty]</label><input type="number" step="0.01" id="dac1c" required>
            <label>Korekta DAC 2 [Volty]</label><input type="number" step="0.01" id="dac2c" required>
            <button type="submit" class="submit-btn" style="background:#00bcd4; color:#000;">WYŚLIJ KALIBRACJĘ</button>
          </form>
        </div>

        <div class="card">
          <h3 style="margin-top:0; color:#4caf50;">8. Zdalna Zmiana WiFi Maszyny</h3>
          <form onsubmit="cmdWiFi(event)">
            <label>Nowa nazwa sieci WiFi (SSID)</label><input type="text" id="m_wifiSSID">
            <label>Nowe hasło do WiFi</label><input type="password" id="m_wifiPASS" placeholder="[Pozostaw puste aby nie zmieniac]">
            <button type="submit" class="submit-btn" style="background:#4caf50; color:#fff;">ZAPISZ I RESTARTUJ MASZYNĘ</button>
          </form>
        </div>

        <div class="card" style="border: 2px solid #03a9f4;">
          <h3 style="margin-top:0; color:#03a9f4;">9. Zdalna Zmiana Chmury Maszyny</h3>
          <form onsubmit="cmdMQTT(event)">
            <label>Adres Brokera (Cluster URL)</label><input type="text" id="m_mqSrv">
            <label>Użytkownik MQTT</label><input type="text" id="m_mqUsr">
            <label>Hasło MQTT</label><input type="password" id="m_mqPas" placeholder="[Pozostaw puste aby nie zmieniac]">
            <label>Unikalne ID Maszyny</label><input type="text" id="m_mqId">
            <button type="submit" class="submit-btn" style="background:#03a9f4; color:#fff;">ZAPISZ MQTT MASZYNY</button>
          </form>
        </div>

        <div class="card" style="border: 2px solid var(--yellow);">
          <h3 style="margin-top:0; color:var(--yellow);">10. Ustawienia Domyślne Maszyny</h3>
          <button onclick="if(confirm('Zapisać na wybranej maszynie jako domyślne?')) sendCmd('CMD:SAVEDEF')" class="submit-btn" style="background:var(--yellow); color:#000;">ZAPISZ OBECNE JAKO DOMYŚLNE</button>
          <button onclick="if(confirm('Przywrócić fabryczne na maszynie?')) sendCmd('CMD:RESTOREDEF')" class="submit-btn" style="background:var(--red); color:#fff; margin-top:10px;">PRZYWRÓĆ USTAWIENIA DOMYŚLNE</button>
        </div>
      </div>

      <div id="SD" class="tab-content">
        <div class="card">
          <h3 style="margin-top:0; color:var(--yellow);">🧠 Parametry Systemu ESP32 Maszyny</h3>
          <div class="row"><span>Czas pracy (Uptime):</span> <span class="val" id="esp_up" style="color:var(--text);">--</span></div>
          <div class="row"><span>Wolna Pamięć RAM:</span> <span class="val" id="esp_ram" style="color:var(--text);">-- %</span></div>
          <div class="row"><span>Zajętość Pamięci (Flash):</span> <span class="val" id="esp_flash" style="color:var(--text);">-- KB</span></div>
          <div class="row"><span>Adres IP (LAN / Router):</span> <span class="val" id="esp_rip" style="color:var(--text);">--</span></div>
        </div>
        <div class="card">
          <h3 style="margin-top:0; color:#00bcd4;">🩺 Status Sprzętu (Na Żywo)</h3>
          <div class="row"><span>Zasilanie (PZEM-004T):</span> <span id="st_pzem" class="badge-err">CZEKAM NA DANE...</span></div>
          <div class="row"><span>Ekran HMI (Nextion):</span> <span id="st_nex" class="badge-err">CZEKAM NA DANE...</span></div>
          <div class="row"><span>Klimat (DHT11):</span> <span id="st_dht" class="badge-err">CZEKAM NA DANE...</span></div>
          <div class="row"><span>Logi (Karta SD):</span> <span id="st_sd" class="badge-err">CZEKAM NA DANE...</span></div>
        </div>
        <p style="font-size:12px; color:#aaa; text-align:center;">Pełna diagnostyka będzie napełniana danymi po zaktualizowaniu Firmware'u na wybranej maszynie do V16.</p>
      </div>

      <div id="OTA" class="tab-content">
        <div class="card" style="border: 2px solid #9c27b0;">
          <h3 style="margin-top:0; color:#9c27b0;">🚀 Zdalna Aktualizacja Firmware (Chmura)</h3>
          <p style="font-size:13px; color:#aaa;">Wklej surowy link URL do pliku .bin (np. z GitHub), aby zdalnie wgrać nowy kod do maszyny.</p>
          <input type="text" id="otaUrl" placeholder="https://raw.githubusercontent.com/.../update.bin">
          <button class="submit-btn" style="background:#9c27b0; color:#fff;" onclick="cmdOTA()">ROZPOCZNIJ ZDALNE FLASHOWANIE</button>
        </div>
      </div>

    </div>
  </div>

  <script>
    let activeMachine = "";
    let lastFocusTime = 0;
    
    // Zmienne śledzące stan maszyny dla poprawnych przycisków TOGGLE
    let currentSysON = 0;
    let currentAutoM = 1;

    window.addEventListener('DOMContentLoaded', () => {
        document.querySelectorAll('input, select').forEach(i => {
            i.addEventListener('focus', () => { lastFocusTime = Date.now(); });
            i.addEventListener('input', () => { lastFocusTime = Date.now(); });
            i.addEventListener('blur', () => { lastFocusTime = Date.now(); });
        });
    });

    function openTab(evt, tabName) {
      document.querySelectorAll(".tab-content").forEach(el => el.style.display = "none");
      document.querySelectorAll(".tablinks").forEach(el => el.classList.remove("active"));
      document.getElementById(tabName).style.display = "block";
      evt.currentTarget.classList.add("active");
    }

    function showDashboard() {
      activeMachine = "";
      document.getElementById("view-machine").style.display = "none";
      document.getElementById("view-dashboard").style.display = "block";
      document.getElementById("btn-back").style.display = "none";
    }

    function selectMachine(id) {
      activeMachine = id;
      document.getElementById("active-m-id").innerText = id;
      document.getElementById("view-dashboard").style.display = "none";
      document.getElementById("view-machine").style.display = "block";
      document.getElementById("btn-back").style.display = "inline-block";
      
      document.querySelectorAll(".tab-content").forEach(el => el.style.display = "none");
      document.querySelectorAll(".tablinks").forEach(el => el.classList.remove("active"));
      document.getElementById("Panel").style.display = "block";
      document.querySelectorAll(".tablinks")[0].classList.add("active");
    }

    // --- FUNKCJE WYSYŁAJĄCE ROZKAZY DO CHMURY ---
    function sendCmd(cmdStr) {
      if(!activeMachine) return;
      fetch('/api/send_cmd?id=' + activeMachine, {
        method: 'POST', headers: {'Content-Type': 'application/x-www-form-urlencoded'},
        body: 'cmd=' + encodeURIComponent(cmdStr)
      }).then(() => alert("Wysłano komendę do maszyny!"));
    }

    // Mądre przełączanie - Serwer wie jaki jest stan i wysyła komendę odwrotną
    function toggleSys() { sendCmd(currentSysON ? 'SYSTEM=OFF' : 'SYSTEM=ON'); }
    function toggleMode() { sendCmd(currentAutoM ? 'MODE=MAN' : 'MODE=AUTO'); }

    function cmdLimits(e) { e.preventDefault(); sendCmd(`CMD:LIMITS:${document.getElementById('minL').value}:${document.getElementById('maxL').value}`); }
    function cmdRatios(e) { e.preventDefault(); sendCmd(`CMD:RATIOS:${document.getElementById('dac1r').value}:${document.getElementById('dac2r').value}`); }
    function cmdPID(e) { e.preventDefault(); sendCmd(`CMD:PID:${document.getElementById('kp').value}:${document.getElementById('ki').value}:${document.getElementById('kd').value}`); }
    function cmdAlarms(e) { e.preventDefault(); sendCmd(`CMD:ALARMS:${document.getElementById('ovL').value}:${document.getElementById('recL').value}`); }
    function cmdVoltLimits(e) { e.preventDefault(); sendCmd(`CMD:VOLT:${document.getElementById('minV').value}:${document.getElementById('maxV').value}`); }
    function cmdOutMode(e) { e.preventDefault(); sendCmd(`CMD:OUTMODE:${document.getElementById('outMode').value}`); }
    function cmdCalib(e) { e.preventDefault(); sendCmd(`CMD:CALIB:${document.getElementById('dac1c').value}:${document.getElementById('dac2c').value}`); }
    
    function cmdWiFi(e) { 
        e.preventDefault(); 
        let pass = document.getElementById('m_wifiPASS').value;
        sendCmd(`CMD:WIFI:${document.getElementById('m_wifiSSID').value}:${pass}`); 
    }
    function cmdMQTT(e) { 
        e.preventDefault(); 
        let pass = document.getElementById('m_mqPas').value;
        sendCmd(`CMD:MQTT:${document.getElementById('m_mqSrv').value}:${document.getElementById('m_mqUsr').value}:${pass}:${document.getElementById('m_mqId').value}`); 
    }

    function cmdOTA() {
      let url = document.getElementById('otaUrl').value;
      if(!url.startsWith("http")) return alert("Błąd! Podaj link HTTP/HTTPS.");
      if(confirm(`UWAGA! Zlecasz zdalne nadpisanie pamięci maszyny ${activeMachine}. Kontynuować?`)) {
         sendCmd("OTA=" + url);
      }
    }

    function saveConfig(e) {
      e.preventDefault();
      let p = `ssid=${encodeURIComponent(document.getElementById('c_ssid').value)}&pass=${encodeURIComponent(document.getElementById('c_pass').value)}&msrv=${encodeURIComponent(document.getElementById('c_msrv').value)}&musr=${encodeURIComponent(document.getElementById('c_musr').value)}&mpas=${encodeURIComponent(document.getElementById('c_mpas').value)}`;
      fetch('/api/save_config', { method: 'POST', headers: {'Content-Type': 'application/x-www-form-urlencoded'}, body: p })
      .then(() => { alert("Zapisano konfigurację Serwera! Restart..."); setTimeout(()=>location.reload(), 5000); });
    }

    // --- PĘTLA POBIERAJĄCA DANE Z SERWERA (CO 1 SEKUNDĘ) ---
    setInterval(() => {
      fetch('/api/machines').then(r => r.json()).then(data => {
        if(activeMachine === "") {
          let html = "";
          data.forEach(m => {
            html += `<div class='machine-btn' onclick='selectMachine("${m.id}")'>
                       <div class='m-title'>⚙️ ${m.id}</div>
                       <div class='m-stat'><span>Ostatni sygnał: ${m.age}s temu</span> <span><span class='dot ok'></span> ONLINE</span></div>
                     </div>`;
          });
          if(data.length === 0) html = "<div style='text-align:center; padding:20px; color:#666;'>Brak maszyn online w HiveMQ...</div>";
          document.getElementById('machine-list').innerHTML = html;
        }
      });

      if(activeMachine !== "") {
        fetch('/api/machine_data?id=' + activeMachine).then(r => r.json()).then(d => {
          
          if(d.sysON !== undefined) currentSysON = d.sysON;
          if(d.autoM !== undefined) currentAutoM = d.autoM;

          document.getElementById('amp').innerText = (d.amp!==undefined?d.amp:"--") + " A";
          document.getElementById('setp').innerText = (d.setp!==undefined?d.setp:"--") + " A";
          document.getElementById('volt').innerText = (d.volt!==undefined?d.volt:"--") + " V";
          
          let btnSys = document.getElementById('btnSys');
          if(currentSysON === 1) { btnSys.className = "ctrl-btn btn-on"; btnSys.innerText = "Zasilanie: ON"; }
          else { btnSys.className = "ctrl-btn btn-off"; btnSys.innerText = "Zasilanie: OFF"; }

          let btnMode = document.getElementById('btnMode');
          if(currentAutoM === 1) { btnMode.className = "ctrl-btn btn-auto"; btnMode.innerText = "Tryb: AUTO"; }
          else { btnMode.className = "ctrl-btn btn-man"; btnMode.innerText = "Tryb: MAN"; }

          let trip = document.getElementById('trip');
          if(d.trip === 1) { trip.innerText = "TAK (Odcięta!)"; trip.style.color = "var(--red)"; }
          else { trip.innerText = "NIE"; trip.style.color = "var(--green)"; }

          if(d.dac !== undefined) document.getElementById('dac').innerText = d.dac + " V";
          if(d.dac2v !== undefined) document.getElementById('dac2v').innerText = d.dac2v + " V";
          if(d.pow !== undefined) document.getElementById('pow').innerText = d.pow + " W";
          if(d.ap_pow !== undefined) document.getElementById('ap_pow').innerText = d.ap_pow + " VA";
          if(d.re_pow !== undefined) document.getElementById('re_pow').innerText = d.re_pow + " Var";
          if(d.pf !== undefined) document.getElementById('pf').innerText = d.pf;
          if(d.temp !== undefined) document.getElementById('temp').innerText = d.temp + " °C";
          if(d.hum !== undefined) document.getElementById('hum').innerText = d.hum + " %";

          // Aktualizacja diagnostyki SD/Sprzetu (Jesli maszyna wysyla te dane)
          if(d.up !== undefined) document.getElementById('esp_up').innerText = d.up;
          if(d.heap_pct !== undefined) document.getElementById('esp_ram').innerText = d.heap_pct + " %";
          if(d.sketch !== undefined) document.getElementById('esp_flash').innerText = d.sketch;
          if(d.router_ip !== undefined) document.getElementById('esp_rip').innerText = d.router_ip;

          function setSt(id, st) {
              let el = document.getElementById(id);
              if (st == 1) { el.innerText = "ONLINE"; el.className = "badge-ok"; } 
              else { el.innerText = "BŁĄD / OFFLINE"; el.className = "badge-err"; }
          }
          if(d.pzem !== undefined) setSt('st_pzem', d.pzem);
          if(d.nex !== undefined) setSt('st_nex', d.nex);
          if(d.dht !== undefined) setSt('st_dht', d.dht);
          if(d.sd !== undefined) setSt('st_sd', d.sd);

          if (Date.now() - lastFocusTime > 10000) {
              if(d.outM !== undefined) document.getElementById('outMode').value = d.outM;
              if(d.minL !== undefined) document.getElementById('minL').value = d.minL;
              if(d.maxL !== undefined) document.getElementById('maxL').value = d.maxL;
              if(d.kp !== undefined) document.getElementById('kp').value = d.kp;
              if(d.ki !== undefined) document.getElementById('ki').value = d.ki;
              if(d.kd !== undefined) document.getElementById('kd').value = d.kd;
              if(d.dac1R !== undefined) document.getElementById('dac1r').value = d.dac1R;
              if(d.dac2R !== undefined) document.getElementById('dac2r').value = d.dac2R;
              if(d.ovL !== undefined) document.getElementById('ovL').value = d.ovL;
              if(d.recL !== undefined) document.getElementById('recL').value = d.recL;
              if(d.dac1C !== undefined) document.getElementById('dac1c').value = d.dac1C;
              if(d.dac2C !== undefined) document.getElementById('dac2c').value = d.dac2C;
              if(d.minV !== undefined) document.getElementById('minV').value = d.minV;
              if(d.maxV !== undefined) document.getElementById('maxV').value = d.maxV;
              if(d.wifi_s !== undefined) document.getElementById('m_wifiSSID').value = d.wifi_s;
              if(d.mq_srv !== undefined) document.getElementById('m_mqSrv').value = d.mq_srv;
              if(d.mq_usr !== undefined) document.getElementById('m_mqUsr').value = d.mq_usr;
              if(d.mq_id !== undefined) document.getElementById('m_mqId').value = d.mq_id;
          }
        }).catch(e => console.log("Czekam na JSON"));
      }

      fetch('/api/server_status').then(r => r.json()).then(data => {
        document.getElementById('srvIP').innerText = data.ip;
        if(data.ip === "0.0.0.0" || data.ip === "192.168.10.1") {
            document.getElementById('mqStat').innerText = "Brak WAN (Router)";
        } else if(data.mqtt_connected === false) {
            document.getElementById('mqStat').innerText = "Szukam HiveMQ...";
        } else {
            document.getElementById('mqStat').innerText = "Chmura ONLINE";
            document.getElementById('mqStat').style.color = "var(--green)";
        }

        if(document.activeElement.tagName !== "INPUT") {
            document.getElementById('c_ssid').value = data.ssid;
            document.getElementById('c_msrv').value = data.mqtt_srv;
            document.getElementById('c_musr').value = data.mqtt_usr;
        }
      });
    }, 1000);

  </script>
</body>
</html>
)rawliteral";

// ================================================================
// FUNKCJE API SERWERA
// ================================================================
void handleRoot() {
    if(!checkAuth()) return;
    server.send(200, "text/html", INDEX_HTML);
}

void handleGetMachines() {
    if(!checkAuth()) return;
    String json = "[";
    bool first = true;
    unsigned long now = millis();
    for(int i=0; i<MAX_MACHINES; i++) {
        if(machines[i].id != "") {
            if((now - machines[i].lastSeen) < 30000) {
                if(!first) json += ",";
                json += "{\"id\":\"" + machines[i].id + "\", \"age\":" + String((now - machines[i].lastSeen)/1000) + "}";
                first = false;
            }
        }
    }
    json += "]";
    server.send(200, "application/json", json);
}

void handleMachineData() {
    if(!checkAuth()) return;
    if(server.hasArg("id")) {
        String reqId = server.arg("id");
        for(int i=0; i<MAX_MACHINES; i++) {
            if(machines[i].id == reqId) {
                server.send(200, "application/json", machines[i].json);
                return;
            }
        }
    }
    server.send(404, "application/json", "{}");
}

void handleServerStatus() {
    if(!checkAuth()) return;
    String json = "{";
    json += "\"mqtt_connected\":" + String(mqtt.connected() ? "true" : "false") + ",";
    String currentIP = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "192.168.10.1";
    json += "\"ip\":\"" + currentIP + "\",";
    json += "\"ssid\":\"" + routerSSID + "\",";
    json += "\"mqtt_srv\":\"" + mqtt_server + "\",";
    json += "\"mqtt_usr\":\"" + mqtt_user + "\"";
    json += "}";
    server.send(200, "application/json", json);
}

void handleSendCommand() {
    if(!checkAuth()) return;
    if (server.hasArg("cmd") && server.hasArg("id") && mqtt.connected()) {
        String cmd = server.arg("cmd");
        String target = server.arg("id");
        String topic = "biuro/" + target + "/rozkazy";
        mqtt.publish(topic.c_str(), cmd.c_str());
        Serial.println("[MQTT] Rozkaz: [" + cmd + "] wyslano na kanal: " + topic);
        server.send(200, "text/plain", "OK");
    } else {
        server.send(500, "text/plain", "Blad chmury lub brak ID maszyny");
    }
}

void handleSaveConfig() {
    if(!checkAuth()) return;
    if(server.hasArg("ssid")) routerSSID = server.arg("ssid");
    if(server.hasArg("pass") && server.arg("pass") != "") routerPASS = server.arg("pass");
    if(server.hasArg("msrv")) mqtt_server = cleanHostAddress(server.arg("msrv")); 
    if(server.hasArg("musr")) mqtt_user = server.arg("musr");
    if(server.hasArg("mpas") && server.arg("mpas") != "") mqtt_pass = server.arg("mpas");

    memory.putString("ssid", routerSSID);
    memory.putString("pass", routerPASS);
    memory.putString("msrv", mqtt_server);
    memory.putString("musr", mqtt_user);
    memory.putString("mpas", mqtt_pass);

    server.send(200, "text/plain", "OK");
    delay(1000);
    ESP.restart();
}

// ================================================================
// OBSŁUGA MQTT (NASŁUCH WILDCARD)
// ================================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String msg = "";
    for (int i = 0; i < length; i++) msg += (char)payload[i];
    
    String t = String(topic);
    int firstSlash = t.indexOf('/');
    int secondSlash = t.indexOf('/', firstSlash + 1);
    
    if (firstSlash > 0 && secondSlash > firstSlash) {
        String machineId = t.substring(firstSlash + 1, secondSlash);
        String subType = t.substring(secondSlash + 1);
        
        if (subType == "dane") {
            bool found = false;
            int emptySlot = -1;
            
            for(int i=0; i<MAX_MACHINES; i++) {
                if(machines[i].id == machineId) {
                    machines[i].json = msg;
                    machines[i].lastSeen = millis();
                    found = true;
                    break;
                }
                if(machines[i].id == "" && emptySlot == -1) emptySlot = i;
            }
            
            if(!found && emptySlot != -1) {
                machines[emptySlot].id = machineId;
                machines[emptySlot].json = msg;
                machines[emptySlot].lastSeen = millis();
                Serial.println("[HUB] Odkryto nowa maszyne w chmurze: " + machineId);
            }
        }
    }
}

void handleMQTT() {
    if (mqtt_server == "" || routerSSID == "") return;
    if (WiFi.status() != WL_CONNECTED || WiFi.localIP().toString() == "0.0.0.0") return;

    if (!mqtt.connected()) {
        if (millis() - lastMqttReconnect > 15000) {
            lastMqttReconnect = millis();
            String cleanHost = cleanHostAddress(mqtt_server);
            mqtt.setServer(cleanHost.c_str(), 8883);

            Serial.print("[MQTT] Polaczenie z chmura: " + cleanHost + "...");
            espClient.stop(); 
            espClient.setInsecure();
            
            String clientId = "SerwerMultiHUB-" + String(random(0xffff), HEX);
            
            if (mqtt.connect(clientId.c_str(), mqtt_user.c_str(), mqtt_pass.c_str())) {
                Serial.println(" SUKCES!");
                mqtt.subscribe("biuro/+/dane");
                Serial.println("[MQTT] Nasluchuje paczek od wszystkich maszyn na: biuro/+/dane");
            } else {
                Serial.print(" BLAD. Kod: ");
                Serial.println(mqtt.state());
            }
        }
    } else {
        mqtt.loop();
    }
}

// ================================================================
// SETUP & LOOP SERWERA
// ================================================================
void setup() {
    pinMode(PIN_LED, OUTPUT);
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\n--- URUCHAMIAM MULTI-HUB FLOTY (V2.1 PEŁNY KLON) ---");

    for(int i=0; i<MAX_MACHINES; i++) machines[i].id = ""; 

    memory.begin("server_conf", false);
    routerSSID = memory.getString("ssid", "");
    routerPASS = memory.getString("pass", "");
    mqtt_server = memory.getString("msrv", "");
    mqtt_user = memory.getString("musr", "");
    mqtt_pass = memory.getString("mpas", "");

    mqtt_server = cleanHostAddress(mqtt_server);
    mqtt.setBufferSize(4096); 

    WiFi.disconnect(true);
    WiFi.softAPdisconnect(true);
    delay(100);

    if (routerSSID != "") {
        WiFi.mode(WIFI_AP_STA);
        WiFi.begin(routerSSID.c_str(), routerPASS.c_str());
        Serial.println("[WIFI] Laczenie z routerem...");
    } else {
        WiFi.mode(WIFI_AP);
    }
    
    IPAddress local_ip(192, 168, 10, 1); 
    IPAddress gateway(192, 168, 10, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(local_ip, gateway, subnet);
    WiFi.softAP("Granulator_HUB"); 

    if (MDNS.begin("granulator-serwer")) {
        Serial.println("[mDNS] Adres serwera w domu: http://granulator-serwer.local");
    }

    espClient.setInsecure(); 
    mqtt.setServer(mqtt_server.c_str(), 8883);
    mqtt.setCallback(mqttCallback);

    server.on("/", HTTP_GET, handleRoot);
    server.on("/api/machines", HTTP_GET, handleGetMachines);
    server.on("/api/machine_data", HTTP_GET, handleMachineData);
    server.on("/api/server_status", HTTP_GET, handleServerStatus);
    server.on("/api/send_cmd", HTTP_POST, handleSendCommand);
    server.on("/api/save_config", HTTP_POST, handleSaveConfig);
    server.begin();
}

void loop() {
    server.handleClient();
    handleMQTT();
    
    if(millis() % 1000 < 50) digitalWrite(PIN_LED, HIGH); else digitalWrite(PIN_LED, LOW);
}