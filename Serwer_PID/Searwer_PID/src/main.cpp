// ================================================================
// SERWER DYSPOZYTORSKI (MULTI-HUB) - V2.0
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
    :root { --bg: #121212; --card: #1e1e1e; --text: #fff; --accent: #00bcd4; --green: #4caf50; --red: #f44336; --orange: #ff9800; --purple: #9c27b0; --yellow: #ffeb3b; }
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
    
    .ctrl-btn { width: 48%; padding: 15px; font-size: 16px; font-weight: bold; border-radius: 8px; border: none; cursor: pointer; color: #fff; text-align:center;}
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
    
    #btn-back { position: absolute; left: 15px; top: 15px; background: none; border: 1px solid var(--accent); color: var(--accent); padding: 5px 15px; border-radius: 5px; cursor: pointer; display: none;}
  </style>
</head>
<body>
  <div class="header">
    <button id="btn-back" onclick="showDashboard()">🔙 Wróć</button>
    <h1>📡 HUB DOWODZENIA</h1>
    <div style="margin-top:5px; font-size:12px; color:#888;">IP Serwera: <span id="srvIP">--</span> | <span id="mqStat">Szukam chmury...</span></div>
  </div>

  <div id="view-dashboard" class="container" style="display:block;">
    <h3 style="color:#aaa;">Dostępne Maszyny:</h3>
    <div id="machine-list">Ładowanie z chmury...</div>
    
    <div class="card" style="margin-top: 40px; border: 1px solid #555;">
      <h3 style="margin-top:0; color:var(--yellow);">⚙️ Konfiguracja Serwera HUB</h3>
      <form onsubmit="saveConfig(event)">
        <label>WiFi SSID (Router domowy/biurowy)</label>
        <input type="text" id="c_ssid">
        <label>WiFi Hasło</label>
        <input type="password" id="c_pass">
        <label>MQTT Adres Brokera (Cluster URL)</label>
        <input type="text" id="c_msrv">
        <label>MQTT Użytkownik</label>
        <input type="text" id="c_musr">
        <label>MQTT Hasło</label>
        <input type="password" id="c_mpas">
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
      <h2 style="text-align:center; color:var(--text); margin-top:0;">Sterujesz: <span id="active-m-id" style="color:var(--accent);">---</span></h2>
      
      <div id="Panel" class="tab-content active">
        <div class="card" style="display: flex; justify-content: space-between;">
          <div id="btnSys" class="ctrl-btn btn-off" onclick="sendCmd('SYSTEM=TOGGLE')">Zasilanie: OFF</div>
          <div id="btnMode" class="ctrl-btn btn-auto" onclick="sendCmd('MODE=TOGGLE')">Tryb: AUTO</div>
        </div>
        <div class="card">
          <div class="row"><span>Prąd Maszyny:</span> <span class="val" id="amp">-- A</span></div>
          <div class="row"><span>Cel PID (Limit):</span> <span class="val" id="setp">-- A</span></div>
          <div class="row"><span>Awaria (Przeciążenie):</span> <span class="val" id="trip" style="color:var(--red);">NIE</span></div>
          <div class="row"><span>Wyjście na Falownik 1:</span> <span class="val" id="dac" style="color:var(--orange);">-- V</span></div>
          <div class="row"><span>Wyjście na Falownik 2:</span> <span class="val" id="dac2v" style="color:var(--purple);">-- V</span></div>
        </div>
        <button class="submit-btn" style="background:var(--red); color:#fff;" onclick="if(confirm('Zrestartować maszynę?')) sendCmd('RESTART=1')">🔄 ZDALNY RESTART MASZYNY</button>
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
        <p style="color:var(--orange); font-size:12px; text-align:center;">Uwaga: Wprowadzane tu zmiany zostaną wysłane chmurą MQTT do wybranej maszyny.</p>
        
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
            <label>P</label><input type="number" step="0.01" id="kp" required>
            <label>I</label><input type="number" step="0.01" id="ki" required>
            <label>D</label><input type="number" step="0.01" id="kd" required>
            <button type="submit" class="submit-btn" style="background:#555; color:#fff;">WYŚLIJ PID DO MASZYNY</button>
          </form>
        </div>
      </div>

      <div id="SD" class="tab-content">
        <div class="card">
          <h3 style="margin-top:0; color:#00bcd4;">🩺 Status Sprzętu (Na Żywo)</h3>
          <div class="row"><span>Zasilanie (PZEM-004T):</span> <span id="st_pzem" class="badge-err">CZEKAM NA DANE...</span></div>
          <div class="row"><span>Ekran HMI (Nextion):</span> <span id="st_nex" class="badge-err">CZEKAM NA DANE...</span></div>
          <div class="row"><span>Klimat (DHT11):</span> <span id="st_dht" class="badge-err">CZEKAM NA DANE...</span></div>
        </div>
        <p style="font-size:12px; color:#aaa; text-align:center;">Pełna diagnostyka będzie dostępna po aktualizacji firmware'u na wybranej maszynie (Faza 4).</p>
      </div>

      <div id="OTA" class="tab-content">
        <div class="card" style="border: 2px solid #9c27b0;">
          <h3 style="margin-top:0; color:#9c27b0;">🚀 Zdalna Aktualizacja Firmware (Chmura)</h3>
          <p style="font-size:13px; color:#aaa;">Wklej surowy link URL do pliku .bin (np. z GitHub), aby zdalnie wgrać nowy system do wskazanej maszyny.</p>
          <input type="text" id="otaUrl" placeholder="https://raw.githubusercontent.com/.../update.bin">
          <button class="submit-btn" style="background:#9c27b0; color:#fff;" onclick="cmdOTA()">ROZPOCZNIJ ZDALNE FLASHOWANIE</button>
        </div>
      </div>

    </div>
  </div>

  <script>
    let activeMachine = "";
    let lastFocusTime = 0;

    window.addEventListener('DOMContentLoaded', () => {
        document.querySelectorAll('input').forEach(i => {
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
      
      // Reset zakładek
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
      }).then(() => alert("Rozkaz wprowadzony do sieci MQTT!"));
    }

    function cmdLimits(e) { e.preventDefault(); sendCmd(`CMD:LIMITS:${document.getElementById('minL').value}:${document.getElementById('maxL').value}`); }
    function cmdRatios(e) { e.preventDefault(); sendCmd(`CMD:RATIOS:${document.getElementById('dac1r').value}:${document.getElementById('dac2r').value}`); }
    function cmdPID(e) { e.preventDefault(); sendCmd(`CMD:PID:${document.getElementById('kp').value}:${document.getElementById('ki').value}:${document.getElementById('kd').value}`); }
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
      .then(() => { alert("Zapisano! Trwa restart HUBa..."); setTimeout(()=>location.reload(), 5000); });
    }

    // --- PĘTLA POBIERAJĄCA DANE Z SERWERA (CO 1 SEKUNDĘ) ---
    setInterval(() => {
      // Pobieranie listy maszyn do Dashboardu
      fetch('/api/machines').then(r => r.json()).then(data => {
        if(activeMachine === "") {
          let html = "";
          data.forEach(m => {
            html += `<div class='machine-btn' onclick='selectMachine("${m.id}")'>
                       <div class='m-title'>⚙️ ${m.id}</div>
                       <div class='m-stat'><span>Ostatni sygnał: ${m.age}s temu</span> <span><span class='dot ok'></span> ONLINE</span></div>
                     </div>`;
          });
          if(data.length === 0) html = "<div style='text-align:center; padding:20px; color:#666;'>Brak połączonych maszyn w chmurze...</div>";
          document.getElementById('machine-list').innerHTML = html;
        }
      });

      // Jeśli jesteśmy w widoku maszyny, pobierz jej szczegóły
      if(activeMachine !== "") {
        fetch('/api/machine_data?id=' + activeMachine).then(r => r.json()).then(d => {
          document.getElementById('amp').innerText = (d.amp!==undefined?d.amp:"--") + " A";
          document.getElementById('setp').innerText = (d.setp!==undefined?d.setp:"--") + " A";
          document.getElementById('volt').innerText = (d.volt!==undefined?d.volt:"--") + " V";
          
          let btnSys = document.getElementById('btnSys');
          if(d.sysON === 1) { btnSys.className = "ctrl-btn btn-on"; btnSys.innerText = "Zasilanie: ON"; }
          else { btnSys.className = "ctrl-btn btn-off"; btnSys.innerText = "Zasilanie: OFF"; }

          let trip = document.getElementById('trip');
          if(d.trip === 1) { trip.innerText = "TAK (Odcięta!)"; trip.style.color = "var(--red)"; }
          else { trip.innerText = "NIE"; trip.style.color = "var(--green)"; }

          // --- TE POLA ZACZNĄ SIĘ WYPEŁNIAĆ PO WGRANIU NOWEGO KODU DO MASZYNY (Faza 4) ---
          if(d.dac !== undefined) document.getElementById('dac').innerText = d.dac + " V";
          if(d.dac2v !== undefined) document.getElementById('dac2v').innerText = d.dac2v + " V";
          if(d.pow !== undefined) document.getElementById('pow').innerText = d.pow + " W";
          if(d.ap_pow !== undefined) document.getElementById('ap_pow').innerText = d.ap_pow + " VA";
          if(d.re_pow !== undefined) document.getElementById('re_pow').innerText = d.re_pow + " Var";
          if(d.pf !== undefined) document.getElementById('pf').innerText = d.pf;
          if(d.temp !== undefined) document.getElementById('temp').innerText = d.temp + " °C";
          if(d.hum !== undefined) document.getElementById('hum').innerText = d.hum + " %";

          if (Date.now() - lastFocusTime > 10000) {
              if(d.minL !== undefined) document.getElementById('minL').value = d.minL;
              if(d.maxL !== undefined) document.getElementById('maxL').value = d.maxL;
              if(d.kp !== undefined) document.getElementById('kp').value = d.kp;
              if(d.ki !== undefined) document.getElementById('ki').value = d.ki;
              if(d.kd !== undefined) document.getElementById('kd').value = d.kd;
              if(d.dac1R !== undefined) document.getElementById('dac1r').value = d.dac1R;
              if(d.dac2R !== undefined) document.getElementById('dac2r').value = d.dac2R;
          }
          
          if(d.pzem !== undefined) {
             document.getElementById('st_pzem').innerText = d.pzem==1?"ONLINE":"BŁĄD";
             document.getElementById('st_pzem').className = d.pzem==1?"badge-ok":"badge-err";
          }
          if(d.nex !== undefined) {
             document.getElementById('st_nex').innerText = d.nex==1?"ONLINE":"BŁĄD";
             document.getElementById('st_nex').className = d.nex==1?"badge-ok":"badge-err";
          }
        }).catch(e => console.log("Brak JSON"));
      }

      // Odświeżanie paska statusu serwera
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
            // Uznajemy maszyne za martwa, jesli nie nadala sygnalu przez ostatnie 30 sekund
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
    
    // Parser tematu: biuro/Granulator_01/dane
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
                
                // + (PLUS) to Wildcard MQTT - lapie kazda maszyne w srodkowym wezle!
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
    Serial.println("\n\n--- URUCHAMIAM MULTI-HUB FLOTY (V2.0) ---");

    for(int i=0; i<MAX_MACHINES; i++) machines[i].id = ""; // Czyszczenie listy na start

    memory.begin("server_conf", false);
    routerSSID = memory.getString("ssid", "");
    routerPASS = memory.getString("pass", "");
    mqtt_server = memory.getString("msrv", "");
    mqtt_user = memory.getString("musr", "");
    mqtt_pass = memory.getString("mpas", "");

    mqtt_server = cleanHostAddress(mqtt_server);
    mqtt.setBufferSize(2048); // Zwiekszamy bufor, bo "Gruby JSON" od maszyny bedzie wiekszy!

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