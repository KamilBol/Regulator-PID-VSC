// ================================================================
// SERWER DYSPOZYTORSKI (CENTRUM DOWODZENIA) - V1.3 FAST
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

#define PIN_LED 2 // Dioda statusowa na plytce ESP32

// --- ZMIENNE KONFIGURACYJNE ---
String routerSSID = "";
String routerPASS = "";
String mqtt_server = "";
String mqtt_user = "";
String mqtt_pass = "";
String target_machine_id = "Granulator_01"; 

// --- BUFOR DANYCH Z HALI ---
String latest_machine_data = "{\"amp\":0,\"setp\":0,\"sysON\":0,\"trip\":0,\"volt\":0}";

unsigned long lastMqttReconnect = 0;
bool isWifiAPActive = false;

// ================================================================
// FUNKCJE POMOCNICZE
// ================================================================
String cleanHostAddress(String host) {
    String clean = host;
    clean.replace("http://", "");
    clean.replace("https://", "");
    int colonIndex = clean.indexOf(':');
    if (colonIndex > 0) {
        clean = clean.substring(0, colonIndex); // Ucina wszystko od dwukropka w prawo
    }
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
  <title>Centrum Dowodzenia - Serwer PID</title>
  <style>
    :root { --bg: #121212; --card: #1e1e1e; --text: #fff; --accent: #00bcd4; --green: #4caf50; --red: #f44336; --orange: #ff9800; --purple: #9c27b0; --yellow: #ffeb3b; --blue: #2196f3; }
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
    .btn-man { background: #555; }
    
    label { display: block; margin-top: 10px; font-size: 14px; color: #aaa; }
    input { width: 100%; padding: 10px; margin-top: 5px; background: #2a2a2a; border: 1px solid #444; color: #fff; border-radius: 6px; box-sizing: border-box; }
    .submit-btn { width: 100%; padding: 15px; margin-top: 15px; background: var(--accent); color: #000; border: none; font-weight: bold; border-radius: 8px; cursor: pointer; }
    
    .status-dot { height: 12px; width: 12px; background-color: var(--red); border-radius: 50%; display: inline-block; margin-right: 8px;}
    .status-ok { background-color: var(--green); box-shadow: 0 0 10px var(--green);}
    .status-wait { background-color: var(--orange); box-shadow: 0 0 10px var(--orange);}
  </style>
</head>
<body>
  <div class="header">
    <h1>📡 Serwer Dyspozytorski</h1>
    <div style="margin-top:8px; font-size:13px; color:#aaa;">
      <span id="mqtt-dot" class="status-dot"></span><span id="mqtt-txt">Łączenie...</span>
    </div>
  </div>

  <div class="nav">
    <button class="tablinks active" onclick="openTab(event, 'Maszyna')">📊 Panel Maszyny</button>
    <button class="tablinks" onclick="openTab(event, 'Sterowanie')">🎮 Sterowanie & OTA</button>
    <button class="tablinks" onclick="openTab(event, 'Serwer')">⚙️ Konfiguracja Serwera</button>
  </div>

  <div id="Maszyna" class="tab-content active">
    
    <div class="card" style="border: 1px solid var(--blue); background: #111;">
      <h3 style="margin-top:0; color:var(--blue); font-size:14px; text-align:center;">Adres IP tego Serwera:</h3>
      <div style="text-align:center; font-size:24px; font-weight:bold; color:var(--text);" id="srvIP">--.--.--.--</div>
    </div>

    <div class="card" style="display: flex; justify-content: space-between;">
      <div id="btnSys" class="ctrl-btn btn-off" style="text-align:center;">Zasilanie: OFF</div>
      <div id="btnMode" class="ctrl-btn btn-man" style="text-align:center;">Zdalny Podgląd</div>
    </div>
    
    <div class="card">
      <h3 style="margin-top:0; color:var(--green);">Parametry z Hali (Na Żywo)</h3>
      <div class="row"><span>Prąd Maszyny:</span> <span class="val" id="val-amp">-- A</span></div>
      <div class="row"><span>Cel PID (Limit):</span> <span class="val" id="val-setp">-- A</span></div>
      <div class="row"><span>Awaria (Przeciążenie):</span> <span class="val" id="val-trip" style="color:var(--red);">NIE</span></div>
      <div class="row"><span>Napięcie Sieci:</span> <span class="val" id="val-volt" style="color:var(--orange);">-- V</span></div>
    </div>
  </div>

  <div id="Sterowanie" class="tab-content">
    <div class="card">
      <h3 style="margin-top:0; color:var(--accent);">🎮 Zdalne Uruchamianie</h3>
      <div style="display:flex; justify-content: space-between;">
        <button class="submit-btn" style="width:48%; background:var(--green); color:#fff; margin-top:0;" onclick="sendCommand('SYSTEM=ON')">START</button>
        <button class="submit-btn" style="width:48%; background:var(--red); color:#fff; margin-top:0;" onclick="sendCommand('SYSTEM=OFF')">STOP</button>
      </div>
    </div>

    <div class="card">
      <h3 style="margin-top:0;">⚡ Zmień Limit Amperów</h3>
      <label>Wpisz nowe max Ampery (np. 42.5)</label>
      <input type="number" step="0.1" id="newLimit">
      <button class="submit-btn" style="background:var(--blue); color:#fff;" onclick="sendLimit()">WYŚLIJ LIMIT</button>
    </div>

    <div class="card" style="border: 1px solid var(--purple);">
      <h3 style="margin-top:0; color:var(--purple);">🚀 Zdalna Aktualizacja (OTA)</h3>
      <p style="font-size:12px; color:#aaa;">Wklej link (Raw URL) do pliku .bin z serwera np. GitHub. Maszyna na hali sama go pobierze i zainstaluje.</p>
      <input type="text" id="otaUrl" placeholder="https://raw.githubusercontent.com/...">
      <button class="submit-btn" style="background:var(--purple); color:#fff;" onclick="sendOTA()">ZLEC AKTUALIZACJĘ DO MASZYNY</button>
    </div>
  </div>

  <div id="Serwer" class="tab-content">
    <div class="card">
      <h3 style="margin-top:0; color:var(--yellow);">⚙️ Ustawienia Sieci i Chmury</h3>
      <p style="font-size:12px; color:#aaa;">Dane potrzebne do pracy tego Serwera w domu.</p>
      <form onsubmit="saveConfig(event)">
        <label>ID Maszyny (Którą maszynę śledzimy?)</label>
        <input type="text" id="c_tid" required>
        <label>WiFi SSID (Router w domu)</label>
        <input type="text" id="c_ssid">
        <label>WiFi Hasło</label>
        <input type="password" id="c_pass">
        <label>MQTT Adres Brokera (HiveMQ Cluster URL)</label>
        <input type="text" id="c_msrv">
        <label>MQTT Użytkownik</label>
        <input type="text" id="c_musr">
        <label>MQTT Hasło</label>
        <input type="password" id="c_mpas">
        <button type="submit" class="submit-btn" style="background:var(--yellow); color:#000;">ZAPISZ I RESTARTUJ SERWER</button>
      </form>
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

    function sendCommand(cmd) {
      if(confirm("Wysłać rozkaz: " + cmd + " do maszyny na hali?")) {
        fetch('/api/send_cmd', {
          method: 'POST',
          headers: {'Content-Type': 'application/x-www-form-urlencoded'},
          body: 'cmd=' + encodeURIComponent(cmd)
        }).then(() => alert("Rozkaz wysłany!"));
      }
    }

    function sendLimit() {
      let v = document.getElementById('newLimit').value;
      if(!v) return alert("Wpisz wartosc!");
      sendCommand("LIMIT_MAX=" + v);
      document.getElementById('newLimit').value = "";
    }

    function sendOTA() {
      let url = document.getElementById('otaUrl').value;
      if(!url.startsWith("http")) return alert("Podaj poprawny link HTTP/HTTPS!");
      sendCommand("OTA=" + url);
      document.getElementById('otaUrl').value = "";
    }

    function saveConfig(e) {
      e.preventDefault();
      let payload = `tid=${encodeURIComponent(document.getElementById('c_tid').value)}&ssid=${encodeURIComponent(document.getElementById('c_ssid').value)}&pass=${encodeURIComponent(document.getElementById('c_pass').value)}&msrv=${encodeURIComponent(document.getElementById('c_msrv').value)}&musr=${encodeURIComponent(document.getElementById('c_musr').value)}&mpas=${encodeURIComponent(document.getElementById('c_mpas').value)}`;
      
      fetch('/api/save_config', {
        method: 'POST',
        headers: {'Content-Type': 'application/x-www-form-urlencoded'},
        body: payload
      }).then(() => {
        alert("Konfiguracja serwera zapisana. Trwa restart...");
        setTimeout(()=>location.reload(), 5000);
      });
    }

    setInterval(() => {
      fetch('/api/machine_data').then(r => r.json()).then(data => {
        document.getElementById('val-amp').innerText = data.amp + " A";
        document.getElementById('val-setp').innerText = data.setp + " A";
        document.getElementById('val-volt').innerText = data.volt + " V";
        
        let btnSys = document.getElementById('btnSys');
        if(data.sysON === 1) {
            btnSys.className = "ctrl-btn btn-on";
            btnSys.innerText = "Hala Praca: ON";
        } else {
            btnSys.className = "ctrl-btn btn-off";
            btnSys.innerText = "Hala Praca: OFF";
        }

        let valTrip = document.getElementById('val-trip');
        if(data.trip === 1) {
            valTrip.innerText = "TAK (Odcięta!)";
        } else {
            valTrip.innerText = "NIE";
        }
      });
    }, 1000);

    setInterval(() => {
      fetch('/api/server_status').then(r => r.json()).then(data => {
        let dot = document.getElementById('mqtt-dot');
        let txt = document.getElementById('mqtt-txt');
        
        if(data.ip === "0.0.0.0" || data.ip === "192.168.10.1") {
            dot.className = "status-dot";
            txt.innerText = "Brak internetu (Tylko sieć lokalna Serwera)";
        } else if(data.mqtt_connected === false) {
            dot.className = "status-dot status-wait";
            txt.innerText = "WiFi OK! Szukam chmury HiveMQ...";
        } else {
            dot.className = "status-dot status-ok";
            txt.innerText = "Połączono z Maszyną (" + data.target_id + ")";
        }

        document.getElementById('srvIP').innerText = data.ip;

        if(document.activeElement.tagName !== "INPUT") {
            document.getElementById('c_tid').value = data.target_id;
            document.getElementById('c_ssid').value = data.ssid;
            document.getElementById('c_msrv').value = data.mqtt_srv;
            document.getElementById('c_musr').value = data.mqtt_usr;
        }
      });
    }, 2000);

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

void handleMachineData() {
    if(!checkAuth()) return;
    server.send(200, "application/json", latest_machine_data);
}

void handleServerStatus() {
    if(!checkAuth()) return;
    String json = "{";
    json += "\"mqtt_connected\":" + String(mqtt.connected() ? "true" : "false") + ",";
    
    String currentIP = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "192.168.10.1";
    json += "\"ip\":\"" + currentIP + "\",";
    
    json += "\"target_id\":\"" + target_machine_id + "\",";
    json += "\"ssid\":\"" + routerSSID + "\",";
    json += "\"mqtt_srv\":\"" + mqtt_server + "\",";
    json += "\"mqtt_usr\":\"" + mqtt_user + "\"";
    json += "}";
    server.send(200, "application/json", json);
}

void handleSendCommand() {
    if(!checkAuth()) return;
    if (server.hasArg("cmd") && mqtt.connected()) {
        String cmd = server.arg("cmd");
        String topic = "biuro/" + target_machine_id + "/rozkazy";
        mqtt.publish(topic.c_str(), cmd.c_str());
        Serial.println("[MQTT] Wysłano rozkaz: " + cmd + " do " + topic);
        server.send(200, "text/plain", "OK");
    } else {
        server.send(500, "text/plain", "Brak polaczenia MQTT");
    }
}

void handleSaveConfig() {
    if(!checkAuth()) return;
    if(server.hasArg("tid")) target_machine_id = server.arg("tid");
    if(server.hasArg("ssid")) routerSSID = server.arg("ssid");
    if(server.hasArg("pass")) routerPASS = server.arg("pass");
    if(server.hasArg("msrv")) mqtt_server = cleanHostAddress(server.arg("msrv")); // Czyszczenie adresu!
    if(server.hasArg("musr")) mqtt_user = server.arg("musr");
    if(server.hasArg("mpas")) mqtt_pass = server.arg("mpas");

    memory.putString("tid", target_machine_id);
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
// OBSŁUGA MQTT W SERWERZE
// ================================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String msg = "";
    for (int i = 0; i < length; i++) msg += (char)payload[i];
    
    String expectedTopic = "biuro/" + target_machine_id + "/dane";
    if (String(topic) == expectedTopic) {
        latest_machine_data = msg; 
    }
}

void handleMQTT() {
    if (mqtt_server == "" || routerSSID == "") return;
    if (WiFi.status() != WL_CONNECTED || WiFi.localIP().toString() == "0.0.0.0") return;

    if (!mqtt.connected()) {
        // ZWIĘKSZONY TIMEOUT - Serwer WWW nie będzie blokowany przez uwalone TLS!
        if (millis() - lastMqttReconnect > 15000) {
            lastMqttReconnect = millis();
            
            // Jeszcze raz czyścimy adres z pamięci, na wypadek gdybyś załadował zły z EEPROM
            String cleanHost = cleanHostAddress(mqtt_server);
            mqtt.setServer(cleanHost.c_str(), 8883);

            Serial.print("[MQTT] Proba polaczenia z chmura: " + cleanHost + "...");
            
            espClient.stop(); 
            espClient.setInsecure();
            
            String clientId = "SerwerBiurko-" + String(random(0xffff), HEX);
            
            if (mqtt.connect(clientId.c_str(), mqtt_user.c_str(), mqtt_pass.c_str())) {
                Serial.println(" SUKCES!");
                String subTopic = "biuro/" + target_machine_id + "/dane";
                mqtt.subscribe(subTopic.c_str());
                Serial.println("[MQTT] Nasluchuje informacji od maszyny na: " + subTopic);
            } else {
                Serial.print(" BLAD TLS/LOGOWANIA! Kod: ");
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
    Serial.println("\n\n--- URUCHAMIAM SERWER CENTRUM DOWODZENIA (V1.3 FAST) ---");

    memory.begin("server_conf", false);
    target_machine_id = memory.getString("tid", "Granulator_01");
    routerSSID = memory.getString("ssid", "");
    routerPASS = memory.getString("pass", "");
    mqtt_server = memory.getString("msrv", "");
    mqtt_user = memory.getString("musr", "");
    mqtt_pass = memory.getString("mpas", "");

    // Zabezpieczenie przed starym, brudnym adresem w pamieci EEPROM
    mqtt_server = cleanHostAddress(mqtt_server);

    mqtt.setBufferSize(1024);

    WiFi.disconnect(true);
    WiFi.softAPdisconnect(true);
    delay(100);

    if (routerSSID != "") {
        WiFi.mode(WIFI_AP_STA);
        WiFi.begin(routerSSID.c_str(), routerPASS.c_str());
        Serial.println("[WIFI] Proba polaczenia z routerem domowym...");
    } else {
        WiFi.mode(WIFI_AP);
        Serial.println("[WIFI] Brak danych routera. Start w trybie tylko Serwisowym AP.");
    }
    
    IPAddress local_ip(192, 168, 10, 1); 
    IPAddress gateway(192, 168, 10, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(local_ip, gateway, subnet);
    WiFi.softAP("Granulator_SERWER"); 

    if (MDNS.begin("granulator-serwer")) {
        Serial.println("[mDNS] Adres serwera w domu to: http://granulator-serwer.local");
    }

    espClient.setInsecure(); 
    mqtt.setServer(mqtt_server.c_str(), 8883);
    mqtt.setCallback(mqttCallback);

    server.on("/", HTTP_GET, handleRoot);
    server.on("/api/machine_data", HTTP_GET, handleMachineData);
    server.on("/api/server_status", HTTP_GET, handleServerStatus);
    server.on("/api/send_cmd", HTTP_POST, handleSendCommand);
    server.on("/api/save_config", HTTP_POST, handleSaveConfig);
    server.begin();
    
    Serial.println("[SYS] Serwer Web uruchomiony.");
}

void loop() {
    // Serwer WWW obsługiwany jako absolutny priorytet
    server.handleClient();
    handleMQTT();
    
    if(millis() % 1000 < 50) digitalWrite(PIN_LED, HIGH); else digitalWrite(PIN_LED, LOW);
}