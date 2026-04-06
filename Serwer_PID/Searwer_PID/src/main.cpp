// ================================================================
// SERWER DYSPOZYTORSKI (CENTRUM DOWODZENIA) - V1.0
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
String target_machine_id = "Granulator_01"; // Do jakiej maszyny wysylamy rozkazy

// --- BUFOR DANYCH Z HALI ---
// Tutaj przechowujemy ostatnia paczke JSON jaka przyszla z maszyny
String latest_machine_data = "{\"amp\":0,\"setp\":0,\"sysON\":0,\"trip\":0,\"volt\":0}";

unsigned long lastMqttReconnect = 0;
bool isWifiAPActive = false;

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
  <title>Serwer Dyspozytorski - Granulator</title>
  <style>
    :root { --bg: #0a0a0a; --card: #1a1a1a; --text: #e0e0e0; --accent: #ff5722; --green: #4caf50; --red: #f44336; --blue: #2196f3; }
    body { background-color: var(--bg); color: var(--text); font-family: 'Segoe UI', sans-serif; margin: 0; padding: 0; }
    .header { background: #000; padding: 20px; text-align: center; border-bottom: 3px solid var(--accent); }
    h1 { margin: 0; font-size: 24px; color: var(--accent); text-transform: uppercase; letter-spacing: 2px;}
    .container { padding: 15px; max-width: 600px; margin: 0 auto; }
    .card { background: var(--card); border-radius: 10px; padding: 20px; margin-bottom: 20px; border: 1px solid #333; box-shadow: 0 8px 16px rgba(0,0,0,0.6); }
    h3 { margin-top: 0; color: var(--blue); border-bottom: 1px solid #333; padding-bottom: 10px;}
    .data-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; }
    .data-box { background: #222; padding: 15px; border-radius: 8px; text-align: center; border-left: 4px solid var(--green);}
    .data-box.alert { border-left: 4px solid var(--red); }
    .data-label { font-size: 12px; color: #888; text-transform: uppercase; }
    .data-val { font-size: 28px; font-weight: bold; margin-top: 5px; color: #fff;}
    
    label { display: block; margin-top: 15px; font-size: 13px; color: #aaa; }
    input { width: 100%; padding: 12px; margin-top: 5px; background: #111; border: 1px solid #444; color: #fff; border-radius: 6px; box-sizing: border-box; font-size:16px;}
    .btn { width: 100%; padding: 15px; margin-top: 15px; border: none; font-weight: bold; border-radius: 6px; cursor: pointer; font-size: 16px; transition: 0.3s; text-transform: uppercase;}
    .btn-blue { background: var(--blue); color: #fff; }
    .btn-red { background: var(--red); color: #fff; }
    .btn-green { background: var(--green); color: #fff; }
    .btn-accent { background: var(--accent); color: #fff; }
    .btn:active { opacity: 0.7; transform: scale(0.98); }
    
    .status-dot { height: 12px; width: 12px; background-color: var(--red); border-radius: 50%; display: inline-block; margin-right: 8px;}
    .status-ok { background-color: var(--green); box-shadow: 0 0 10px var(--green);}
  </style>
</head>
<body>
  <div class="header">
    <h1>📡 Serwer Dyspozytorski</h1>
    <div style="margin-top:10px; font-size:14px; color:#aaa;">
      <span id="mqtt-dot" class="status-dot"></span><span id="mqtt-txt">Chmura rozłączona</span>
    </div>
  </div>

  <div class="container">
    
    <div class="card">
      <h3 style="color: var(--green);">📊 Parametry Maszyny (Na żywo)</h3>
      <div class="data-grid">
        <div class="data-box" id="box-amp">
          <div class="data-label">Obciążenie (Prąd)</div>
          <div class="data-val" id="val-amp">-- A</div>
        </div>
        <div class="data-box" id="box-setp">
          <div class="data-label">Aktualny Cel (Limit)</div>
          <div class="data-val" id="val-setp">-- A</div>
        </div>
        <div class="data-box">
          <div class="data-label">Napięcie Sieci</div>
          <div class="data-val" id="val-volt">-- V</div>
        </div>
        <div class="data-box" id="box-sys">
          <div class="data-label">Status Falowników</div>
          <div class="data-val" id="val-sys">OFF</div>
        </div>
      </div>
      <div style="margin-top: 15px; text-align: center; color: var(--red); font-weight: bold; display:none;" id="alert-trip">
        ⚠️ UWAGA: MASZYNA W STANIE PRZECIĄŻENIA!
      </div>
    </div>

    <div class="card">
      <h3 style="color: var(--accent);">🎮 Zdalne Sterowanie</h3>
      <div style="display:flex; gap:10px; margin-bottom: 20px;">
        <button class="btn btn-green" style="margin-top:0;" onclick="sendCommand('SYSTEM=ON')">START MASZYNY</button>
        <button class="btn btn-red" style="margin-top:0;" onclick="sendCommand('SYSTEM=OFF')">STOP (AWARYJNY)</button>
      </div>
      
      <div style="background: #222; padding: 15px; border-radius: 8px; border: 1px solid #444;">
        <label style="margin-top:0; color:#fff; font-weight:bold;">Zmień dopuszczalny limit amperów:</label>
        <input type="number" step="0.1" id="newLimit" placeholder="Wpisz nowe max Ampery (np. 42.5)">
        <button class="btn btn-blue" onclick="sendLimit()">WYŚLIJ NOWY LIMIT DO MASZYNY</button>
      </div>
    </div>

    <div class="card" style="border-color: #9c27b0;">
      <h3 style="color: #9c27b0;">🚀 Zdalna Aktualizacja Oprogramowania (OTA)</h3>
      <p style="font-size:13px; color:#aaa;">Wklej surowy link URL (Raw) do pliku .bin z serwera np. GitHub, aby maszyna na hali pobrała nowy kod.</p>
      <input type="text" id="otaUrl" placeholder="https://raw.githubusercontent.com/.../update.bin">
      <button class="btn" style="background:#9c27b0; color:#fff;" onclick="sendOTA()">ZLEC AKTUALIZACJĘ DO MASZYNY</button>
    </div>

    <div class="card">
      <h3>⚙️ Konfiguracja tego Serwera</h3>
      <p style="font-size:12px; color:#aaa;">Z kim ten panel ma się komunikować?</p>
      <form onsubmit="saveConfig(event)">
        <label>ID Docelowej Maszyny (Z kim gadamy?)</label>
        <input type="text" id="c_tid" required>
        <label>WiFi SSID (Router w domu)</label>
        <input type="text" id="c_ssid">
        <label>WiFi Hasło</label>
        <input type="password" id="c_pass">
        <label>MQTT Adres Brokera (HiveMQ)</label>
        <input type="text" id="c_msrv">
        <label>MQTT Użytkownik</label>
        <input type="text" id="c_musr">
        <label>MQTT Hasło</label>
        <input type="password" id="c_mpas">
        <button type="submit" class="btn btn-accent">ZAPISZ I RESTARTUJ SERWER</button>
      </form>
    </div>

  </div>

  <script>
    function sendCommand(cmd) {
      if(confirm("Wysłać rozkaz: " + cmd + " ?")) {
        fetch('/api/send_cmd', {
          method: 'POST',
          headers: {'Content-Type': 'application/x-www-form-urlencoded'},
          body: 'cmd=' + encodeURIComponent(cmd)
        }).then(() => alert("Rozkaz wysłany w chmurę!"));
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
      let tid = document.getElementById('c_tid').value;
      let ssid = document.getElementById('c_ssid').value;
      let pass = document.getElementById('c_pass').value;
      let msrv = document.getElementById('c_msrv').value;
      let musr = document.getElementById('c_musr').value;
      let mpas = document.getElementById('c_mpas').value;
      
      let payload = `tid=${encodeURIComponent(tid)}&ssid=${encodeURIComponent(ssid)}&pass=${encodeURIComponent(pass)}&msrv=${encodeURIComponent(msrv)}&musr=${encodeURIComponent(musr)}&mpas=${encodeURIComponent(mpas)}`;
      
      fetch('/api/save_config', {
        method: 'POST',
        headers: {'Content-Type': 'application/x-www-form-urlencoded'},
        body: payload
      }).then(() => {
        alert("Konfiguracja serwera zapisana. Trwa restart...");
        setTimeout(()=>location.reload(), 5000);
      });
    }

    // Odswiezanie danych z chmury co 1 sekunde
    setInterval(() => {
      fetch('/api/machine_data').then(r => r.json()).then(data => {
        // Data to po prostu przeklejony JSON od maszyny
        document.getElementById('val-amp').innerText = data.amp + " A";
        document.getElementById('val-setp').innerText = data.setp + " A";
        document.getElementById('val-volt').innerText = data.volt + " V";
        
        let sysBox = document.getElementById('box-sys');
        let sysVal = document.getElementById('val-sys');
        if(data.sysON === 1) {
            sysBox.style.borderLeftColor = "var(--green)";
            sysVal.innerText = "PRACUJE (ON)";
            sysVal.style.color = "var(--green)";
        } else {
            sysBox.style.borderLeftColor = "#555";
            sysVal.innerText = "ZATRZYMANA (OFF)";
            sysVal.style.color = "#aaa";
        }

        let ampBox = document.getElementById('box-amp');
        let alertTrip = document.getElementById('alert-trip');
        if(data.trip === 1) {
            ampBox.className = "data-box alert";
            alertTrip.style.display = "block";
        } else {
            ampBox.className = "data-box";
            alertTrip.style.display = "none";
        }
      });
    }, 1000);

    // Odswiezanie statusu samego serwera
    setInterval(() => {
      fetch('/api/server_status').then(r => r.json()).then(data => {
        let dot = document.getElementById('mqtt-dot');
        let txt = document.getElementById('mqtt-txt');
        if(data.mqtt_connected) {
            dot.className = "status-dot status-ok";
            txt.innerText = "Chmura Połączona (" + data.target_id + ")";
        } else {
            dot.className = "status-dot";
            txt.innerText = "Szukanie chmury...";
        }

        // Aktualizacja pól configu jeśli nie są w focusie
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
    // Zwracamy przegladarce surowy tekst jaki dostalismy z MQTT od Maszyny
    server.send(200, "application/json", latest_machine_data);
}

void handleServerStatus() {
    if(!checkAuth()) return;
    String json = "{";
    json += "\"mqtt_connected\":" + String(mqtt.connected() ? "true" : "false") + ",";
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
    if(server.hasArg("msrv")) mqtt_server = server.arg("msrv");
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
    
    // Jesli to sa dane z docelowej maszyny, zapisujemy do pamieci RAM (Dla WebUI)
    String expectedTopic = "biuro/" + target_machine_id + "/dane";
    if (String(topic) == expectedTopic) {
        latest_machine_data = msg; 
    }
}

void handleMQTT() {
    if (mqtt_server == "" || WiFi.status() != WL_CONNECTED) return;

    if (!mqtt.connected()) {
        if (millis() - lastMqttReconnect > 5000) {
            lastMqttReconnect = millis();
            Serial.print("[MQTT] Proba polaczenia Serwera z chmura...");
            
            String clientId = "SerwerBiurko-" + String(random(0xffff), HEX);
            if (mqtt.connect(clientId.c_str(), mqtt_user.c_str(), mqtt_pass.c_str())) {
                Serial.println(" SUKCES!");
                
                // Serwer NASŁUCHUJE danych od maszyny
                String subTopic = "biuro/" + target_machine_id + "/dane";
                mqtt.subscribe(subTopic.c_str());
                Serial.println("[MQTT] Nasluchuje informacji z: " + subTopic);
            } else {
                Serial.println(" BLAD!");
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
    Serial.println("\n\n--- URUCHAMIAM SERWER CENTRUM DOWODZENIA ---");

    memory.begin("server_conf", false);
    target_machine_id = memory.getString("tid", "Granulator_01");
    routerSSID = memory.getString("ssid", "");
    routerPASS = memory.getString("pass", "");
    mqtt_server = memory.getString("msrv", "");
    mqtt_user = memory.getString("musr", "");
    mqtt_pass = memory.getString("mpas", "");

    WiFi.disconnect(true);
    WiFi.softAPdisconnect(true);
    delay(100);

    if (routerSSID != "") {
        WiFi.mode(WIFI_AP_STA);
        WiFi.begin(routerSSID.c_str(), routerPASS.c_str());
    } else {
        WiFi.mode(WIFI_AP);
    }
    
    IPAddress local_ip(192, 168, 10, 1); // Serwer ma inna pule niz maszyna!
    IPAddress gateway(192, 168, 10, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(local_ip, gateway, subnet);
    WiFi.softAP("Granulator_SERWER"); // Otwarta siec do awaryjnej konfiguracji serwera

    if (MDNS.begin("granulator-serwer")) {
        Serial.println("[mDNS] Adres serwera w domu to: http://granulator-serwer.local");
    }

    espClient.setInsecure(); // Wymagane dla darmowego HiveMQ Cloud z portem 8883
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
    server.handleClient();
    handleMQTT();
    
    // Miganie diody - sygnalizacja ze serwer zyje
    if(millis() % 1000 < 50) digitalWrite(PIN_LED, HIGH); else digitalWrite(PIN_LED, LOW);
}