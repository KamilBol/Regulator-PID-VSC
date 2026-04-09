#ifndef STRONA_WWW_H
#define STRONA_WWW_H

#include <Arduino.h>

// =====================================================================================
// GŁÓWNY KOD STRONY INTERNETOWEJ (HTML + CSS + JAVASCRIPT)
// Przechowywany w pamięci PROGMEM (Flash), aby nie zużywać cennego RAM-u procesora.
// =====================================================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pl">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Regulator PID - Centrum Kontroli</title>
  
  <style>
    /* ZMIENNE GLOBALNE: Kolorystyka i motyw (Ciemny) */
    :root { 
        --bg: #121212; 
        --card: #1e1e1e; 
        --text: #fff; 
        --accent: #00bcd4; 
        --green: #4caf50; 
        --red: #f44336; 
        --orange: #ff9800; 
        --purple: #9c27b0; 
        --yellow: #ffeb3b; 
        --pink: #e91e63; 
    }
    
    /* GŁÓWNE CIAŁO STRONY */
    body { 
        background-color: var(--bg); 
        color: var(--text); 
        font-family: 'Segoe UI', sans-serif; 
        margin: 0; 
        padding: 0; 
    }
    
    /* NAGŁÓWEK (Pasek na samej górze) */
    .header { 
        background: #000; 
        padding: 15px; 
        text-align: center; 
        border-bottom: 2px solid var(--accent); 
    }
    h1 { 
        margin: 0; 
        font-size: 22px; 
        color: var(--accent); 
    }
    
    /* STYLIZACJA TWOJEGO LOGO */
    .main-logo { 
        max-height: 70px; 
        margin-bottom: 10px; 
    }
    .ota-logo { 
        max-height: 100px; 
        margin-bottom: 20px; 
        border-radius: 10px; 
        box-shadow: 0 0 20px rgba(0, 188, 212, 0.4); 
    }

    /* PASEK NAWIGACJI (Zakładki) */
    .nav { 
        display: flex; 
        justify-content: space-around; 
        background: #222; 
        padding: 10px 0; 
        overflow-x: auto;
    }
    .nav button { 
        background: none; 
        border: none; 
        color: #aaa; 
        font-size: 14px; 
        font-weight: bold; 
        cursor: pointer; 
        padding: 10px; 
        white-space: nowrap; 
    }
    .nav button.active { 
        color: var(--accent); 
        border-bottom: 2px solid var(--accent); 
    }

    /* KONTENERY ZAKŁADEK (Ukrywanie i pokazywanie) */
    .tab-content { 
        display: none; 
        padding: 15px; 
        max-width: 500px; 
        margin: 0 auto; 
    }
    .tab-content.active { 
        display: block; 
    }

    /* KARTY (Pojedyncze bloki danych) */
    .card { 
        background: var(--card); 
        border-radius: 12px; 
        padding: 15px; 
        margin-bottom: 15px; 
        box-shadow: 0 4px 8px rgba(0,0,0,0.5); 
    }
    
    /* STYL OPISÓW POMOCNICZYCH (Nowość - intuicyjne opisy nastaw) */
    .help-text {
        font-size: 11px;
        color: #888;
        margin-top: 5px;
        margin-bottom: 10px;
        line-height: 1.3;
    }

    .row { 
        display: flex; 
        justify-content: space-between; 
        font-size: 16px; 
        padding: 10px 0; 
        border-bottom: 1px solid #333; 
    }
    .row:last-child { 
        border: none; 
    }
    .val { 
        font-weight: bold; 
        color: var(--green); 
    }

    /* PRZYCISKI KONTROLNE (Zasilanie, Tryb) */
    .ctrl-btn { 
        width: 48%; 
        padding: 15px; 
        font-size: 16px; 
        font-weight: bold; 
        border-radius: 8px; 
        border: none; 
        cursor: pointer; 
        color: #fff; 
        transition: 0.2s;
    }
    .btn-on { background: var(--green); }
    .btn-off { background: var(--red); }
    .btn-auto { background: var(--accent); color: #000; }
    .btn-man { background: #555; }
    
    /* PRZYCISK ECO CHMURY */
    .btn-eco { 
        background: #0288d1; 
        color: #fff; 
        width: 100%; 
        margin-top: 10px; 
        padding: 10px; 
        font-weight: bold; 
        border: none; 
        border-radius: 8px; 
        cursor: pointer;
    }
    .btn-eco-off { 
        background: #555; 
        color: #aaa; 
        width: 100%; 
        margin-top: 10px; 
        padding: 10px; 
        font-weight: bold; 
        border: none; 
        border-radius: 8px; 
        cursor: pointer;
    }

    /* FORMULARZE I INPUTY */
    label { 
        display: block; 
        margin-top: 10px; 
        font-size: 14px; 
        color: #aaa; 
    }
    input, select { 
        width: 100%; 
        padding: 10px; 
        margin-top: 5px; 
        background: #2a2a2a; 
        border: 1px solid #444; 
        color: #fff; 
        border-radius: 6px; 
        box-sizing: border-box; 
    }
    .submit-btn { 
        width: 100%; 
        padding: 15px; 
        margin-top: 15px; 
        background: var(--accent); 
        color: #000; 
        border: none; 
        font-weight: bold; 
        border-radius: 8px; 
        cursor: pointer; 
    }

    /* ELEMENTY LISTY SD */
    .file-item { 
        display: flex; 
        justify-content: space-between; 
        background: #2a2a2a; 
        padding: 10px; 
        margin-bottom: 5px; 
        border-radius: 6px; 
    }
    .file-item a { 
        color: var(--accent); 
        text-decoration: none; 
        font-weight: bold; 
    }

    /* ZNACZNIKI STATUSÓW */
    .badge-ok { 
        color: var(--green); 
        font-weight: bold; 
        text-shadow: 0 0 5px rgba(76, 175, 80, 0.5); 
    }
    .badge-err { 
        color: var(--red); 
        font-weight: bold; 
        text-shadow: 0 0 5px rgba(244, 67, 54, 0.5); 
    }
    
    /* OVERLAY DLA ZDALNEJ AKTUALIZACJI Z GITHUBA */
    #remote-ota-overlay {
        display: none; 
        position: fixed; 
        top: 0; 
        left: 0; 
        width: 100%; 
        height: 100%;
        background: rgba(0,0,0,0.95); 
        z-index: 9999; 
        flex-direction: column; 
        justify-content: center; 
        align-items: center; 
        text-align: center;
    }
    .ota-spinner { 
        font-size: 50px; 
        margin-bottom: 20px; 
        animation: spin 2s linear infinite; 
    }
    @keyframes spin { 
        100% { transform: rotate(360deg); } 
    }
  </style>
</head>
<body>

  <div id="remote-ota-overlay">
      <img src="https://github.com/KamilBol/Regulator-PID-VSC/blob/main/firmware/Logo/Logo%20Bia%C5%82y%20napis%20na%20czarnym%20tle%20mniejsze.jpg?raw=true" class="ota-logo" alt="Logo">
      <div class="ota-spinner">⚙️</div>
      <h2 style="color: var(--accent); font-size:24px;" id="rota-state">Inicjalizacja pobierania...</h2>
      <div style="width: 80%; background: #333; height: 30px; border-radius: 10px; margin-top: 20px; overflow: hidden; box-shadow: 0 0 15px rgba(0, 188, 212, 0.5);">
          <div id="rota-bar" style="width: 0%; height: 100%; background: var(--green); line-height: 30px; font-weight: bold; transition: width 0.3s;">0%</div>
      </div>
      <p style="color:#aaa; font-size:12px; margin-top:20px;">Nie wyłączaj zasilania maszyny!</p>
  </div>

  <div class="header">
      <img src="https://github.com/KamilBol/Regulator-PID-VSC/blob/main/firmware/Logo/Logo%20Bia%C5%82y%20napis%20na%20czarnym%20tle%20mniejsze.jpg?raw=true" class="main-logo" alt="Logo Bolu">
      <h1>⚙️ Granulator Pro V16.5</h1>
  </div>
  
  <div class="nav">
    <button class="tablinks active" onclick="openTab(event, 'Panel')">📊 Panel</button>
    <button class="tablinks" onclick="openTab(event, 'Sensory')">📡 Sensory</button>
    <button class="tablinks" onclick="openTab(event, 'Nastawy')">⚙️ Nastawy</button>
    <button class="tablinks" onclick="openTab(event, 'SD')">🩺 Diagnostyka</button>
    <button class="tablinks" onclick="openTab(event, 'OTA')">📥 Lokalne OTA</button>
  </div>

  <div id="Panel" class="tab-content active">
    <div class="card" style="display: flex; justify-content: space-between;">
      <button id="btnSys" class="ctrl-btn btn-off" onclick="toggleSys()">Zasilanie: OFF</button>
      <button id="btnMode" class="ctrl-btn btn-man" onclick="toggleMode()">Tryb: MAN</button>
    </div>
    
    <div class="card" style="border: 1px solid #0288d1;">
      <h3 style="margin-top:0; color:#0288d1; text-align:center; font-size:16px;">Telemetria Chmurowa</h3>
      <button id="btnEco" class="btn-eco-off" onclick="toggleEco()">Tryb MQTT: ŁADOWANIE</button>
      <p style="font-size:11px; color:#aaa; text-align:center; margin-top:10px;">ECO = Oszczędność danych transferu. MAX = Pełna analityka.</p>
    </div>

    <div class="card">
      <div class="row"><span>Prąd Maszyny:</span> <span class="val" id="amp">-- A</span></div>
      <div class="row"><span>Cel PID (Limit):</span> <span class="val" id="setp">-- A</span></div>
      <div class="row"><span>Awaria (Przeciążenie):</span> <span class="val" id="trip" style="color:var(--red);">NIE</span></div>
      <div class="row"><span>Wyjście Falownik 1:</span> <span class="val" id="dac" style="color:var(--orange);">-- V</span></div>
      <div class="row"><span>Wyjście Falownik 2:</span> <span class="val" id="dac2v" style="color:var(--purple);">-- V</span></div>
    </div>
    <button class="submit-btn" style="background:var(--red); color:#fff; font-size:16px;" onclick="restartESP()">🔄 RESTART MASZYNY</button>
  </div>

  <div id="Sensory" class="tab-content">
    <div class="card">
      <h3 style="margin-top:0; color:var(--accent);">Odczyt PZEM</h3>
      <p class="help-text">Główne parametry elektryczne pobierane bezpośrednio z układu pomiarowego.</p>
      <div class="row"><span>Napięcie Sieci:</span> <span class="val" id="volt">-- V</span></div>
      <div class="row"><span>Moc Czynna (P):</span> <span class="val" id="pow">-- W</span></div>
      <div class="row"><span>Moc Pozorna (S):</span> <span class="val" id="ap_pow">-- VA</span></div>
      <div class="row"><span>Moc Bierna (Q):</span> <span class="val" id="re_pow">-- Var</span></div>
      <div class="row"><span>Cosinus Fi (PF):</span> <span class="val" id="pf">--</span></div>
    </div>
    <div class="card">
      <h3 style="margin-top:0; color:var(--accent);">Warunki DHT</h3>
      <p class="help-text">Warunki klimatyczne panujące wewnątrz szafy sterowniczej.</p>
      <div class="row"><span>Temperatura:</span> <span class="val" id="temp">-- °C</span></div>
      <div class="row"><span>Wilgotność:</span> <span class="val" id="hum">-- %</span></div>
    </div>
  </div>

  <div id="Nastawy" class="tab-content">
    
    <div class="card">
        <h3 style="margin-top:0;">1. Widełki Pracy</h3>
        <p class="help-text">Zakres dopuszczalnego poboru prądu. Algorytm dąży do utrzymania prądu na poziomie zbliżonym do wartości "Max".</p>
        <form onsubmit="saveLimits(event)">
            <label>Min [A]</label><input type="number" step="0.1" id="minL" required>
            <label>Max [A]</label><input type="number" step="0.1" id="maxL" required>
            <button type="submit" class="submit-btn">ZAPISZ WIDEŁKI</button>
        </form>
    </div>
    
    <div class="card">
        <h3 style="margin-top:0; color:var(--orange);">2. Proporcje Falowników</h3>
        <p class="help-text">Określa procentowy podział mocy sterującej między falowniki. Pozwala na asymetryczną pracę silników.</p>
        <form onsubmit="saveRatios(event)">
            <label>DAC 1 [%]</label><input type="number" step="1" min="0" max="100" id="dac1r" required>
            <label>DAC 2 [%]</label><input type="number" step="1" min="0" max="100" id="dac2r" required>
            <button type="submit" class="submit-btn" style="background:var(--orange); color:#fff;">ZAPISZ PROPORCJE</button>
        </form>
    </div>
    
    <div class="card">
        <h3 style="margin-top:0;">3. Strojenie PID</h3>
        <p class="help-text">Parametry dynamiki algorytmu. P (Proporcjonalny) to szybkość reakcji, I (Całkujący) koryguje błędy stałe, D (Różniczkujący) tłumi gwałtowne skoki.</p>
        <form onsubmit="savePID(event)">
            <label>Współczynnik P</label><input type="number" step="0.01" id="kp" required>
            <label>Współczynnik I</label><input type="number" step="0.01" id="ki" required>
            <label>Współczynnik D</label><input type="number" step="0.01" id="kd" required>
            <button type="submit" class="submit-btn" style="background:#555; color:#fff;">ZAPISZ PID</button>
        </form>
    </div>
    
    <div class="card">
        <h3 style="margin-top:0; color:var(--green);">4. Alarmy i Zabezpieczenia</h3>
        <p class="help-text">O ile Amperów może zostać przekroczony prąd Max zanim system awaryjnie odetnie falowniki. "Wznowienie" to wartość poniżej limitu Min pozwalająca na ponowny start.</p>
        <form onsubmit="saveAlarms(event)">
            <label>Próg odcięcia awaryjnego [A]</label><input type="number" step="0.1" id="ovL" required>
            <label>Próg wznowienia [A]</label><input type="number" step="0.1" id="recL" required>
            <button type="submit" class="submit-btn" style="background:var(--green); color:#fff;">ZAPISZ ALARMY</button>
        </form>
    </div>
    
    <div class="card">
        <h3 style="margin-top:0; color:var(--pink);">5. Fizyczne Limity Napięcia</h3>
        <p class="help-text">Ograniczenie twarde napięcia na wyjściu DAC. Gwarantuje, że sterownik nie poda na falownik sygnału poza tym zakresem GDY.</p>
        <form onsubmit="saveVoltLimits(event)">
            <label>Podłoga (Min V)</label><input type="number" step="0.01" id="minV" required>
            <label>Sufit (Max V)</label><input type="number" step="0.01" id="maxV" required>
            <button type="submit" class="submit-btn" style="background:var(--pink); color:#fff;">ZAPISZ LIMITY</button>
        </form>
    </div>
    
    <div class="card">
        <h3 style="margin-top:0; color:var(--purple);">6. Profil Sygnału Sterującego</h3>
        <p class="help-text">Dopasowanie rodzaju sygnału wyjściowego do standardu wejścia analogowego zastosowanego w falowniku.</p>
        <form onsubmit="saveOutMode(event)">
            <select id="outMode">
                <option value="0">0-10V Napięciowy</option>
                <option value="1">0-20mA Prądowy</option>
                <option value="2">4-20mA Prądowy</option>
            </select>
            <button type="submit" class="submit-btn" style="background:var(--purple); color:#fff;">ZAPISZ PROFIL</button>
        </form>
    </div>
    
    <div class="card">
        <h3 style="margin-top:0; color:#00bcd4;">7. Kalibracja Sprzętowa (Offset)</h3>
        <p class="help-text">Pozwala na stałe dodanie lub odjęcie ułamków Volta w celu zniwelowania ewentualnych spadków napięcia na długich przewodach.</p>
        <form onsubmit="saveCalib(event)">
            <label>Korekta DAC 1 [V]</label><input type="number" step="0.01" id="dac1c" required>
            <label>Korekta DAC 2 [V]</label><input type="number" step="0.01" id="dac2c" required>
            <button type="submit" class="submit-btn" style="background:#00bcd4; color:#000;">ZAPISZ KALIBRACJĘ</button>
        </form>
    </div>
    
    <div class="card">
        <h3 style="margin-top:0; color:#4caf50;">8. Łączność WiFi</h3>
        <p class="help-text">Poświadczenia routera na hali. Jeśli połączenie się nie uda, maszyna wystawi własną sieć "RegulatorPID". Zapis restartuje urządzenie.</p>
        <form onsubmit="saveWiFi(event)">
            <label>SSID Sieci</label><input type="text" id="wifiSSID">
            <label>Hasło Sieci</label><input type="password" id="wifiPASS" placeholder="[Zapisane w pamięci]">
            <button type="submit" class="submit-btn" style="background:#4caf50; color:#fff;">ZAPISZ WIFI I RESTARTUJ</button>
        </form>
    </div>
    
    <div class="card">
        <h3 style="margin-top:0; color:#03a9f4;">9. Połączenie MQTT</h3>
        <p class="help-text">Konfiguracja chmury HiveMQ pozwalająca na komunikację maszyny z Serwerem HUB na biurku. Zapis restartuje urządzenie.</p>
        <form onsubmit="saveMQTT(event)">
            <label>Adres Brokera</label><input type="text" id="mqSrv">
            <label>Użytkownik</label><input type="text" id="mqUsr">
            <label>Hasło</label><input type="password" id="mqPas" placeholder="[Zapisane w pamięci]">
            <label>Unikalne ID Maszyny</label><input type="text" id="mqId">
            <button type="submit" class="submit-btn" style="background:#03a9f4; color:#fff;">ZAPISZ MQTT I RESTARTUJ</button>
        </form>
    </div>
    
    <div class="card">
        <h3 style="margin-top:0; color:var(--yellow);">10. Zarządzanie Pamięcią</h3>
        <p class="help-text">Możesz zabezpieczyć obecne, stabilne parametry i odzyskać je po nieudanym strojeniu lub przywrócić układ do ustawień fabrycznych.</p>
        <button onclick="saveDefaults()" class="submit-btn" style="background:var(--yellow); color:#000;">ZAPISZ AKTUALNE JAKO DOMYŚLNE</button>
        <button onclick="restoreDefaults()" class="submit-btn" style="background:var(--red); color:#fff; margin-top:10px;">PRZYWRÓĆ USTAWIENIA FABRYCZNE</button>
    </div>
  </div>

  <div id="SD" class="tab-content">
    <div class="card">
      <h3 style="margin-top:0; color:var(--yellow);">🧠 Parametry ESP32</h3>
      <div class="row"><span>Uptime:</span> <span class="val" id="esp_up" style="color:var(--text);">--</span></div>
      <div class="row"><span>Wolny RAM:</span> <span class="val" id="esp_ram" style="color:var(--text);">-- %</span></div>
      <div class="row"><span>CPU:</span> <span class="val" id="esp_cpu" style="color:var(--text);">--</span></div>
      <div class="row"><span>Chip:</span> <span class="val" id="esp_chip" style="color:var(--text);">--</span></div>
      <div class="row"><span>Flash:</span> <span class="val" id="esp_flash" style="color:var(--text);">-- KB</span></div>
      <div class="row"><span>LAN IP:</span> <span class="val" id="esp_rip" style="color:var(--text);">--</span></div>
    </div>
    <div class="card">
      <h3 style="margin-top:0; color:#00bcd4;">🩺 Status Sprzętu</h3>
      <div class="row"><span>PZEM-004T:</span> <span id="st_pzem" class="badge-err">ŁADOWANIE</span></div>
      <div class="row"><span>Nextion:</span> <span id="st_nex" class="badge-err">ŁADOWANIE</span></div>
      <div class="row"><span>ADS1115:</span> <span id="st_ads" class="badge-err">ŁADOWANIE</span></div>
      <div class="row"><span>GP8403:</span> <span id="st_dac" class="badge-err">ŁADOWANIE</span></div>
      <div class="row"><span>ISO1540:</span> <span id="st_iso" class="badge-err">ŁADOWANIE</span></div>
      <div class="row"><span>DHT11:</span> <span id="st_dht" class="badge-err">ŁADOWANIE</span></div>
      <div class="row"><span>Karta SD:</span> <span id="st_sd" class="badge-err">ŁADOWANIE</span></div>
    </div>
    <div class="card">
        <h3 style="margin-top:0;">Eksplorator Karty SD</h3>
        <p class="help-text">Wymaga podłączenia do lokalnej sieci WiFi maszyny, aby pobrać pliki tekstowe z logami systemu.</p>
        <button onclick="loadSD()" style="padding:10px; background:#444; color:#fff; border:none; width:100%; border-radius:5px;">Odśwież listę plików</button>
        <div id="sd-list" style="margin-top:10px;">Brak plików do wyświetlenia</div>
    </div>
  </div>

  <div id="OTA" class="tab-content">
    <div class="card">
      <h3 style="margin-top:0; color:var(--red);">Aktualizacja Lokalna (Sieć WiFi)</h3>
      <p class="help-text">Użyj tej opcji, jeśli z jakiegoś powodu aktualizacja z chmury (GitHuba) przez Serwer HUB zawiodła. Wybierz plik firmware.bin z dysku komputera.</p>
      <form method="POST" action="#" enctype="multipart/form-data" id="upload_form">
        <input type="file" name="update" id="file" accept=".bin" required style="padding: 10px 0;">
        <button type="submit" class="submit-btn" style="background:var(--red); color:#fff;">WGRAJ PLIK Z KOMPUTERA</button>
      </form>
      <div id="prog-container" style="width: 100%; background: #333; border-radius: 5px; display: none; margin-top: 15px;">
          <div id="prog-bar" style="width: 0%; height: 20px; background: var(--green); border-radius: 5px; text-align: center; color: white; line-height: 20px; font-size: 12px;">0%</div>
      </div>
      <p id="ota-status" style="margin-top:10px; font-weight:bold;"></p>
    </div>
  </div>

  <script>
    let lastFocusTime = 0;
    
    // Zabezpieczenie przed nadpisywaniem formularzy podczas wpisywania
    window.addEventListener('DOMContentLoaded', () => { 
        document.querySelectorAll('input, select').forEach(i => { 
            i.addEventListener('focus', () => { lastFocusTime = Date.now(); }); 
            i.addEventListener('input', () => { lastFocusTime = Date.now(); }); 
            i.addEventListener('blur', () => { lastFocusTime = Date.now(); }); 
        }); 
    });
    
    // Funkcja nawigacji po zakładkach
    function openTab(evt, tabName) { 
        document.querySelectorAll(".tab-content").forEach(el => el.style.display = "none"); 
        document.querySelectorAll(".tablinks").forEach(el => el.classList.remove("active")); 
        document.getElementById(tabName).style.display = "block"; 
        evt.currentTarget.classList.add("active"); 
    }

    // ==========================================
    // PĘTLA 1: STATUS ZDALNEJ AKTUALIZACJI OTA (Co 1s)
    // ==========================================
    setInterval(function() {
      fetch('/api/ota_status').then(res => res.json()).then(data => {
        let overlay = document.getElementById('remote-ota-overlay');
        if(data.progress >= 0) {
            overlay.style.display = "flex";
            document.getElementById('rota-state').innerText = data.state;
            document.getElementById('rota-bar').style.width = data.progress + '%';
            document.getElementById('rota-bar').innerText = data.progress + '%';
            if (data.progress === 100) {
                setTimeout(() => location.reload(), 6000);
            }
        } else {
            overlay.style.display = "none";
        }
      }).catch(e => {}); 
    }, 1000);

    // ==========================================
    // PĘTLA 2: GŁÓWNA TELEMETRIA (Co 1s)
    // ==========================================
    setInterval(function() {
      fetch('/api/data').then(res => res.json()).then(data => {
        document.getElementById('amp').innerText = data.amp + " A";
        document.getElementById('setp').innerText = data.setp + " A";
        document.getElementById('dac').innerText = data.dac + " V";
        document.getElementById('dac2v').innerText = data.dac2v + " V";
        document.getElementById('trip').innerText = data.trip == "1" ? "TAK" : "NIE";
        
        let btnSys = document.getElementById('btnSys');
        if(data.sysON == "1") { btnSys.className = "ctrl-btn btn-on"; btnSys.innerText = "Zasilanie: ON"; } else { btnSys.className = "ctrl-btn btn-off"; btnSys.innerText = "Zasilanie: OFF"; }
        
        let btnMode = document.getElementById('btnMode');
        if(data.autoM == "1") { btnMode.className = "ctrl-btn btn-auto"; btnMode.innerText = "Tryb: AUTO"; } else { btnMode.className = "ctrl-btn btn-man"; btnMode.innerText = "Tryb: MAN"; }

        let btnEco = document.getElementById('btnEco');
        if(data.eco == "1") { 
            btnEco.style.background = "var(--green)"; btnEco.style.color = "#fff"; btnEco.innerText = "Tryb MQTT: ECO"; 
        } else { 
            btnEco.style.background = "var(--orange)"; btnEco.style.color = "#fff"; btnEco.innerText = "Tryb MQTT: MAX"; 
        }

        document.getElementById('volt').innerText = data.volt + " V";
        document.getElementById('pow').innerText = data.pow + " W";
        document.getElementById('ap_pow').innerText = data.ap_pow + " VA";
        document.getElementById('re_pow').innerText = data.re_pow + " Var";
        document.getElementById('pf').innerText = data.pf;
        document.getElementById('temp').innerText = data.temp + " °C";
        document.getElementById('hum').innerText = data.hum + " %";

        // Aktualizowanie pól inputów tylko gdy użytkownik ich nie edytuje
        if (Date.now() - lastFocusTime > 10000) {
            document.getElementById('outMode').value = data.outM; document.getElementById('dac1r').value = data.dac1R; document.getElementById('dac2r').value = data.dac2R;
            document.getElementById('ovL').value = data.ovL; document.getElementById('recL').value = data.recL; document.getElementById('minL').value = data.minL;
            document.getElementById('maxL').value = data.maxL; document.getElementById('kp').value = data.kp; document.getElementById('ki').value = data.ki;
            document.getElementById('kd').value = data.kd; document.getElementById('dac1c').value = data.dac1C; document.getElementById('dac2c').value = data.dac2C;
            document.getElementById('minV').value = data.minV; document.getElementById('maxV').value = data.maxV; document.getElementById('wifiSSID').value = data.wifi_s;
            document.getElementById('mqSrv').value = data.mq_srv; document.getElementById('mqUsr').value = data.mq_usr; document.getElementById('mqId').value = data.mq_id;
        }
      });
    }, 1000);

    // ==========================================
    // PĘTLA 3: ZDROWIE SYSTEMU (Co 2s)
    // ==========================================
    setInterval(function() {
      fetch('/api/health').then(res => res.json()).then(data => {
        function setSt(id, st, failText) { let el = document.getElementById(id); if (st === "1") { el.innerText = "ONLINE"; el.className = "badge-ok"; } else { el.innerText = failText; el.className = "badge-err"; } }
        setSt('st_pzem', data.pzem, "BŁĄD"); setSt('st_nex', data.nex, "BŁĄD"); setSt('st_ads', data.ads, "BŁĄD"); setSt('st_dac', data.dac, "BŁĄD"); setSt('st_dht', data.dht, "BŁĄD"); setSt('st_sd', data.sd, "BŁĄD");
        let el_iso = document.getElementById('st_iso'); if (data.dac === "1" || data.ads === "1") { el_iso.innerText = "ONLINE"; el_iso.className = "badge-ok"; } else { el_iso.innerText = "BŁĄD"; el_iso.className = "badge-err"; }
        if(document.getElementById('esp_up')) document.getElementById('esp_up').innerText = data.up; if(document.getElementById('esp_ram')) document.getElementById('esp_ram').innerText = data.heap_pct + " %";
        if(document.getElementById('esp_cpu')) document.getElementById('esp_cpu').innerText = data.cpu; if(document.getElementById('esp_chip')) document.getElementById('esp_chip').innerText = data.chip;
        if(document.getElementById('esp_flash')) document.getElementById('esp_flash').innerText = data.sketch; if(document.getElementById('esp_rip')) document.getElementById('esp_rip').innerText = data.router_ip;
      });
    }, 2000);

    // ==========================================
    // API POST (Wysyłanie danych z formularzy na ESP32)
    // ==========================================
    function toggleSys() { fetch('/api/toggle_sys', {method: 'POST'}); }
    function toggleMode() { fetch('/api/toggle_mode', {method: 'POST'}); }
    function toggleEco() { fetch('/api/toggle_eco', {method: 'POST'}); }
    function saveOutMode(e) { e.preventDefault(); fetch('/api/set_outmode?m='+document.getElementById('outMode').value, {method: 'POST'}).then(() => alert("Profil zapisany!")); }
    function saveRatios(e) { e.preventDefault(); fetch('/api/set_ratios?r1='+document.getElementById('dac1r').value+'&r2='+document.getElementById('dac2r').value, {method: 'POST'}).then(() => alert("Zapisano!")); }
    function saveAlarms(e) { e.preventDefault(); fetch('/api/set_alarms?ov='+document.getElementById('ovL').value+'&rec='+document.getElementById('recL').value, {method: 'POST'}).then(() => alert("Zapisano!")); }
    function saveLimits(e) { e.preventDefault(); fetch('/api/set_limits?min='+document.getElementById('minL').value+'&max='+document.getElementById('maxL').value, {method: 'POST'}).then(() => alert("Zapisano!")); }
    function savePID(e) { e.preventDefault(); fetch('/api/set_pid?kp='+document.getElementById('kp').value+'&ki='+document.getElementById('ki').value+'&kd='+document.getElementById('kd').value, {method: 'POST'}).then(() => alert("Zapisano!")); }
    function saveCalib(e) { e.preventDefault(); fetch('/api/set_calib?c1='+document.getElementById('dac1c').value+'&c2='+document.getElementById('dac2c').value, {method: 'POST'}).then(() => alert("Zapisano!")); }
    function saveVoltLimits(e) { e.preventDefault(); fetch('/api/set_volt_limits?min='+document.getElementById('minV').value+'&max='+document.getElementById('maxV').value, {method: 'POST'}).then(() => alert("Zapisano!")); }
    function saveWiFi(e) { e.preventDefault(); fetch('/api/set_wifi?s='+encodeURIComponent(document.getElementById('wifiSSID').value)+'&p='+encodeURIComponent(document.getElementById('wifiPASS').value), {method: 'POST'}).then(() => { alert("Zapisano. Maszyna uruchomi się ponownie."); setTimeout(() => location.reload(), 8000); }); }
    function saveMQTT(e) { e.preventDefault(); fetch('/api/set_mqtt?srv='+encodeURIComponent(document.getElementById('mqSrv').value)+'&usr='+encodeURIComponent(document.getElementById('mqUsr').value)+'&pas='+encodeURIComponent(document.getElementById('mqPas').value)+'&id='+encodeURIComponent(document.getElementById('mqId').value), {method: 'POST'}).then(() => { alert("Zapisano konfigurację chmury. Restart..."); setTimeout(() => location.reload(), 8000); }); }
    function restartESP() { if(confirm("Na pewno chcesz zrestartować układ sterujący maszyny?")) fetch('/api/restart', {method: 'POST'}).then(() => setTimeout(() => location.reload(), 10000)); }
    function saveDefaults() { if(confirm("Czy na pewno chcesz nadpisać wartości domyślne obecnymi?")) fetch('/api/save_defaults', {method: 'POST'}).then(() => alert("Zapisano w pamięci trwałej!")); }
    function restoreDefaults() { if(confirm("UWAGA! Ta operacja zresetuje maszynę do ustawień domyślnych. Kontynuować?")) fetch('/api/restore_defaults', {method: 'POST'}).then(() => setTimeout(() => location.reload(), 8000)); }
    
    // Generowanie listy plików SD
    function loadSD() { 
        document.getElementById('sd-list').innerHTML = "Odpytywanie karty pamięci..."; 
        fetch('/api/sd_list').then(r => r.json()).then(d => { 
            let h = ""; 
            d.forEach(f => { h += `<div class='file-item'><a href='/sd_read?f=${f.name}' target='_blank'>📄 ${f.name}</a><span>${f.size} KB</span></div>`; }); 
            document.getElementById('sd-list').innerHTML = h || "Brak logów tekstowych do wyświetlenia."; 
        }); 
    }
    
    // ==========================================
    // LOKALNE WGRYWANIE OTA Z PLIKU KOMPUTERA
    // ==========================================
    document.getElementById('upload_form').addEventListener('submit', function(e) { 
        e.preventDefault(); 
        var f = document.getElementById('file').files[0]; 
        if(!f) return; 
        var d = new FormData(); 
        d.append('update', f, f.name); 
        document.getElementById('prog-container').style.display = 'block'; 
        document.getElementById('ota-status').innerText = "Trwa przesyłanie pliku..."; 
        var x = new XMLHttpRequest(); 
        x.open('POST', '/update', true); 
        x.upload.addEventListener('progress', function(e) { 
            if(e.lengthComputable) { 
                var p = Math.round((e.loaded/e.total)*100); 
                document.getElementById('prog-bar').style.width = p+'%'; 
                document.getElementById('prog-bar').innerText = p+'%'; 
            } 
        }); 
        x.onload = function() { 
            document.getElementById('ota-status').innerText = x.status==200 ? "Zakończono sukcesem! Następuje restart maszyny..." : "Wystąpił błąd podczas wgrywania!"; 
            if(x.status==200) setTimeout(()=>location.reload(), 5000); 
        }; 
        x.send(d); 
    });
  </script>
</body>
</html>
)rawliteral";

#endif