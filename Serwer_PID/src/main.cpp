// =====================================================================================
// SERWER DYSPOZYTORSKI (MULTI-HUB) - V2.5 (LOGO INTEGRATION, UX DESCRIPTIONS, FULL COMMENTS)
// =====================================================================================
#include <Arduino.h>

// --- BIBLIOTEKI SIECIOWE I SYSTEMOWE ---
#include <WiFi.h>               // Obsługa rdzenia WiFi
#include <WiFiClientSecure.h>   // Bezpieczny klient (wymagany dla MQTT przez SSL)
#include <ESPmDNS.h>            // Przyjazne adresy w sieci lokalnej (np. granulator-serwer.local)
#include <WebServer.h>          // Lokalny serwer strony WWW
#include <Preferences.h>        // Trwała pamięć Flash do zapisu ustawień (SSID, Hasła)
#include <PubSubClient.h>       // Klient protokołu MQTT (HiveMQ)

// =====================================================================================
// OBIEKTY GLOBALNE I KONFIGURACJA
// =====================================================================================
WebServer server(80); 
WiFiClientSecure espClient; 
PubSubClient mqtt(espClient);
Preferences memory;

#define PIN_LED 2 // Dioda LED do sygnalizacji pracy procesora

// --- DANE LOGOWANIA DO SIECI I CHMURY ---
String routerSSID = "";
String routerPASS = "";
String mqtt_server = "";
String mqtt_user = "";
String mqtt_pass = "";

unsigned long lastMqttReconnect = 0; // Timer do odliczania prób połączenia z chmurą

// --- STRUKTURA BAZY DANYCH FLOTY MASZYN ---
// Serwer HUB potrafi obsłużyć do 10 maszyn jednocześnie
#define MAX_MACHINES 10

struct Machine {
    String id;               // Unikalne ID maszyny (np. Granulator_01)
    String json;             // Ostatni odebrany pakiet danych (Thick JSON lub Thin JSON z danymi)
    String sd_json;          // Ostatnia odebrana lista plików z karty SD
    unsigned long lastSeen;  // Znacznik czasu ostatniego sygnału (do wykrywania stanu OFFLINE)
};

Machine machines[MAX_MACHINES]; // Tablica przechowująca wszystkie podłączone maszyny

// =====================================================================================
// GŁÓWNY KOD STRONY INTERNETOWEJ (HTML + CSS + JAVASCRIPT) DLA SERWERA
// Przechowywany w pamięci PROGMEM, serwowany prosto do przeglądarki.
// =====================================================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pl">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>HUB Dowodzenia - Flota</title>
    
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
        body { 
            background-color: var(--bg); 
            color: var(--text); 
            font-family: 'Segoe UI', sans-serif; 
            margin: 0; 
            padding: 0; 
        }
        
        /* GŁÓWNY NAGŁÓWEK Z LOGO */
        .header { 
            background: #000; 
            padding: 15px; 
            text-align: center; 
            border-bottom: 2px solid var(--accent); 
            position: relative;
        }
        h1 { 
            margin: 0; 
            font-size: 22px; 
            color: var(--accent); 
        }
        .main-logo { 
            max-height: 80px; 
            margin-bottom: 10px; 
            border-radius: 8px;
        }

        /* PASEK ZAKŁADEK */
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
        
        .container { 
            padding: 15px; 
            max-width: 600px; 
            margin: 0 auto; 
        }
        .tab-content { 
            display: none; 
        }
        .tab-content.active { 
            display: block; 
        }
        
        /* WYGLĄD POJEDYNCZYCH KART I WIERSZY DANYCH */
        .card { 
            background: var(--card); 
            border-radius: 12px; 
            padding: 15px; 
            margin-bottom: 15px; 
            box-shadow: 0 4px 8px rgba(0,0,0,0.5); 
        }
        
        /* STYL OPISÓW POMOCNICZYCH UX */
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
        
        /* PRZYCISKI STERUJĄCE */
        .ctrl-btn { 
            width: 48%; 
            padding: 15px; 
            font-size: 16px; 
            font-weight: bold; 
            border-radius: 8px; 
            border: none; 
            cursor: pointer; 
            color: #fff; 
            text-align: center; 
            transition: 0.2s;
        }
        .btn-on { background: var(--green); }
        .btn-off { background: var(--red); }
        .btn-auto { background: var(--accent); color: #000; }
        .btn-man { background: #555; }
        
        /* PRZYCISKI TRYBU ECO / MAX */
        .btn-eco { 
            background: var(--green); 
            color: #fff; 
            width: 100%; 
            margin-top: 10px; 
            padding: 15px; 
            font-weight: bold; 
            border: none; 
            border-radius: 8px; 
            cursor: pointer; 
            transition: 0.3s;
        }
        .btn-max { 
            background: var(--orange); 
            color: #fff; 
            width: 100%; 
            margin-top: 10px; 
            padding: 15px; 
            font-weight: bold; 
            border: none; 
            border-radius: 8px; 
            cursor: pointer; 
            transition: 0.3s;
        }
        
        /* FORMULARZE */
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
        
        /* WYGLĄD PRZYCISKU MASZYNY W GŁÓWNYM MENU */
        .machine-btn { 
            display: block; 
            width: 100%; 
            background: #222; 
            border: 2px solid #444; 
            padding: 20px; 
            border-radius: 10px; 
            margin-bottom: 15px; 
            cursor: pointer; 
            text-align: left; 
            transition: 0.3s;
        }
        .machine-btn:hover { 
            border-color: var(--accent); 
            background: #2a2a2a;
        }
        .m-title { 
            font-size: 20px; 
            font-weight: bold; 
            color: var(--accent); 
            margin-bottom: 5px;
        }
        .m-stat { 
            font-size: 14px; 
            color: #888; 
            display: flex; 
            justify-content: space-between;
        }
        
        /* KROPKI STATUSU ONLINE/OFFLINE */
        .dot { 
            height: 12px; 
            width: 12px; 
            background-color: var(--red); 
            border-radius: 50%; 
            display: inline-block;
        }
        .dot.ok { 
            background-color: var(--green); 
            box-shadow: 0 0 8px var(--green);
        }
            /* Animacja pulsowania dla panelu nagrywania */
        @keyframes pulse { 0% { opacity: 1; } 50% { opacity: 0.5; } 100% { opacity: 1; } }
        .badge-ok { color: var(--green); font-weight: bold; text-shadow: 0 0 5px rgba(76, 175, 80, 0.5); }
        .badge-err { color: var(--red); font-weight: bold; text-shadow: 0 0 5px rgba(244, 67, 54, 0.5); }
        
        #btn-back { 
            position: absolute; 
            left: 15px; 
            top: 15px; 
            background: none; 
            border: 1px solid var(--accent); 
            color: var(--accent); 
            padding: 5px 15px; 
            border-radius: 5px; 
            cursor: pointer; 
            display: none;
        }
        
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
    </style>
</head>
<body>
    
    <div class="header">
        <button id="btn-back" onclick="showDashboard()">🔙 Wróć do Floty</button>
        <img src="https://github.com/KamilBol/Regulator-PID-VSC/blob/main/firmware/Logo/Logo%20Bia%C5%82y%20napis%20na%20czarnym%20tle%20mniejsze.jpg?raw=true" class="main-logo" alt="Logo Bolu">
        <h1>📡 HUB DOWODZENIA</h1>
        <div style="margin-top:5px; font-size:12px; color:#888;">
            IP Serwera HUB: <span id="srvIP">--</span> | <span id="mqStat">Szukam chmury...</span>
        </div>
    </div>

    <div id="view-dashboard" class="container" style="display:block;">
        <h3 style="color:#aaa; border-bottom:1px solid #333; padding-bottom:10px;">Dostępne Maszyny (Live):</h3>
        
        <div id="machine-list">Ładowanie sygnału z chmury...</div>
        
        <div class="card" style="margin-top: 40px; border: 1px solid #555;">
            <h3 style="margin-top:0; color:var(--yellow);">⚙️ Konfiguracja Sieciowa Serwera HUB</h3>
            <p class="help-text">Wprowadź dane sieci WiFi hali oraz dane logowania do brokera MQTT, aby centrala mogła komunikować się z flotą maszyn.</p>
            <form onsubmit="saveConfig(event)">
                <label>WiFi SSID</label>
                <input type="text" id="c_ssid">
                <label>WiFi Hasło</label>
                <input type="password" id="c_pass" placeholder="[Zapisane w pamięci]">
                <label>MQTT Adres Brokera (np. url.hivemq.cloud)</label>
                <input type="text" id="c_msrv">
                <label>MQTT Użytkownik</label>
                <input type="text" id="c_musr">
                <label>MQTT Hasło</label>
                <input type="password" id="c_mpas" placeholder="[Zapisane w pamięci]">
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
            <button class="tablinks" onclick="openTab(event, 'OTA')">📥 OTA (Zdalne)</button>
        </div>

        <div class="container">
            <h2 style="text-align:center; color:var(--text); margin-top:0; border-bottom:1px dashed #444; padding-bottom:10px;">
                Maszyna: <span id="active-m-id" style="color:var(--accent);">---</span>
            </h2>
            
            <div id="Panel" class="tab-content active">
                <div class="card" style="display: flex; justify-content: space-between;">
                    <div id="btnSys" class="ctrl-btn btn-off" onclick="toggleSys()">Zasilanie: OFF</div>
                    <div id="btnMode" class="ctrl-btn btn-man" onclick="toggleMode()">Tryb: MAN</div>
                </div>
                
                <div class="card" style="border: 2px solid #0288d1; text-align:center;">
                    <h3 style="margin-top:0; color:#0288d1; font-size:16px;">Telemetria Chmurowa</h3>
                    <button id="btnEco" class="btn-max" onclick="toggleEco()">Tryb MQTT: ŁADOWANIE</button>
                    <p style="font-size:11px; color:#aaa; margin-top:10px;">Oszczędzaj zużycie limitów transferu danych na serwerze HiveMQ przełączając wybraną maszynę w tryb ECO.</p>
                </div>

                <div class="card">
                    <div class="row"><span>Prąd Maszyny:</span> <span class="val" id="amp">-- A</span></div>
                    <div class="row"><span>Cel PID (Limit):</span> <span class="val" id="setp">-- A</span></div>
                    <div class="row"><span>Awaria (Przeciążenie):</span> <span class="val" id="trip" style="color:var(--red);">NIE</span></div>
                    <div class="row"><span>Wyjście na Falownik 1:</span> <span class="val" id="dac" style="color:var(--orange);">-- V</span></div>
                    <div class="row"><span>Wyjście na Falownik 2:</span> <span class="val" id="dac2v" style="color:var(--purple);">-- V</span></div>
                </div>
                <button class="submit-btn" style="background:var(--red); color:#fff;" onclick="if(confirm('Zrestartować zdalnie wybraną maszynę?')) sendCmd('CMD:RESTART')">🔄 ZDALNY RESTART MASZYNY (ESP32)</button>
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
                    <p class="help-text">Warunki klimatyczne panujące wewnątrz szafy sterowniczej na hali.</p>
                    <div class="row"><span>Temperatura:</span> <span class="val" id="temp">-- °C</span></div>
                    <div class="row"><span>Wilgotność:</span> <span class="val" id="hum">-- %</span></div>
                </div>
            </div>

            <div id="Nastawy" class="tab-content">
                <p style="color:var(--orange); font-size:12px; text-align:center;">Wprowadzane tu zmiany zostaną bezzwłocznie przesłane i wdrożone w wybranej maszynie przez chmurę MQTT.</p>
                
                <div class="card">
                    <h3 style="margin-top:0;">3. Zaawansowane Strojenie PID</h3>
                    <p class="help-text" style="margin-top:0; margin-bottom:15px;">Zdalna konfiguracja algorytmu oraz ustawienia zapobiegające szarpaniu maszyny.</p>
                    <form onsubmit="cmdPID(event)">
                        
                        <label style="color:var(--accent); margin-top:0; margin-bottom:2px;">Współczynnik P (Kp) - "Siła Hamulca"</label>
                        <p class="help-text" style="margin-top:0; margin-bottom:5px;">Mała wartość: zwalnia delikatnie. Duża wartość: gwałtownie ucina zasilanie na falowniku, ale może szarpać maszyną.</p>
                        <input type="number" step="0.01" id="kp" style="margin-top:0; margin-bottom:15px;" required>
                        
                        <label style="color:var(--accent); margin-top:0; margin-bottom:2px;">Współczynnik I (Ki) - "Cierpliwość / Dociskanie"</label>
                        <p class="help-text" style="margin-top:0; margin-bottom:5px;">Dociska hamulec w czasie, gdy prąd stale jest za wysoki. Zbyt duża wartość sprawi, że maszyna "przedobrzy" i udusi obroty.</p>
                        <input type="number" step="0.01" id="ki" style="margin-top:0; margin-bottom:15px;" required>
                        
                        <label style="color:var(--accent); margin-top:0; margin-bottom:2px;">Współczynnik D (Kd) - "Amortyzator"</label>
                        <p class="help-text" style="margin-top:0; margin-bottom:5px;">Pomaga płynnie "wyjść z zakrętu", zapobiegając ciągłemu falowaniu. Z reguły bardzo blisko zera (np. 0.05).</p>
                        <input type="number" step="0.01" id="kd" style="margin-top:0; margin-bottom:5px;" required>
                        
                        <hr style="border: 0; border-top: 1px solid #444; margin: 15px 0;">
                        
                        <label style="color:var(--accent); margin-top:0; margin-bottom:2px;">Czas dojazdu materiału do noży [s]</label>
                        <p class="help-text" style="margin-top:0; margin-bottom:5px;">Maszyna odczeka ten czas po każdej zmianie obrotów podajnika. Wpisz "0", aby reagowała natychmiast.</p>
                        <input type="number" step="0.1" min="0.0" max="60.0" id="delayTime" style="margin-top:0; margin-bottom:15px;" required>
                        
                        <label style="color:var(--accent); margin-top:0; margin-bottom:2px;">Tolerancja wahań prądu [A]</label>
                        <p class="help-text" style="margin-top:0; margin-bottom:5px;">Zapas błędu. Jeśli wpiszesz 1.0A, prąd skaczący +/- 1A wokół celu zostanie zignorowany. Wpisz "0", by reagować na każdy ułamek Ampera.</p>
                        <input type="number" step="0.1" min="0.0" max="10.0" id="deadBand" style="margin-top:0; margin-bottom:15px;" required>

                        <button type="submit" class="submit-btn" style="background:#555; color:#fff; margin-top:0;">WYŚLIJ USTAWIENIA PID W CHMURĘ</button>
                    </form>
                </div>
                
                <div class="card">
                    <h3 style="margin-top:0; color:var(--orange);">2. Proporcje Falowników</h3>
                    <p class="help-text">Określa procentowy podział wypracowanej mocy sterującej między falowniki. Pozwala na zbalansowaną asymetrię silników.</p>
                    <form onsubmit="cmdRatios(event)">
                        <label>Współczynnik DAC 1 [%]</label><input type="number" step="1" id="dac1r" required>
                        <label>Współczynnik DAC 2 [%]</label><input type="number" step="1" id="dac2r" required>
                        <button type="submit" class="submit-btn" style="background:var(--orange); color:#fff;">WYŚLIJ PROPORCJE DO MASZYNY</button>
                    </form>
                </div>
                
                <div class="card">
            <h3 style="margin-top:0;">3. Zaawansowane Strojenie PID</h3>
            <p class="help-text" style="margin-top:0; margin-bottom:15px;">Główne parametry algorytmu (P, I, D) oraz ustawienia zapobiegające "panikowaniu" i szarpaniu maszyny.</p>
            <form onsubmit="savePID(event)">
                
                <label style="color:var(--accent); margin-top:0; margin-bottom:2px;">Współczynnik P (Kp) - "Siła Hamulca"</label>
                <p class="help-text" style="margin-top:0; margin-bottom:5px;">Jak agresywnie system reaguje na rosnący prąd granulatora. Mała wartość: zwalnia delikatnie, ryzyko zapchania. Duża wartość: gwałtownie ucina zasilanie na falowniku, ale może szarpać maszyną.</p>
                <input type="number" step="0.01" id="kp" style="margin-top:0; margin-bottom:15px;" required>
                
                <label style="color:var(--accent); margin-top:0; margin-bottom:2px;">Współczynnik I (Ki) - "Cierpliwość / Dociskanie"</label>
                <p class="help-text" style="margin-top:0; margin-bottom:5px;">Jeśli prąd od dłuższego czasu wciąż jest za wysoki, ten parametr z każdą sekundą coraz mocniej "dociska" hamulec aż do skutku. Zbyt duża wartość sprawi, że maszyna "przedobrzy" i udusi obroty na za długo.</p>
                <input type="number" step="0.01" id="ki" style="margin-top:0; margin-bottom:15px;" required>
                
                <label style="color:var(--accent); margin-top:0; margin-bottom:2px;">Współczynnik D (Kd) - "Amortyzator"</label>
                <p class="help-text" style="margin-top:0; margin-bottom:5px;">Tłumi zapędy dwóch powyższych parametrów. Pomaga płynnie "wyjść z zakrętu", zapobiegając ciągłemu falowaniu obrotów góra-dół. Przy tym młynie z reguły ustawia się to bardzo blisko zera (np. 0.05).</p>
                <input type="number" step="0.01" id="kd" style="margin-top:0; margin-bottom:5px;" required>
                
                <hr style="border: 0; border-top: 1px solid #444; margin: 15px 0;">
                
                <label style="color:var(--accent); margin-top:0; margin-bottom:2px;">Czas dojazdu materiału do noży [s]</label>
                <p class="help-text" style="margin-top:0; margin-bottom:5px;">Ile sekund mija, zanim zmiana obrotów podajnika wpłynie na prąd granulatora? Maszyna odczeka ten czas po każdej zmianie, żeby uniknąć przeregulowania. Wpisz "0", aby system reagował natychmiast, bez czekania.</p>
                <input type="number" step="0.1" min="0.0" max="60.0" id="delayTime" style="margin-top:0; margin-bottom:15px;" required>
                
                <label style="color:var(--accent); margin-top:0; margin-bottom:2px;">Tolerancja wahań prądu [A]</label>
                <p class="help-text" style="margin-top:0; margin-bottom:5px;">Zapas błędu. Jeśli cel to 38A, a wpiszesz tu 1.0A, to prąd skaczący między 37A a 39A zostanie zignorowany. Zapobiega to ciągłemu szarpaniu falownikiem. Wpisz "0", aby układ reagował na każdy ułamek Ampera.</p>
                <input type="number" step="0.1" min="0.0" max="10.0" id="deadBand" style="margin-top:0; margin-bottom:15px;" required>

                <button type="submit" class="submit-btn" style="background:#555; color:#fff; margin-top:0;">ZAPISZ USTAWIENIA PID</button>
            </form>
        </div>
                
                <div class="card">
                    <h3 style="margin-top:0; color:var(--green);">4. Ustawienia Bezpieczeństwa</h3>
                    <p class="help-text">Ile Amperów ponad Limit Max zatrzyma natychmiast pracę falowników (Odcięcie). System wznowi pracę automatycznie, gdy prąd spadnie poniżej Limitu Min (Wznowienie).</p>
                    <form onsubmit="cmdAlarms(event)">
                        <label>Odcięcie Awaryjne [A]</label><input type="number" step="0.1" id="ovL" required>
                        <label>Wznowienie Pracy [A]</label><input type="number" step="0.1" id="recL" required>
                        <button type="submit" class="submit-btn" style="background:var(--green); color:#fff;">WYŚLIJ ALARMY DO MASZYNY</button>
                    </form>
                </div>
                
                <div class="card">
                    <h3 style="margin-top:0; color:var(--pink);">5. Ochrona Falownika (Limity V)</h3>
                    <p class="help-text">Twarde, sprzętowe limity napięcia wyjściowego. Zabezpieczają wejścia analogowe falowników przed podaniem sygnału poza ten zakres.</p>
                    <form onsubmit="cmdVoltLimits(event)">
                        <label>Podłoga [Min V]</label><input type="number" step="0.01" id="minV" required>
                        <label>Sufit [Max V]</label><input type="number" step="0.01" id="maxV" required>
                        <button type="submit" class="submit-btn" style="background:var(--pink); color:#fff;">WYŚLIJ LIMITY NAPIĘCIA DO MASZYNY</button>
                    </form>
                </div>
                
                <div class="card">
                    <h3 style="margin-top:0; color:var(--purple);">6. Konfiguracja Sygnału Sterującego</h3>
                    <p class="help-text">Wybierz standard obsługiwany przez wejście analogowe w podłączonym falowniku maszyny.</p>
                    <form onsubmit="cmdOutMode(event)">
                        <select id="outMode">
                            <option value="0">0-10V Napięciowy</option>
                            <option value="1">0-20mA Prądowy</option>
                            <option value="2">4-20mA Prądowy</option>
                        </select>
                        <button type="submit" class="submit-btn" style="background:var(--purple); color:#fff;">WYŚLIJ PROFIL DO MASZYNY</button>
                    </form>
                </div>
                
                <div class="card">
                    <h3 style="margin-top:0; color:#00bcd4;">7. Kalibracja Sprzętowa DAC</h3>
                    <p class="help-text">Wprowadź poprawki w Voltach (np. 0.15), aby zniwelować spadki napięcia na przewodach prowadzących do szafy falowników.</p>
                    <form onsubmit="cmdCalib(event)">
                        <label>Korekta Offset DAC 1 [V]</label><input type="number" step="0.01" id="dac1c" required>
                        <label>Korekta Offset DAC 2 [V]</label><input type="number" step="0.01" id="dac2c" required>
                        <button type="submit" class="submit-btn" style="background:#00bcd4; color:#000;">WYŚLIJ KALIBRACJĘ DO MASZYNY</button>
                    </form>
                </div>
                
                <div class="card">
                    <h3 style="margin-top:0; color:#4caf50;">8. Zdalna Zmiana WiFi Maszyny</h3>
                    <p class="help-text">Zmiana loginu do punktu dostępowego WiFi na hali. Zapis spowoduje automatyczny restart układu wykonawczego ESP32.</p>
                    <form onsubmit="cmdWiFi(event)">
                        <label>SSID Nowej Sieci</label><input type="text" id="m_wifiSSID">
                        <label>Hasło Nowej Sieci</label><input type="password" id="m_wifiPASS" placeholder="[Zapisane w pamięci maszyny]">
                        <button type="submit" class="submit-btn" style="background:#4caf50; color:#fff;">ZAPISZ I RESTARTUJ MASZYNĘ</button>
                    </form>
                </div>
                
                <div class="card">
                    <h3 style="margin-top:0; color:#03a9f4;">9. Zdalna Zmiana Kanału MQTT Maszyny</h3>
                    <p class="help-text">Modyfikacja ścieżki i nazwy ID raportującej maszyny do HiveMQ. Zapis spowoduje przerwę w transmisji z powodu restartu.</p>
                    <form onsubmit="cmdMQTT(event)">
                        <label>Broker sieci</label><input type="text" id="m_mqSrv">
                        <label>User autoryzacji</label><input type="text" id="m_mqUsr">
                        <label>Hasło autoryzacji</label><input type="password" id="m_mqPas" placeholder="[Zapisane w pamięci maszyny]">
                        <label>Identyfikator Maszyny</label><input type="text" id="m_mqId">
                        <button type="submit" class="submit-btn" style="background:#03a9f4; color:#fff;">ZAPISZ MQTT W MASZYNIE</button>
                    </form>
                </div>
                
                <div class="card">
                    <h3 style="margin-top:0; color:var(--yellow);">10. Zarządzanie Pamięcią EPROM Maszyny</h3>
                    <p class="help-text">Zabezpiecz obecne wartości w trwałej pamięci lub zresetuj płytę główną maszyny do zaprogramowanego stanu czystego (fabrycznego).</p>
                    <button onclick="if(confirm('Nadpisać trwale domyślne ustawienia wybranej maszyny?')) sendCmd('CMD:SAVEDEF')" class="submit-btn" style="background:var(--yellow); color:#000;">ZAPISZ JAKO DOMYŚLNE</button>
                    <button onclick="if(confirm('UWAGA! Maszyna przywróci ustawienia fabryczne i zresetuje połączenie. Kontynuować?')) sendCmd('CMD:RESTOREDEF')" class="submit-btn" style="background:var(--red); color:#fff; margin-top:10px;">PRZYWRÓĆ FABRYCZNE</button>
                </div>
            </div>

            <div id="SD" class="tab-content">
                <div class="card">
                    <h3 style="margin-top:0; color:var(--yellow);">🧠 Parametry Systemu ESP32 Maszyny</h3>
                    <div class="row"><span>Czas pracy (Uptime):</span> <span class="val" id="esp_up" style="color:var(--text);">--</span></div>
                    <div class="row"><span>Wolna Pamięć RAM:</span> <span class="val" id="esp_ram" style="color:var(--text);">-- %</span></div>
                    <div class="row"><span>Procesor (CPU):</span> <span class="val" id="esp_cpu" style="color:var(--text);">-- MHz</span></div>
                    <div class="row"><span>Model Układu:</span> <span class="val" id="esp_chip" style="color:var(--text);">--</span></div>
                    <div class="row"><span>Zajętość Pamięci (Flash):</span> <span class="val" id="esp_flash" style="color:var(--text);">-- KB</span></div>
                    <div class="row"><span>Adres IP (LAN / Router):</span> <span class="val" id="esp_rip" style="color:var(--text);">--</span></div>
                    <div class="row"><span>Podłączone Telefony (Hala):</span> <span class="val" id="esp_cli" style="color:var(--text);">--</span></div>
                </div>
                <div class="card">
                    <h3 style="margin-top:0; color:#00bcd4;">🩺 Status Sprzętu (Na Żywo)</h3>
                    <div class="row"><span>Zasilanie (PZEM-004T):</span> <span id="st_pzem" class="badge-err">CZEKAM...</span></div>
                    <div class="row"><span>Ekran HMI (Nextion):</span> <span id="st_nex" class="badge-err">CZEKAM...</span></div>
                    <div class="row"><span>Zadajnik (ADS1115):</span> <span id="st_ads" class="badge-err">CZEKAM...</span></div>
                    <div class="row"><span>Falowniki (GP8403):</span> <span id="st_dac" class="badge-err">CZEKAM...</span></div>
                    <div class="row"><span>Izolator I2C (ISO1540):</span> <span id="st_iso" class="badge-err">CZEKAM...</span></div>
                    <div class="row"><span>Klimat (DHT11):</span> <span id="st_dht" class="badge-err">CZEKAM...</span></div>
                    <div class="row"><span>Logi (Karta SD):</span> <span id="st_sd" class="badge-err">CZEKAM...</span></div>
                </div>
                
                <div class="card">
                    <h3 style="margin-top:0;">Pliki na karcie SD (Zdalnie)</h3>
                    <p class="help-text">Pobieranie plików tekstowych wymaga, aby urządzenie pobierające było podłączone do tej samej podsieci co obserwowana maszyna.</p>
                    <button onclick="reqSDList()" style="padding:10px; background:#444; color:#fff; border:none; border-radius:5px; margin-bottom:15px; width:100%;">🔄 Poproś chmurę o listę plików</button>
                    <div id="sd-list">Brak danych... kliknij Odśwież.</div>
                </div>

                <div class="card" style="border: 2px solid var(--purple);">
                    <h3 style="margin-top:0; color:var(--purple);">🔴 Rejestrator Parametrów (Czarna Skrzynka)</h3>
                    <p class="help-text">Moduł wysyła bezpośredni sygnał w sieci LAN do maszyny, zlecając jej gęsty zapis CSV. Twój serwer HUB musi być w zasięgu tego samego routera co maszyna!</p>
                    
                    <div id="log-active-ui" style="display:none; text-align:center; padding: 15px; background:#2a2a2a; border-radius:8px; margin-bottom:15px;">
                        <img src="https://github.com/KamilBol/Regulator-PID-VSC/blob/main/firmware/Logo/Logo%20Bia%C5%82y%20napis%20na%20czarnym%20tle%20mniejsze.jpg?raw=true" style="max-height:50px; border-radius:5px; margin-bottom:10px; animation: pulse 2s infinite;" alt="Logo Rec">
                        <div style="color:var(--red); font-weight:bold; font-size:18px;">🔴 REJESTRACJA W TOKU...</div>
                        <div style="font-size:24px; font-weight:bold; margin-top:5px; color:var(--text);" id="logTimer">--:--</div>
                        <button onclick="triggerLog(0)" style="margin-top:10px; background:var(--red); color:#fff; border:none; padding:10px; border-radius:5px; cursor:pointer; width:100%;">ZATRZYMAJ TERAZ</button>
                    </div>

                    <div id="log-start-ui">
                        <div style="display:flex; justify-content:space-between; margin-bottom:5px;">
                            <button onclick="triggerLog(2)" class="submit-btn" style="background:#555; width:30%; margin-top:0; color:#fff;">2 MIN</button>
                            <button onclick="triggerLog(10)" class="submit-btn" style="background:#555; width:30%; margin-top:0; color:#fff;">10 MIN</button>
                            <button onclick="triggerLog(30)" class="submit-btn" style="background:#555; width:30%; margin-top:0; color:#fff;">30 MIN</button>
                        </div>
                    </div>
                </div>
            </div>

            <div id="OTA" class="tab-content">
                <div class="card" style="border: 2px solid #9c27b0;">
                    <h3 style="margin-top:0; color:#9c27b0;">🚀 Zdalna Aktualizacja Firmware Maszyny (Chmura)</h3>
                    <p class="help-text">Wklej prawidłowy, bezpośredni odnośnik RAW z chmury GitHub wskazujący na skompilowany plik Firmware.bin z nową wersją oprogramowania wybranej maszyny.</p>
                    <input type="text" id="otaUrl" placeholder="https://raw.githubusercontent.com/.../fw_vX.bin">
                    <button class="submit-btn" style="background:#9c27b0; color:#fff;" onclick="cmdOTA()">WYŚLIJ ROZKAZ FLASHOWANIA MASZYNY</button>
                </div>
            </div>

        </div>
    </div>

    <script>
        let activeMachine = "";
        let activeMachineIP = "";
        let lastFocusTime = 0;
        let currentSysON = 0;
        let currentAutoM = 1;

        // Śledzenie focusu formularzy (by blokować przeładowania z API podczas wpisywania przez operatora)
        window.addEventListener('DOMContentLoaded', () => {
            document.querySelectorAll('input, select').forEach(i => {
                i.addEventListener('focus', () => { lastFocusTime = Date.now(); });
                i.addEventListener('input', () => { lastFocusTime = Date.now(); });
                i.addEventListener('blur', () => { lastFocusTime = Date.now(); });
            });
        });

        // Procedura obsługi przełączania kart (DOM Manipulation)
        function openTab(evt, tabName) {
            document.querySelectorAll(".tab-content").forEach(el => el.style.display = "none");
            document.querySelectorAll(".tablinks").forEach(el => el.classList.remove("active"));
            document.getElementById(tabName).style.display = "block";
            evt.currentTarget.classList.add("active");
        }

        // Procedura opuszczania widoku maszyny i powrotu do przeglądu całej floty
        function showDashboard() {
            activeMachine = "";
            document.getElementById("view-machine").style.display = "none";
            document.getElementById("view-dashboard").style.display = "block";
            document.getElementById("btn-back").style.display = "none";
        }

        // Procedura wyboru konkretnej jednostki z listy
        function selectMachine(id) {
            activeMachine = id;
            document.getElementById("active-m-id").innerText = id;
            document.getElementById("view-dashboard").style.display = "none";
            document.getElementById("view-machine").style.display = "block";
            document.getElementById("btn-back").style.display = "inline-block";
            openTab({currentTarget: document.querySelectorAll(".tablinks")[0]}, 'Panel');
        }

        // Rdzeń komunikacyjny: Wysłanie żądania POST do lokalnego C++ z daną komendą, która następnie trafi w MQTT
        function sendCmd(cmdStr) {
            if (!activeMachine) return;
            fetch('/api/send_cmd?id=' + activeMachine, {
                method: 'POST', 
                headers: {'Content-Type': 'application/x-www-form-urlencoded'}, 
                body: 'cmd=' + encodeURIComponent(cmdStr)
            }).then(() => alert("Pomyślnie nadano komendę z Serwera!"));
        }

        // Deklaracje komend natychmiastowych UI
        function toggleSys() { sendCmd(currentSysON ? 'SYSTEM=OFF' : 'SYSTEM=ON'); }
        function toggleMode() { sendCmd(currentAutoM ? 'MODE=MAN' : 'MODE=AUTO'); }
        function toggleEco() { sendCmd('ECO=TOGGLE'); } 

        // Deklaracje komend parsujących ciągi tekstowe z formularzy do formatu strukturalnego maszyny
        function cmdLimits(e) { e.preventDefault(); sendCmd(`CMD:LIMITS:${document.getElementById('minL').value}:${document.getElementById('maxL').value}`); }
        function cmdRatios(e) { e.preventDefault(); sendCmd(`CMD:RATIOS:${document.getElementById('dac1r').value}:${document.getElementById('dac2r').value}`); }
        function cmdPID(e) { e.preventDefault(); sendCmd(`CMD:PID:${document.getElementById('kp').value}:${document.getElementById('ki').value}:${document.getElementById('kd').value}:${document.getElementById('delayTime').value}:${document.getElementById('deadBand').value}`); }
        function cmdAlarms(e) { e.preventDefault(); sendCmd(`CMD:ALARMS:${document.getElementById('ovL').value}:${document.getElementById('recL').value}`); }
        function cmdVoltLimits(e) { e.preventDefault(); sendCmd(`CMD:VOLT:${document.getElementById('minV').value}:${document.getElementById('maxV').value}`); }
        function cmdOutMode(e) { e.preventDefault(); sendCmd(`CMD:OUTMODE:${document.getElementById('outMode').value}`); }
        function cmdCalib(e) { e.preventDefault(); sendCmd(`CMD:CALIB:${document.getElementById('dac1c').value}:${document.getElementById('dac2c').value}`); }
        function cmdWiFi(e) { e.preventDefault(); sendCmd(`CMD:WIFI:${document.getElementById('m_wifiSSID').value}:${document.getElementById('m_wifiPASS').value}`); }
        function cmdMQTT(e) { e.preventDefault(); sendCmd(`CMD:MQTT:${document.getElementById('m_mqSrv').value}:${document.getElementById('m_mqUsr').value}:${document.getElementById('m_mqPas').value}:${document.getElementById('m_mqId').value}`); }
        // Wyzwalanie nagrywania za pomocą bezpośredniego ataku na IP maszyny (Bypassing MQTT Cloud)
        function triggerLog(mins) {
            if (!activeMachineIP || activeMachineIP === "--" || activeMachineIP === "Brak (AP)") {
                alert("Błąd: Serwer HUB nie odebrał lokalnego adresu IP maszyny. Upewnij się, że jesteś połączony z siecią LAN i maszyna zgłosiła swoje IP.");
                return;
            }
            if (mins === 0 && !confirm("Czy na pewno przerwać nagrywanie czarnej skrzynki w maszynie?")) return;
            
            // Strzał prosto w API maszyny przez sieć lokalną
            fetch(`http://${activeMachineIP}/api/start_log?min=${mins}`, {method: 'POST'})
                .then(() => { if(mins > 0) alert("Rozkaz przyjęty. Maszyna rozpoczęła gęste logowanie na karcie SD!"); })
                .catch(() => alert("Brak łączności P2P. Twój telefon/komputer musi być zalogowany do tego samego routera WiFi co maszyna!"));
        }

        // Odpalenie zdalnego wgrywania i upewnienie się co do struktury odnośnika internetowego
        function cmdOTA() {
            let url = document.getElementById('otaUrl').value;
            if(!url.startsWith("http")) {
                return alert("Krytyczny Błąd URL! Link surowy z GitHuba musi zaczynać się od identyfikatora protokołu 'https://'");
            }
            if(confirm(`UWAGA SYSTEMOWA: Zlecasz właśnie fizyczne nadpisanie pamięci flash w pracującej maszynie o ID: ${activeMachine}. Zła wersja pliku spowoduje permanentne uszkodzenie procesora. Upewnij się co do linku RAW. Kontynuować?`)) {
                sendCmd("OTA=" + url);
            }
        }
        
        function reqSDList() {
            document.getElementById('sd-list').innerHTML = "Nawiązywanie wymiany informacji z HiveMQ. Proszę czekać...";
            sendCmd('CMD:SDLIST');
        }

        // Zapis konfiguracji do pamięci Serwera (C++)
        function saveConfig(e) {
            e.preventDefault();
            let p = `ssid=${encodeURIComponent(document.getElementById('c_ssid').value)}&pass=${encodeURIComponent(document.getElementById('c_pass').value)}&msrv=${encodeURIComponent(document.getElementById('c_msrv').value)}&musr=${encodeURIComponent(document.getElementById('c_musr').value)}&mpas=${encodeURIComponent(document.getElementById('c_mpas').value)}`;
            
            fetch('/api/save_config', { 
                method: 'POST', 
                headers: {'Content-Type': 'application/x-www-form-urlencoded'}, 
                body: p 
            }).then(() => { 
                alert("Pomyślnie zaktualizowano bazę danych. Nastąpi twardy restart zasilania Serwera HUB..."); 
                setTimeout(()=>location.reload(), 5000); 
            });
        }

        // ==========================================
        // GŁÓWNA PĘTLA POBIERANIA DANYCH TELEMETRYCZNYCH Z API SERWERA (Zegar 1-Sekundowy)
        // ==========================================
        setInterval(() => {
            // Faza 1: Nasłuch na całą listę zarejestrowanych jednostek floty
            fetch('/api/machines').then(r => r.json()).then(data => {
                if (activeMachine === "") {
                    let html = "";
                    data.forEach(m => {
                        html += `<div class='machine-btn' onclick='selectMachine("${m.id}")'>
                                   <div class='m-title'>⚙️ ${m.id}</div>
                                   <div class='m-stat'><span>Ostatni kontakt nawiązano: ${m.age}s temu</span> <span><span class='dot ok'></span> PING OK</span></div>
                                 </div>`;
                    });
                    if (data.length === 0) {
                        html = "<div style='text-align:center; padding:20px; color:#666;'>Brak połączonych urządzeń w ekosystemie MQTT... Upewnij się że maszyny mają dostęp do Internetu.</div>";
                    }
                    document.getElementById('machine-list').innerHTML = html;
                }
            });

            // Faza 2: Detaliczna ekstrakcja pakietu (Thick/Thin JSON) wybranego procesora docelowego
            if (activeMachine !== "") {
                fetch('/api/machine_data?id=' + activeMachine).then(r => r.json()).then(d => {
                    
                    if (d.sysON !== undefined) currentSysON = d.sysON;
                    if (d.autoM !== undefined) currentAutoM = d.autoM;

                    // Kolorowanie przycisku ECO pod wpływem odczytanej odpowiedzi zwrotnej
                    let btnEco = document.getElementById('btnEco');
                    if (d.eco == "1") { 
                        btnEco.className = "btn-eco"; 
                        btnEco.innerText = "Aktywny Tryb Chmurowy: ECO"; 
                    } else if (d.eco == "0") { 
                        btnEco.className = "btn-max"; 
                        btnEco.innerText = "Aktywny Tryb Chmurowy: MAX (Pełna Analityka)"; 
                    }

                    document.getElementById('amp').innerText = (d.amp !== undefined ? d.amp : "--") + " A";
                    
                    // Interpolacja warunkowa parametrów wycinanych w trybie ECO (uniknięcie wartości undefined w DOM)
                    document.getElementById('setp').innerText = (d.setp !== undefined ? d.setp : "--") + " A";
                    document.getElementById('volt').innerText = (d.volt !== undefined ? d.volt : "--") + " V";
                    
                    let btnSys = document.getElementById('btnSys');
                    if (currentSysON === 1) { 
                        btnSys.className = "ctrl-btn btn-on"; 
                        btnSys.innerText = "Zasilanie: ON (Uruchomiono)"; 
                    } else { 
                        btnSys.className = "ctrl-btn btn-off"; 
                        btnSys.innerText = "Zasilanie: OFF (Zatrzymano)"; 
                    }

                    let btnMode = document.getElementById('btnMode');
                    if (currentAutoM === 1) { 
                        btnMode.className = "ctrl-btn btn-auto"; 
                        btnMode.innerText = "Tryb Regulacji: AUTO (Zamknięta Pętla PID)"; 
                    } else { 
                        btnMode.className = "ctrl-btn btn-man"; 
                        btnMode.innerText = "Tryb Regulacji: MAN (Manualny - Symulacja z Zadajnika)"; 
                    }

                    let trip = document.getElementById('trip');
                    if (d.trip === 1) { 
                        trip.innerText = "BŁĄD ZABEZPIECZENIOWY (Limit Odcięcia Złamany)"; 
                        trip.style.color = "var(--red)"; 
                    } else { 
                        trip.innerText = "Brak odchyleń krytycznych"; 
                        trip.style.color = "var(--green)"; 
                    }

                    // Obsługa wskaźników fizycznych (PZEM, ADC, DAC, DHT)
                    if(d.dac !== undefined) document.getElementById('dac').innerText = d.dac + " V"; else document.getElementById('dac').innerText = "Wygaszono (Tryb ECO)";
                    if(d.dac2v !== undefined) document.getElementById('dac2v').innerText = d.dac2v + " V"; else document.getElementById('dac2v').innerText = "Wygaszono (Tryb ECO)";
                    if(d.pow !== undefined) document.getElementById('pow').innerText = d.pow + " W"; else document.getElementById('pow').innerText = "-- W";
                    if(d.ap_pow !== undefined) document.getElementById('ap_pow').innerText = d.ap_pow + " VA"; else document.getElementById('ap_pow').innerText = "-- VA";
                    if(d.re_pow !== undefined) document.getElementById('re_pow').innerText = d.re_pow + " Var"; else document.getElementById('re_pow').innerText = "-- Var";
                    if(d.pf !== undefined) document.getElementById('pf').innerText = d.pf; else document.getElementById('pf').innerText = "N/A";
                    if(d.temp !== undefined) document.getElementById('temp').innerText = d.temp + " °C"; else document.getElementById('temp').innerText = "Odczyt Wstrzymany";
                    if(d.hum !== undefined) document.getElementById('hum').innerText = d.hum + " %"; else document.getElementById('hum').innerText = "Odczyt Wstrzymany";

                    // Formaty inżynieryjne matematyki czasu działania uC (Uptime)
                    if(d.up_s !== undefined) {
                         let sec = d.up_s;
                         let day = Math.floor(sec / 86400); 
                         let h = Math.floor((sec % 86400) / 3600); 
                         let m = Math.floor((sec % 3600) / 60);
                         document.getElementById('esp_up').innerText = day>0 ? `${day}d ${h}h ${m}m` : `${h}h ${m}m ${sec%60}s`;
                    }
                    if(d.heap_pct !== undefined) document.getElementById('esp_ram').innerText = d.heap_pct + " %";
                    if(d.cpu !== undefined) document.getElementById('esp_cpu').innerText = d.cpu + " MHz";
                    if(d.chip !== undefined) document.getElementById('esp_chip').innerText = d.chip;
                    if(d.sketch_k !== undefined) document.getElementById('esp_flash').innerText = d.sketch_k + " KB";
                    if(d.cli !== undefined) document.getElementById('esp_cli').innerText = d.cli;
                    if(d.ip !== undefined) { 
                        document.getElementById('esp_rip').innerText = d.ip; 
                        activeMachineIP = d.ip; 
                    }

                    // Dynamiczne kolorowanie diagnostyki Hardware Maszyny z tablicy Truth-Table
                    function setSt(id, st) {
                        let el = document.getElementById(id);
                        if (st == 1) { 
                            el.innerText = "AKTYWNY (Zasilanie I2C/SPI Poprawne)"; 
                            el.className = "badge-ok"; 
                        } else { 
                            el.innerText = "BŁĄD KOMUNIKACJI / OFFLINE (Brak sygnału Ping) / ECO"; 
                            el.className = "badge-err"; 
                        }
                    }
                    
                    if(d.pzem !== undefined) setSt('st_pzem', d.pzem);
                    if(d.nex !== undefined) setSt('st_nex', d.nex);
                    if(d.ads !== undefined) setSt('st_ads', d.ads);
                    if(d.dac_st !== undefined) setSt('st_dac', d.dac_st);
                    if(d.dht_st !== undefined) setSt('st_dht', d.dht_st);
                    if(d.sd !== undefined) setSt('st_sd', d.sd);
                    if(d.iso !== undefined) setSt('st_iso', d.iso);

                    // Re-populowanie komórek formularza na bazie danych z maszyny (Timer bezczynności klawiatury 10s)
                    if (Date.now() - lastFocusTime > 10000) {
                        if(d.outM !== undefined) document.getElementById('outMode').value = d.outM;
                        if(d.minL !== undefined) document.getElementById('minL').value = d.minL;
                        if(d.maxL !== undefined) document.getElementById('maxL').value = d.maxL;
                        if(d.kp !== undefined) document.getElementById('kp').value = d.kp;
                        if(d.ki !== undefined) document.getElementById('ki').value = d.ki;
                        if(d.kd !== undefined) document.getElementById('kd').value = d.kd;
                        if(d.dt !== undefined) document.getElementById('delayTime').value = d.dt;
                        if(d.db !== undefined) document.getElementById('deadBand').value = d.db;
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
                }).catch(e => console.log("[FETCH_WARNING] Serwer Hub oczekuje na kompresję pliku JSON od Maszyny Docelowej. Przekroczenie czasu gniazda..."));
                
                // Obsługa wczytywania listingu plików SD pobranego z Chmury jako pakiet informacyjny
                fetch('/api/machine_sd?id=' + activeMachine).then(r => r.json()).then(files => {
                     if (files && files.length >= 0) {
                             let html = "";
                             files.forEach(f => {
                                     // Generowanie przekierowania bezpośrednio do rdzenia HTTP maszyny (Działa tylko na tej samej hali produkcyjnej)
                                     let link = `http://${activeMachineIP}/sd_read?f=${f.name}`;
                                     html += `<div class='file-item'><a href='${link}' target='_blank'>📄 ${f.name}</a> <span>Waga: ${f.size} KiloBajtów</span></div>`;
                             });
                             if (html === "") {
                                 html = "Karta wpięta poprawnie. Brak zalogowanych plików tekstowych na partycji Root. Czekaj na utworzenie AI_LOG.txt.";
                             }
                             document.getElementById('sd-list').innerHTML = html;
                     }
                }).catch(e => {});
            }

            // Faza 3: Nadzór nad warstwą fizyczną Serwera HUB i jego statusem u klienta dostawcy chmury (HiveMQ Broker)
            fetch('/api/server_status').then(r => r.json()).then(data => {
                document.getElementById('srvIP').innerText = data.ip;
                
                // Zabezpieczenie przed brakiem modemu lub routera domowego na zewnątrz portu Gateway
                if (data.ip === "0.0.0.0" || data.ip === "192.168.10.1") {
                    document.getElementById('mqStat').innerText = "Izolacja Sieci: Brak połączenia ze światem zewnętrznym (Tylko sieć lokalna)";
                } else if (data.mqtt_connected === false) {
                    document.getElementById('mqStat').innerText = "Wykryto Internet. Rozwiązywanie hosta brokera MQTT...";
                } else {
                    document.getElementById('mqStat').innerText = "Pełna integracja z usługami Chmurowymi. Tunel aktywny."; 
                    document.getElementById('mqStat').style.color = "var(--green)";
                }

                // Autouzupełnianie w polach setupu serwera
                if (document.activeElement.tagName !== "INPUT") {
                    document.getElementById('c_ssid').value = data.ssid;
                    document.getElementById('c_msrv').value = data.mqtt_srv;
                    document.getElementById('c_musr').value = data.mqtt_usr;
                }
            });

            // Faza 4: Bezpośrednie odpytanie IP maszyny (jeśli jest znane) o to, czy nagrywanie jest aktualnie w toku
            if (activeMachineIP && activeMachineIP !== "--" && activeMachineIP !== "Brak (AP)") {
                fetch(`http://${activeMachineIP}/api/health`)
                    .then(r => r.json())
                    .then(hData => {
                        if(hData.log_rem !== undefined && hData.log_rem > 0) {
                            document.getElementById('log-active-ui').style.display = 'block';
                            document.getElementById('log-start-ui').style.display = 'none';
                            let m = Math.floor(hData.log_rem / 60);
                            let s = hData.log_rem % 60;
                            document.getElementById('logTimer').innerText = (m < 10 ? "0"+m : m) + ":" + (s < 10 ? "0"+s : s);
                        } else {
                            document.getElementById('log-active-ui').style.display = 'none';
                            document.getElementById('log-start-ui').style.display = 'block';
                        }
                    }).catch(e => {}); // Milczymy przy błędzie odpytywania
            }

        }, 1000); // 1000ms = Sztywny zegar przerwania cyklu pomiarowego
    </script>
</body>
</html>
)rawliteral";

// =====================================================================================
// FUNKCJE POMOCNICZE W ŚRODOWISKU SERWERA (Jądro C++)
// =====================================================================================

// System usuwający przypadkowe błędy zapisu operatora (odcięcie nagłówków HTTP z adresu DNS serwera)
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

// =====================================================================================
// BRAMKA AUTORYZACJI ORAZ ENDPOINTY ROZGŁOSZENIOWE API (Serwowanie JSONów do JS)
// =====================================================================================
bool checkAuth() {
    if (!server.authenticate("admin", "regpid12")) {
        server.requestAuthentication();
        return false;
    }
    return true;
}

// Udzielanie podstawowej struktury DOM z zaprogramowanego strumienia const char PROGMEM w celu zaoszczędzenia RAM-u układu ESP32
void handleRoot() {
    if (!checkAuth()) return;
    server.send(200, "text/html", INDEX_HTML);
}

// System radarowy wyłuskujący aktywne maszyny z bazy uC do interfejsu klienta w przeglądarce
void handleGetMachines() {
    if (!checkAuth()) return;
    String json = "[";
    bool first = true;
    unsigned long now = millis();
    
    // Przeszukiwanie sterty przydzielonej do Tablicy Konstrukcyjnej "machines[]"
    for(int i = 0; i < MAX_MACHINES; i++) {
        if(machines[i].id != "") {
            // Analiza znaczników czasu PingTimeout 30 000 ms. Jeśli maszyna milczy dłużej - traktowana jako wylogowana
            if((now - machines[i].lastSeen) < 30000) {
                if(!first) {
                    json += ",";
                }
                json += "{\"id\":\"" + machines[i].id + "\", \"age\":" + String((now - machines[i].lastSeen)/1000) + "}";
                first = false;
            }
        }
    }
    json += "]";
    server.send(200, "application/json", json); // Zwrócenie pakietu wektorowego listującego flotę do pliku zewnetrznego JS
}

// Most buforujący wypakowany JSON konkretnego urządzenia bezpośrednio na żądanie wskaźnika kliknięcia w Frontendzie (Parametr ?id=)
void handleMachineData() {
    if (!checkAuth()) return;
    if (server.hasArg("id")) {
        String reqId = server.arg("id");
        for(int i = 0; i < MAX_MACHINES; i++) {
            if(machines[i].id == reqId) {
                server.send(200, "application/json", machines[i].json);
                return;
            }
        }
    }
    server.send(404, "application/json", "{}");
}

// Most buforujący JSON spisu katalogu Directory Root (Karta uSD podłączona do układu docelowego SPI maszyny)
void handleMachineSD() {
    if (!checkAuth()) return;
    if (server.hasArg("id")) {
        String reqId = server.arg("id");
        for(int i = 0; i < MAX_MACHINES; i++) {
            if(machines[i].id == reqId && machines[i].sd_json != "") { 
                server.send(200, "application/json", machines[i].sd_json); 
                return; 
            }
        }
    }
    server.send(404, "application/json", "[]");
}

// Analiza parametrów samego Hub-a zarządzającego podana w postaci surowego JSON, weryfikacja routingu LAN i tunelu VPN MQTT
void handleServerStatus() {
    if (!checkAuth()) return;
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

// Krytyczny Punkt Wejścia Danych (Relay) - przyjmuje surowe wartości string z przycisków HTML, kompresuje do standardu komend i publikuje jako Pakiet Sterujący przez wirtualny port maszyny /rozkazy
void handleSendCommand() {
    if (!checkAuth()) return;
    
    if (server.hasArg("cmd") && server.hasArg("id") && mqtt.connected()) {
        String cmd = server.arg("cmd");
        String target = server.arg("id");
        String topic = "biuro/" + target + "/rozkazy";
        
        mqtt.publish(topic.c_str(), cmd.c_str());
        Serial.println("[ZDARZENIE SYSTEMOWE] Rozkaz nadrzędny: [" + cmd + "] przekazano do satelity docelowego: " + topic);
        server.send(200, "text/plain", "OK");
    } else {
        server.send(500, "text/plain", "Blad chmury lub brak ID maszyny w żądaniu");
    }
}

// Bezpośredni parser wejścia z formularzy konfiguracyjnych Serwera (Plik .ini) Zapis w pamięci fizycznej Flash
void handleSaveConfig() {
    if (!checkAuth()) return;
    
    if (server.hasArg("ssid")) {
        routerSSID = server.arg("ssid");
    }
    if (server.hasArg("pass") && server.arg("pass") != "") {
        routerPASS = server.arg("pass");
    }
    if (server.hasArg("msrv")) {
        mqtt_server = cleanHostAddress(server.arg("msrv")); 
    }
    if (server.hasArg("musr")) {
        mqtt_user = server.arg("musr");
    }
    if (server.hasArg("mpas") && server.arg("mpas") != "") {
        mqtt_pass = server.arg("mpas");
    }

    memory.putString("ssid", routerSSID);
    memory.putString("pass", routerPASS);
    memory.putString("msrv", mqtt_server);
    memory.putString("musr", mqtt_user);
    memory.putString("mpas", mqtt_pass);

    server.send(200, "text/plain", "OK");
    delay(1000);
    ESP.restart(); // Przerwanie cyklu pracy w celu wywołania procedury BOOT układu ESP32 celem nałożenia nowych loginów WiFi/MQTT
}

// =====================================================================================
// OBSŁUGA POŁĄCZEŃ MQTT DLA SERWERA (ZASADA NASŁUCHU WILDCARD '+')
// =====================================================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String msg = "";
    for (int i = 0; i < length; i++) {
        msg += (char)payload[i];
    }
    
    String t = String(topic);
    
    // Obliczanie położenia znaczników '/' w nadchodzącym strumieniu tematowym, wyodrębnianie i kategoryzowanie nadawcy
    int firstSlash = t.indexOf('/');
    int secondSlash = t.indexOf('/', firstSlash + 1);
    
    if (firstSlash > 0 && secondSlash > firstSlash) {
        String machineId = t.substring(firstSlash + 1, secondSlash);
        String subType = t.substring(secondSlash + 1);
        
        // Procedura automatycznej rejestracji nowej stacji do floty macierzystej Serwera (Analiza komórek pamięci)
        int slot = -1;
        for(int i = 0; i < MAX_MACHINES; i++) {
            if(machines[i].id == machineId) {
                slot = i; // Znaleziono stary wpis. Podpinanie nowego pakietu
                break;
            }
            // Zajmowanie pierwszego niezapisanego Slotu w buforze
            if(machines[i].id == "" && slot == -1) {
                slot = i;
            }
        }
        
        if(slot != -1) {
            // Jeżeli była to czysta (pusta) pozycja, informujemy operatora (Port COM) o zarejestrowaniu nowej stacji z hali
            if (machines[slot].id == "") {
                Serial.println("[RADAR HUB] Odkryto nowy układ satelity w ekosystemie: " + machineId);
            }
            
            machines[slot].id = machineId;
            machines[slot].lastSeen = millis(); // Odnowienie certyfikatu żywotności
            
            // Podłączenie ładunku danych pod odpowiedni typ zmiennej w ramce
            if (subType == "dane") {
                machines[slot].json = msg;
            } else if (subType == "sdlist") {
                machines[slot].sd_json = msg;
            }
        }
    }
}

// Otrzymanie i ciągłe potrzymanie pętli w czasie trwania sygnału do Brokera Cloud z zastosowaniem autoryzacji tokenem
void handleMQTT() {
    if (mqtt_server == "" || routerSSID == "") return; // Pomiń logikę, jeżeli dane autoryzacji są puste w pamięci EPROM
    if (WiFi.status() != WL_CONNECTED || WiFi.localIP().toString() == "0.0.0.0") return;

    if (!mqtt.connected()) {
        if (millis() - lastMqttReconnect > 15000) {
            lastMqttReconnect = millis();
            
            String cleanHost = cleanHostAddress(mqtt_server);
            mqtt.setServer(cleanHost.c_str(), 8883); // Port bezpieczny 8883. Wykorzystuje protokoły SSL

            Serial.print("[KONTROLER MQTT] Uzbrajanie pakietu TCP w celu logowania z centralą: " + cleanHost + "...");
            espClient.stop(); 
            espClient.setInsecure(); // Procedura pominięcia weryfikacji certyfikatu dla zaufanych domowych kanałów
            
            String clientId = "SerwerMultiHUB-" + String(random(0xffff), HEX);
            
            if (mqtt.connect(clientId.c_str(), mqtt_user.c_str(), mqtt_pass.c_str())) {
                Serial.println(" SUKCES! Nawiązano tunel.");
                
                // Kluczowe zastosowanie znaków uniwersalnych (Wildcards). Zamiast wpisywać nazwy każdej z 10-maszyn na sztywno do kodu - nasłuch na wszystko co wpadnie do wirtualnego folderu /biuro
                mqtt.subscribe("biuro/+/dane");
                mqtt.subscribe("biuro/+/sdlist");
                
                Serial.println("[KONTROLER MQTT] Nasłuchuje fal roboczych floty maszyn we wszystkich kierunkach.");
            } else {
                Serial.print(" BŁĄD TRASOWANIA! KOD BLEDU POZIOMU SSL/TCP: ");
                Serial.println(mqtt.state());
            }
        }
    } else {
        // Skonstruowany rdzeń protokołu podtrzymujący przesył serwerów (PING/PONG) zapobiegający zerwaniu na skutek Timeout-u
        mqtt.loop();
    }
}

// =====================================================================================
// SEKCJA BOOTLOADERA (SETUP) - Uruchamiana wyłącznie raz po fizycznym włączeniu przełącznika lub restarcie uC
// =====================================================================================
void setup() {
    pinMode(PIN_LED, OUTPUT);
    Serial.begin(115200); // 115200 Bodów - standard szybkiej wymiany logów na poziomie debuggowania przez program VSC
    delay(1000);
    Serial.println("\n\n--- URUCHAMIAM MULTI-HUB FLOTY (V2.5 LOGO & UNCOMPRESSED / FULL COMMENTS) ---");

    // Czyszczenie rejestru obsługi maszyn z przypadkowych zer pamięci dla struktury przed rozpoczęciem alokacji po wczytaniu
    for(int i = 0; i < MAX_MACHINES; i++) {
        machines[i].id = ""; 
    }

    // Dekodowanie stringów logowania z sektora NV (Non-Volatile) na potrzeby zestawienia zmiennych
    memory.begin("server_conf", false);
    routerSSID = memory.getString("ssid", "");
    routerPASS = memory.getString("pass", "");
    mqtt_server = memory.getString("msrv", "");
    mqtt_user = memory.getString("musr", "");
    mqtt_pass = memory.getString("mpas", "");

    mqtt_server = cleanHostAddress(mqtt_server);
    
    // Potężny bufor na ogromne ładunki typu "Thick-JSON". Zapobiega wyrzucaniu na pysk po otrzymaniu pełnego pakietu parametrów z maszyny. Fabrycznie wynosi to zaledwie ~256 bajtów. Poniższa modyfikacja dopuszcza ~4KB.
    mqtt.setBufferSize(4096); 

    WiFi.disconnect(true);
    WiFi.softAPdisconnect(true);
    delay(100);

    // Połączenie z domowym routerem w celach osiągnięcia Gateway na internet do brokera
    if (routerSSID != "") {
        WiFi.mode(WIFI_AP_STA);
        WiFi.begin(routerSSID.c_str(), routerPASS.c_str());
        Serial.println("[KONTROLER SIECI W-LAN] Odpytuję modem WAN o przyznanie dzierżawy IP i otwarcie tunelu na Serwer...");
    } else {
        // W przeciwnym razie ustaw urządzenie na izolowanym biegu jałowym emitując własną, wewnętrzną sieć Access Point bez podłączania do reszty świata
        WiFi.mode(WIFI_AP);
    }
    
    // Ustalenie lokalnego punktu dostępu (Rozgłośnia AP Serwera) na statycznym, twardo przypisanym kanale klasy klasa C (192.168.10.x)
    IPAddress local_ip(192, 168, 10, 1); 
    IPAddress gateway(192, 168, 10, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(local_ip, gateway, subnet);
    WiFi.softAP("Granulator_HUB"); 

    // Konfiguracja tłumacza DNS na wewnętrzne domeny lokalne po wyrazach bliskoznacznych zamiast rygorystycznego wklepywania adresów IP klawiatura na telefonie. Działa tak na iOS, Windows jak i środowisku Android poprzez implementację systemu Zero Configuration Networking
    if (MDNS.begin("granulator-serwer")) {
        Serial.println("[MODUŁ DOMEN MDNS] Tłumacz nazwy załadowany poprawnie. Adres serwera w sieci domowej osiągalny pod: http://granulator-serwer.local");
    }

    espClient.setInsecure(); // Świadome poluzowanie polityk SSL, aby oszczędzać zasoby przeliczeniowe, gdy podajemy mu pliki OTA z darmowego GitHuba bez wgrywania i śledzenia dedykowanych certyfikatów korzeniowych (Root Certificates) Let's Encrypt / DigiCert.
    mqtt.setServer(mqtt_server.c_str(), 8883);
    mqtt.setCallback(mqttCallback);

    // Rejestracja bezwzględnych ścieżek sieciowych API dla serwowania protokołu warstwy 7 - HTTP
    server.on("/", HTTP_GET, handleRoot);
    server.on("/api/machines", HTTP_GET, handleGetMachines);
    server.on("/api/machine_data", HTTP_GET, handleMachineData);
    server.on("/api/machine_sd", HTTP_GET, handleMachineSD);
    server.on("/api/server_status", HTTP_GET, handleServerStatus);
    server.on("/api/send_cmd", HTTP_POST, handleSendCommand);
    server.on("/api/save_config", HTTP_POST, handleSaveConfig);
    server.begin(); // Odbezpieczenie gniazda Socket 80 nasłuchującego
}

// =====================================================================================
// GŁÓWNA PĘTLA CYKLU SERWERA HUB (Kręci się w nieskończoność zaraz po zakończeniu wywołań w metodzie Setup)
// =====================================================================================
void loop() {
    // 1. Priorytet Nadrzędny - Sprawdzanie i serwowanie odpowiedzi do klientów otwartego połączenia WWW
    server.handleClient();
    
    // 2. Podtrzymywanie fizycznej dzierżawy pętli TCP protokołu do HiveMQ Cloud / Subskrypcje urządzeń nadających na biuro/+/...
    handleMQTT();
    
    // 3. Optyczny Strażnik Przerw - Obwód wywołujący piki napięcia dla mignięcia LED jako dowód działającego rdzenia (Używamy operacji Modulo % aby oszczędzić liczenie i wprowadzanie dodatkowej zmiennej buforującej ms dla Watchdog-a).
    if (millis() % 1000 < 50) {
        digitalWrite(PIN_LED, HIGH);
    } else {
        digitalWrite(PIN_LED, LOW);
    }
}