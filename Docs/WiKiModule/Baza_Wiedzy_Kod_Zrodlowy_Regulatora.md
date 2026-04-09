# 🧠 BAZA WIEDZY: KOD ŹRÓDŁOWY REGULATORA PID (V16.6 Smart OTA & IoT)

## 1. ARCHITEKTURA I STRUKTURA PLIKU
Kod w języku C++ dla mikrokontrolerów ESP32 opiera się na architekturze asynchronicznej (Non-Blocking). Został napisany w środowisku PlatformIO. Składa się z pięciu głównych bloków logicznych:
1. **Dyrektywy Preprocesora i Biblioteki:** Narzędzia sieciowe (WiFi, MQTT, WebServer) i sprzętowe (I2C, SPI).
2. **Deklaracje Obiektów i Zmiennych (Pamięć RAM):** Wirtualne reprezentacje fizycznego sprzętu i stany maszyny.
3. **Funkcje Pomocnicze i API:** API dla panelu WWW, integracja chmury, silnik aktualizacji OTA.
4. **Sekcja `setup()`:** Bezpieczna konfiguracja początkowa (Bootloader układu).
5. **Sekcja `loop()`:** Wielowątkowa pętla główna oparta na niezależnych timerach sprzętowych.

---

## 2. KLUCZOWE SEKCJE I ICH ZNACZENIE

### A. Dyrektywy i Pinologia (Słownik sprzętowy)
Używamy dyrektyw `#define`, aby twardo przypisać piny mikrokontrolera do słów.
* Dzięki temu kompilator podmienia słowo `PIN_RELAY_2` na cyfrę `38` w całym kodzie. Przy zmianie okablowania w szafie, zmieniamy cyfrę tylko w jednym miejscu.
* **Logika Przekaźników:** Zdefiniowano `RELAY_ON (LOW)` i `RELAY_OFF (HIGH)`. Zastosowane moduły przekaźników są typu "Low-Trigger" (załączają się po podaniu masy). Definicja na stałe eliminuje błędy odwróconej logiki.

### B. Obiekty i Zmienne Globalne
Zmienne utrzymywane w pamięci przez cały czas pracy maszyny:
* **Obiekty sprzętowe:** `ads` (Precyzyjny przetwornik ADC 16-bit), `dac` (Generator napięcia 0-10V), `myNex` (Ekran HMI), `pzem` (Miernik prądu z cewką), `myPID` (Instancja algorytmu matematycznego PID).
* **Filtry i Kalibracja:** * `WSPOLCZYNNIK_DZIELNIKA (1.982)`: Korekta sprzętowych strat napięcia na fizycznych rezystorach zadajnika.
  * `filtr_waga (0.15)`: Współczynnik filtru dolnoprzepustowego EMA (Exponential Moving Average). Wygładza odczyty ADC eliminując szumy z kabli.

---

## 3. FUNKCJE WYKONAWCZE (Podprogramy)

* `initSD()`: Inicjalizacja karty uSD na magistrali SPI. Tworzy i dopisuje wiersz startowy do pliku `/AI_LOG.txt`. Służy do diagnostyki błędów i restartów maszyny.
* `startRegulator()`: Aktywuje tryb automatyczny PID oraz załącza przekaźniki silników. 
  * **Bumpless Transfer (Bezszarpnięciowy start):** Zanim procesor załączy przekaźniki, aplikuje na układ DAC ostatnio wyliczone lub zczytane napięcie. Falowniki ruszają gładko, bez nagłych uderzeń momentu obrotowego.
* `stopRegulator()`: Bezzwłoczne wyłączenie przekaźników.
* `performRemoteOTA(String url)`: **Inteligentny silnik aktualizacji (Smart OTA).** Omija błędy przekierowań GitHuba (`HTTP 302 Redirect`) i wgrywa plik do pamięci Flash w małych paczkach 512-bajtowych (*Chunking*). Dzięki temu w tle nadal działa serwer WWW wyświetlający pasek postępu.
* `handleNextionInput()`: Asynchroniczny parser dotyku. Dekoduje hexadecymalne ramki wejściowe (np. `0x65 0x00 0x0B 0x01` -> Strona 0, Przycisk 11, Wciśnięto).

---

## 4. SERCE PROGRAMU (Boot i Loop)

### Sekcja `setup()` – Bezpieczny Rozruch
Wykonuje się raz po podaniu zasilania 24V DC na płytę główną.
* **Stan Bezpieczny (Hardware Safety First):** Pierwszym rozkazem kompilatora jest definicja pinów przekaźnikowych jako `OUTPUT` i wymuszenie stanu `RELAY_OFF`. Wyklucza to przypadkowy start falowników podczas bootowania procesora.
* Pobranie wszystkich nastaw z trwałej pamięci Flash (`Preferences`) do ulotnej pamięci operacyjnej RAM.
* Uruchomienie lokalnego serwera WWW, modułu mDNS (`granulator.local`) oraz wynegocjowanie kluczy SSL do chmury HiveMQ.

### Sekcja `loop()` – Rdzeń RTOS (Timery)
W nowoczesnym systemie przemysłowym nie używa się funkcji `delay()`, która dławi procesor. System bazuje na niezależnych stoperach `millis()`, co pozwala na asynchroniczną pracę (wielozadaniowość):

#### 1. Pętla Szybka (Co 50ms) - Szyna I2C
Odpowiada za stabilność wysterowania analogowego. Czyta potencjometr przez ADS1115, filtruje wynik, kalibruje uchyby (Offset) i bezzwłocznie bombarduje magistralę I2C układu GP8403 nowym stanem napięcia wyjściowego. 

#### 2. Pętla PID i Zabezpieczeń (Co 200ms)
Mózg regulacji. Czyta amperaż z układu PZEM. Jeśli prąd przekracza twardy limit (`current_Amps >= maxLimit + overloadLimit`), natychmiast wywołuje `stopRegulator()`. Jeśli mieści się w normie, algorytm PID przelicza nowe napięcie, starając się utrzymać maszynę tuż pod progiem MaxLimit.

#### 3. Pętla Wolna HMI (Co 1000ms)
Zadania informacyjne. Odczytuje powolny czujnik DHT11, kalkuluje moc pozorną (VA) oraz bierną (Var) na podstawie cosinusa fi, a następnie formatuje zmienne (C-string `sprintf`) i strzela pakietami aktualizacyjnymi w fizyczny ekran Nextion.

#### 4. Pętla Diagnostyki Sprzętowej (Co 2000ms)
*Heartbeat* systemu. Odbija sygnał od adresów `0x48` i `0x58` na szynie I2C oraz od układu karty SD. Jeśli któryś kabel się obluzuje, maszyna to wykryje i zapali czerwoną kontrolkę błędu w panelu WWW.
