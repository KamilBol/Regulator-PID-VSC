# ⚡ BAZA WIEDZY: Izolacja Galwaniczna i Konwersja Sygnałów

Niniejszy dokument opisuje kluczowy element bezpieczeństwa systemu – sprzętową tarcze ochronną. Oddzielamy czułą elektronikę (procesor ESP32) od agresywnego środowiska hali produkcyjnej (falowniki, styczniki, prądy udarowe) za pomocą barier galwanicznych.

## 1. Wyjaśnienie elementów: Co to jest i do czego służy?

### B0505S-1W (Przetwornica izolująca napięcie DC-DC)
!https://github.com/KamilBol/Regulator-PID-VSC/blob/main/Docs/WiKiModule/Picture/ADS1115%20(Przetwornik%20ADC)%20.jpg?raw=true
* **Co to jest:** To miniaturowy zasilacz z wbudowanym mikro-transformatorem.
* **Co robi:** Pobiera prąd z czystej strefy zasilania procesora i generuje nowe, całkowicie "odcięte" zasilanie 5V. Stanowi barierę galwaniczną. Jeśli na linii z maszyną dojdzie do potężnego zwarcia lub przepięcia (np. 400V), energia spali ten moduł, ale fizycznie nie ma prawa cofnąć się i zniszczyć sterownika ESP32.
* **Piny (od strony napisów):**
  * **Pin 1 (GND / -Vin):** Masa wspólna ze sterownikiem ESP32.
  * **Pin 2 (VCC / +Vin):** Zasilanie 5V ze strony sterownika ESP32.
  * **Pin 3 (0V / -Vout):** NOWA, izolowana masa dla strefy brudnej.
  * **Pin 4 (+Vo / +Vout):** NOWE, izolowane 5V do zasilania brudnej strefy sygnałowej.

### ADS1115 (Precyzyjny Przetwornik ADC 16-bit)
* **Co to jest:** Cyfrowy miernik napięcia komunikujący się z układem poprzez szynę I2C.
* **Co robi:** Zastąpił problematyczny protokół Modbus w obszarze bezpieczeństwa. Pozwala sterownikowi na "podsłuchanie" napięcia na zadajniku ręcznym operatora maszyny. Dzięki niemu system jest w stanie wyrównać swoje parametry wyjściowe i wykonać tzw. *Bumpless Transfer* (bezuderzeniowe przejęcie kontroli) bez zatrzymywania maszyny.

### Przemysłowy Izolator/Rozdzielacz Sygnału (Moduł GLK DIN-Rail)
* **Co to jest:** Profesjonalny układ automatyki przemysłowej do kondycjonowania i konwersji sygnałów analogowych, montowany w szafie sterowniczej na uniwersalnej szynie DIN.
* **Co robi (Zastępuje "Zieloną Płytkę"):** Moduł ten przyjmuje wyliczone, sterujące napięcie (0-10V) wygenerowane przez nasz wewnętrzny układ DAC (GP8403) i sprzętowo "tłumaczy" je na sygnał przemysłowy, który akceptuje falownik (np. pętlę prądową 0-20mA lub separowane napięcie). Moduł wprowadza własną barierę galwaniczną między wejściem a wyjściem, likwidując "pętle masy" i czyniąc komunikację z napędami całkowicie wolną od szumów oraz zakłóceń elektromagnetycznych.

---

## 2. BIBLIA POŁĄCZEŃ (Instrukcja lutowania krok po kroku)

Stół roboczy / płytkę dzielimy na dwie odrębne przestrzenie miedziane:
* **Strefa Czysta:** Po stronie mikrokontrolera ESP32.
* **Strefa Brudna (Izolowana):** Po stronie zasilającej przetworniki podłączone do maszyny.
*(Pinout wg założeń: GPIO 1 to SDA, a GPIO 2 to SCL dla magistrali I2C)*.

### KROK 1: Zasilanie strefy czystej (Tarcza napięciowa)
1. Wlutuj 5V ze swojego głównego zasilacza (zasilającego ESP) do pinu 2 (`+Vin`) na kostce B0505S-1W.
2. Wlutuj Masę (GND) z tego samego zasilacza do pinu 1 (`-Vin`) na B0505S-1W.
3. Poprowadź przewód z pinu 3.3V na ESP32 do pinu `VCC1` na izolatorze magistrali I2C (ISO1540). *(To konieczne, bo logika ESP działa na napięciu 3.3V, a nie 5V)*.
4. Poprowadź przewód z pinu GND na ESP32 do pinu `GND1` na izolatorze ISO1540.

### KROK 2: Komunikacja od strony ESP32 do izolatora (Strefa Czysta)
1. Z pinu 1 (SDA) na ESP32 poprowadź kabel do pinu `SDA1` na izolatorze ISO1540.
2. Z pinu 2 (SCL) na ESP32 poprowadź kabel do pinu `SCL1` na izolatorze ISO1540.

### KROK 3: Wyjście z tarczy napięciowej (Budowa Strefy Brudnej)
**KRYTYCZNE UWAGI BHP:** Od tego momentu wchodzimy na obwód prawy izolatora. Żaden kabel z tej sekcji nie ma prawa fizycznie zetknąć się z żadnym kablem ze Strefy Czystej!
1. Z pinu 4 (`+Vo`) na kostce B0505S-1W (nasze NOWE 5V) rozprowadź kable zasilające do:
   * Pinu `VCC2` na izolatorze ISO1540.
   * Pinu `VDD` na przetworniku ADS1115.
   * Pinu `VCC` na module DAC GP8403.
2. Z pinu 3 (`0V`) na kostce B0505S-1W (nasza NOWA masa) rozprowadź kable powrotne do:
   * Pinu `GND2` na izolatorze ISO1540.
   * Pinu `GND` na przetworniku ADS1115.
   * Pinu `GND` na module DAC GP8403.
   * Pinu `ADDR` na module ADS1115 *(Wymusza na sztywno adres logiczny 0x48 sprzętu na I2C)*.

### KROK 4: Komunikacja po Stronie Brudnej (Izolowanej)
Przekazanie sygnałów cyfrowych I2C poza barierę galwaniczną.
1. Z pinu `SDA2` izolatora ISO1540 pociągnij dwa rozgałęzienia do:
   * Pinu `SDA` na module ADS1115.
   * Pinu `SDA` na module DAC GP8403.
2. Z pinu `SCL2` izolatora ISO1540 pociągnij dwa rozgałęzienia do:
   * Pinu `SCL` na module ADS1115.
   * Pinu `SCL` na module DAC GP8403.

### KROK 5: Połączenie Sygnałów z Maszyną

**A. Analiza prądu na starym zadajniku (Wejście do ADS1115)**
* Moduł ten nie jest w stanie przyjąć napięcia wyższego niż jego nowe napięcie zasilania (czyli 5V). Jeżeli oryginalny zadajnik maszyny operuje zakresem 0-10V, **bezwzględnie wymagany jest sprzętowy dzielnik napięcia** (np. użycie dwóch rezystorów 10 kOhm dla redukcji sygnału o połowę).
* Minus (GND) z systemu starego zadajnika łączysz fizycznie z Nową Masą (Pin 3 kostki B0505S-1W).
* Zredukowany sygnał wejściowy (+) lutujesz do pinu `A0` na przetworniku ADS1115. Piny `A1`, `A2`, `A3` oraz `ALRT` muszą pozostać niezawarte z niczym innym.

**B. Wysyłanie Wysterowania do Falownika (Wyjście z DAC -> Izolator GLK DIN)**
* Ostatnim etapem jest wysterowanie systemu. Czysty, analogowy sygnał 0-10V wychodzący z modułu **DAC GP8403** poprowadź kablami do wejść (`Input 0-10V`) Twojego nowego izolatora GLK montowanego na szynie DIN.
* Dopiero z wyjść izolatora GLK (`Output`) ciągniesz grube, instalacyjne kable sterownicze prosto do listwy terminali wejściowych głównego falownika. Układ jest w pełni zabezpieczony i gotowy do pracy w środowisku wysokich prądów.
