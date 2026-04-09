# 🏭 Zaawansowany System Sterowania Granulacją Pelletu (Rozproszone PID & IoT)

![Główne zdjęcie maszyny lub panelu WWW](tutaj_wstaw_link_do_glownego_zdjecia_z_folderu_Media.jpg)

Witaj! Nazywam się **Kamil Ból ("Bólu")**. 
Przedstawiam kompletny, autorski system automatyki przemysłowej oparty o architekturę IoT. Projekt powstał jako odpowiedź na realny problem – optymalizację i automatyzację linii produkującej pellet. 

Zamiast stosować gotowe, zamknięte sterowniki PLC, zaprojektowałem i zaprogramowałem od zera własne rozwiązanie (Sprzęt + Firmware + WebUI), które nie tylko realizuje algorytm PID, ale też zbiera i przetwarza dane telemetryczne w czasie rzeczywistym z użyciem protokołu MQTT.

> ⚠️ **Prawa Autorskie:** Ten projekt **nie jest** Open Source. Stanowi moją własność intelektualną. Został tu udostępniony wyłącznie w celach poglądowych i prezentacyjnych. (Patrz plik [LICENSE](LICENSE)).

---

## ⚙️ Architektura Systemu

System opiera się na dwóch oddzielnych, współpracujących ze sobą jednostkach mikrokontrolerowych (ESP32), napisanych w języku C++ (środowisko PlatformIO / FreeRTOS):

### 1. Maszyna Wykonawcza (Edge Node - Regulator PID)
Mózg pracujący bezpośrednio na hali produkcyjnej, wewnątrz szafy sterowniczej.
* **Zamknięta pętla sprzężenia zwrotnego:** Utrzymuje zadany prąd na linii (Ampery) poprzez dynamiczne, asymetryczne wysterowanie dwóch falowników.
* **Hardware Protection:** Algorytm zabezpieczający na bieżąco analizujący skoki natężenia (zabezpieczenie nadprądowe). W razie zacięcia materiału system sam zrzuca moc i po ustabilizowaniu sytuacji wznawia pracę.
* **Niezależność LAN:** Wbudowany asynchroniczny WebServer. Nawet w przypadku awarii sieci na hali, układ wystawia własny Access Point i pozwala na pełne zarządzanie ze smartfona.

### 2. Centralny HUB Dowodzenia (Serwer Multi-Node)
Jednostka nadrzędna ("dyspozytorska") stojąca w biurze lub dyspozytorni.
* **Skalowalność Floty:** Potrafi nasłuchiwać i zarządzać flotą wielu maszyn (do 10 jednocześnie) za pomocą subskrypcji "Wildcard" w protokole MQTT.
* **Panel operatorski:** Wystawia potężną, lekką stronę WWW z pełną analityką i możliwością zdalnego strojenia parametrów poszczególnych maszyn w czasie rzeczywistym.

---

## 🚀 Kluczowe Technologie i Osiągnięcia w Kodzie

* **Bezpieczna, Nieliniowa Aktualizacja Zdalna (Smart OTA):** Maszyny potrafią aktualizować swój Firmware prosto z repozytorium GitHub. Zastosowałem parsowanie i podążanie za przekierowaniami `HTTP 302 Redirect` oraz wprowadzanie pliku partiami (*Chunking*). Dzięki temu główna pętla programu nie jest blokowana, a na ekranie operatora wyświetla się płynny % pasek postępu.
* **Optymalizacja Pamięci (PROGMEM):** Frontend (HTML, CSS, JS) ważący kilkadziesiąt kilobajtów został w całości skompresowany i zapisany bezpośrednio w pamięci Flash układu ESP32. Oszczędza to kluczowy RAM (SRAM) procesora na obliczenia algorytmu PID.
* **Komunikacja I2C z izolacją galwaniczną:** Pełna komunikacja z peryferiami takimi jak precyzyjny przetwornik ADC (ADS1115) oraz DAC (GP8403). Magistrala chroniona jest szybkim izolatorem I2C (ISO1540), odcinającym logikę mikrokontrolera od zakłóceń potężnych falowników przemysłowych.
* **Dynamiczna Telemetria JSON (Eco/Max):** Zaimplementowany system przepustowości pakietów MQTT. W zależności od potrzeb operatora, maszyna wysyła pełen, "gruby" pakiet JSON ze wszystkimi danymi, lub tryb "ECO" oszczędzający 95% transferu danych w chmurze (HiveMQ).

---

## 🛠️ Wykorzystany Sprzęt (Hardware)
* **Mikrokontrolery:** 2x ESP32 (S3/WROOM)
* **HMI (Human-Machine Interface):** Fizyczny ekran dotykowy Nextion do lokalnej interakcji.
* **Pomiary Prądu:** Moduł PZEM-004T v3.0 (Odczyt V, A, W, VA, PF z sieci 230V/400V).
* **Przetworniki:** * `DFRobot GP8403` (Wyjścia analogowe 0-10V do sterowania zasilaniem falowników).
    * `ADS1115` (16-bitowy odczyt potencjometru / zadajnika trybu ręcznego).
* **Zabezpieczenia / Logika:** Izolatory I2C, wbudowany czytnik kart SD do logowania danych z hali produkcyjnej.

---

## 📁 Nawigacja po Repozytorium

Aby ułatwić poruszanie się po kodzie, zapraszam do poszczególnych folderów:

* 📂 **`/src`** - Tu bije serce maszyny (Główny kod `main.cpp` Regulatora).
* 📂 **`/include`** - Pliki nagłówkowe m.in. skompresowany Frontend (`strona_www.h`).
* 📂 **`/Serwer_PID`** - Kompletny kod źródłowy stacji centralnej (HUBa na biurko).
* 📂 **`/Docs`** - Kompletna dokumentacja projektowa.
    * 📄 [Instrukcje Obsługi WebUI oraz ekranu Nextion](Docs/Instrukcje_Obslugi)
    * 📄 [Dokumentacja Techniczna (I2C, Zabezpieczenia, PID)](Docs/Dokumentacja_Techniczna)
* 📂 **`/Druk_3D`** - Zaprojektowane przeze mnie w SolidWorks obudowy elementów.
* 📂 **`/Media`** - Zdjęcia budowy, screeny interfejsów i linki do YouTube.

---
*Stworzone z pasją do automatyki i czystego kodu C++.* **Kamil Ból**
