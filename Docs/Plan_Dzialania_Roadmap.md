# 🗺️ ROADMAP: Plan Działania i Rozwoju Projektu (Regulator PID)

[cite_start]Projekt ewoluował od prostej koncepcji sterownika (wersja 2.4)  do zaawansowanego, rozproszonego systemu klasy przemysłowej IoT (Wersja 16.6). Poniższy dokument przedstawia zrealizowane kamienie milowe oraz planowane kierunki rozwoju.


## ✅ FAZA 1: Fundament Sprzętowy i Zasady Bezpieczeństwa (ZAKOŃCZONO)
[cite_start]Na tym etapie zdefiniowano architekturę *Hybrid Fail-Safe*, w której stary system maszyny ("Czarne Pudełko") zachowuje absolutny priorytet bezpieczeństwa[cite: 586, 730].
* [cite_start]**Dobór Mózgu:** Wybór ESP32-S3 DevKitC-1-N16R8 (16MB Flash, 8MB PSRAM) jako jednostki centralnej[cite: 461, 594].
* [cite_start]**Pomiary i Wykonawstwo:** Wdrożenie układu PZEM-004T do analizy prądu [cite: 462, 596] [cite_start]oraz precyzyjnego przetwornika DAC I2C sterującego napięciem 0-10V[cite: 463, 600].
* [cite_start]**Architektura Przełączania:** Zastosowanie przekaźników sprzętowych (Styki NC/NO) [cite: 509-510] gwarantujących natychmiastowe przywrócenie kontroli ręcznej w przypadku zrzutu awaryjnego.

## ✅ FAZA 2: Opracowanie Algorytmów "Bumpless Transfer" (ZAKOŃCZONO)
Wyeliminowanie problemu gwałtownych szarpnięć maszyny przy przechodzeniu między trybami.
* [cite_start]**Emulacja Zadajnika:** ESP32 na bieżąco analizuje napięcie ręcznego potencjometru operatora[cite: 589].
* [cite_start]**Miękkie Przejęcie:** Zanim procesor załączy przekaźnik trybu AUTO, wyrównuje swoje napięcie wyjściowe (DAC) do aktualnego napięcia starej gałki [cite: 590, 780-781].
* **Algorytm PID:** Wdrożenie asynchronicznej pętli obliczeniowej utrzymującej prąd w wyznaczonych widełkach.

## ✅ FAZA 3: Era IoT, WebUI oraz Chmury (ZAKOŃCZONO)
[cite_start]Odejście od pierwotnego pomysłu monitorowania maszyny powolnym protokołem Modbus RTU [cite: 733] na rzecz szybkiej, bezprzewodowej telemetrii.
* **Serwer WWW:** Budowa asynchronicznego panelu operatorskiego osadzonego w pamięci PROGMEM.
* **Multi-Node HUB:** Opracowanie oprogramowania dla Serwera Dyspozytorskiego, pozwalającego na zarządzanie flotą maszyn (do 10 jednostek) z jednego biurka.
* **Telemetria MQTT:** Integracja z chmurą HiveMQ z dynamicznym systemem oszczędzania transferu (Tryb ECO / MAX).
* **Smart OTA:** Wdrożenie nieliniowych aktualizacji Firmware'u w locie prosto z repozytorium GitHub (Chunking).

## ⏳ FAZA 4: Testy Obiektowe i Analityka Danych (W TRAKCIE)
Obecny etap skupia się na wdrażaniu systemu na hali i długoterminowej optymalizacji parametrów kinematycznych maszyn.
* [cite_start]**Montaż na obiekcie:** Integracja z falownikami zgodnie z procedurą wpięcia równoległego w obwód PIN 6 [cite: 501-503].
* [cite_start]**Datalogging:** Praca ciągła z zapisem parametrów telemetrycznych na kartę MicroSD w formacie CSV (z rotacją plików) [cite: 604, 799-800].
* [cite_start]**Optymalizacja PID:** Zbieranie danych o opóźnieniach układu mechanicznego w celu ręcznego dostrajania członów Kp, Ki, Kd w Excelu [cite: 704, 984-985]. [cite_start]Odrzucono ryzykowną koncepcję uczenia maszynowego on-board na rzecz "Analityki Big Data"[cite: 705, 982].

## 🚀 FAZA 5: Plany Rozwojowe (FUTURE)
* Wdrożenie powiadomień Push/SMS dla służb Utrzymania Ruchu o zadziałaniu sprzętowego limitu bezpieczeństwa.
* Implementacja lokalnych wykresów z użyciem bibliotek Chart.js bezpośrednio w interfejsie WWW.
