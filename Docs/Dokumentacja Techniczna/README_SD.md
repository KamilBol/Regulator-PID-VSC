# 💾 Dokumentacja Obsługi Karty SD dla ESP32 - Projekt Regulatora

Niniejszy dokument zawiera kompletne informacje dotyczące implementacji zapisu danych na kartę microSD w projekcie przemysłowego regulatora opartego na ESP32. Dokumentacja obejmuje aspekty sprzętowe, programowe oraz analityczne.

## 1. 🔌 Pinologia (Połączenia Sprzętowe)

ESP32 komunikuje się z modułem karty SD za pomocą magistrali **SPI**. W większości płytek (DevKit V1) domyślnie używamy szyny **VSPI**.

| Pin Modułu SD | Funkcja | Pin ESP32 (GPIO) | Opis |
|:---:|:---:|:---:|:---:|
| **CS** | Chip Select | **GPIO 5** | Wybór urządzenia SPI |
| **MOSI** | Data In | **GPIO 23** | Przesył danych do karty |
| **MISO** | Data Out | **GPIO 19** | Przesył danych z karty |
| **SCK** | Clock | **GPIO 18** | Sygnał zegarowy |
| **VCC** | Zasilanie | **5V / 3.3V** | Zależy od modułu (sprawdź stabilizator) |
| **GND** | Masa | **GND** | Wspólna masa układu |

> **⚠️ UWAGA Inżynierska:** Moduły SD są bardzo wrażliwe na jakość zasilania. Jeśli karta "znika" podczas startu silników/styczników na obiekcie, należy wlutować kondensator elektrolityczny (np. 100uF lub 470uF) bezpośrednio na piny VCC i GND modułu czytnika kart.

---

## 2. 💿 Przygotowanie i Formatowanie Karty

ESP32 obsługuje standardowo system plików **FAT32**.

### Jak poprawnie sformatować kartę?
1. **Pojemność:** Najlepiej używać kart **microSD (lub SDHC)** o pojemności do **32GB**. Karty SDXC (64GB i większe) domyślnie formatują się w exFAT, co wymaga dodatkowych bibliotek i obciąża procesor.
2. **Narzędzie:** Nie używaj systemowego formatera Windows (często robi to niedokładnie). Użyj oficjalnego narzędzia: **SD Memory Card Formatter** (dostępne na sdcard.org).
3. **Ustawienia:**
   - Typ: **Overwrite format** (pełne czyszczenie).
   - System plików: **FAT32**.

---

## 3. 📝 Zapis Danych (Jak to działa?)

Dane zapisujemy w formacie **CSV** (Comma Separated Values). Jest to lekki format tekstowy, który z łatwością zaimportujesz do Excela, Pythona czy bazy danych.

### Algorytm logowania:
1. **Inicjalizacja:** Wywołanie `SD.begin(5)` w sekcji `setup()`.
2. **Otwarcie pliku:** Używamy opcji `FILE_APPEND`, aby funkcja dopisywała nowe rekordy na samym końcu pliku, nie niszcząc historii z poprzednich dni.
3. **Zapis linii:** Formowanie Stringa ze zmiennymi oddzielonymi przecinkami.
4. **Zamknięcie (Flush):** Każde dopisanie danych MUSI kończyć się komendą `.close()`. Zapobiega to utracie danych w przypadku nagłego zaniku zasilania szafy sterowniczej.

---

## 4. 📊 Struktura Danych i Interpretacja

Przykładowa struktura pojedynczego rekordu w pliku `logi.csv`:

`2025-12-01 12:00:05, 2.10, 16.5, 14.2, 1`

**Co oznaczają poszczególne kolumny?**
* **Kolumna 1 (Timestamp):** Czas i data odczytu (wymaga synchronizacji z serwerem NTP przez WiFi lub podłączenia modułu czasu rzeczywistego RTC).
* **Kolumna 2 (Output_V):** Napięcie sterujące (0-10V) wysyłane do modułu GLK i dalej na falownik podajnika (Z DAC GP8403).
* **Kolumna 3 (Setpoint_A):** Aktualna nastawa graniczna prądu silnika.
* **Kolumna 4 (Current_Load_A):** Rzeczywiste, fizyczne obciążenie silnika głównego (odczyt na żywo z przekładnika PZEM-004T).
* **Kolumna 5 (State):** Kod błędu/stanu maszyny (np. 0 - Postój, 1 - Praca Automatyczna, 2 - Ochrona przed zacięciem).

---

## 5. 🚀 Zastosowanie Logów na Obiekcie

Moduł SD w tej konfiguracji pełni rolę Przemysłowej Czarnej Skrzynki:
1. **Zabezpieczenie gwarancyjne:** Masz twardy dowód pracy maszyny, jeśli operator przekroczy dopuszczalne parametry mechaniczne.
2. **Analiza zacięć (Anti-Jam):** Na wykresie z CSV dokładnie zobaczysz, jak narastał prąd silnika tuż przed wrzuceniem twardego plastiku i z jaką opóźnieniem zareagował Twój algorytm zdejmując napięcie z podajnika.
3. **Statystyki produkcyjne:** Możliwość wyliczenia roboczogodzin oraz średniego obciążenia w danym dniu/zmianie produkcyjnej.

---

## 6. 🛠️ Kod Inicjalizacyjny C++ (Szablon)

```cpp
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// Przypisanie pinu Chip Select
const int CS_PIN = 5; 

void setup() {
  Serial.begin(115200);
  
  if (!SD.begin(CS_PIN)) {
    Serial.println("Krytyczny Błąd: Brak Karty SD lub złe piny!");
    return;
  }
  Serial.println("Karta SD podłączona poprawnie.");
}

void saveToLog(String timeStamp, float outVoltage, float setpoint, float actualLoad, int status) {
  // Otwarcie pliku w trybie dopisywania
  File dataFile = SD.open("/dane_maszyny.csv", FILE_APPEND);
  
  if (dataFile) {
    dataFile.print(timeStamp);
    dataFile.print(",");
    dataFile.print(outVoltage);
    dataFile.print(",");
    dataFile.print(setpoint);
    dataFile.print(",");
    dataFile.print(actualLoad);
    dataFile.print(",");
    dataFile.println(status);
    
    dataFile.close(); // Zapisanie na karcie
  } else {
    Serial.println("Błąd zapisu pliku na SD!");
  }
}
