# 📖 PODRĘCZNIK UŻYTKOWNIKA (System Granulatora V16.6)
**Autor:** Kamil Ból ("Bólu")

Niniejszy dokument stanowi oficjalną instrukcję obsługi rozproszonego systemu sterowania PID. System został podzielony na trzy interfejsy dostępowe: fizyczny ekran na szafie sterowniczej (Nextion), panel operatorski WWW w samej maszynie oraz centralny Serwer HUB.

---

## ROZDZIAŁ 1: Obsługa Ekranu Dotykowego (Nextion HMI)
Fizyczny ekran zamontowany na szafie sterowniczej służy do szybkiej interakcji na hali produkcyjnej.

### 1. Ekran Główny (Pulpit)
To domyślny widok pracy maszyny.
* **Wskaźnik Prądu [A]:** Centralny, największy odczyt. Pokazuje rzeczywiste obciążenie silnika głównego.
* **Wskaźniki Napięć/Hz (Podajnik 1 i 2):** Informują o aktualnym sygnale wysyłanym z procesora do falowników.
* **Przycisk [PID ON / OFF]:** Główny włącznik systemu.
  * *Status OFF:* Mikrokontroler oddaje sterowanie. Sygnał płynie z oryginalnej gałki na szafie (Bypass).
  * *Status ON:* Mikrokontroler przejmuje stery (Bumpless Transfer) i zaczyna automatycznie dostosowywać prędkość podajników.
* **Przycisk [PRACA AUT / MAN]:** Przełącznik trybu awaryjnego.
  * *Tryb MAN (Manualny):* Jeśli maszyna przeciąży się i zrzuci zasilanie, system pozostanie wyłączony. Wymaga podejścia operatora.
  * *Tryb AUT (Automatyczny):* Jeśli nastąpi przeciążenie, system odczeka na udrożnienie granulatora, po czym samoczynnie wznowi pracę podajników.
* **Przycisk [RESET] (Hold 3s):** Przycisk z zabezpieczeniem przed przypadkowym dotknięciem (wymaga przytrzymania). Całkowicie resetuje system ESP32 do zera i awaryjnie opuszcza przekaźniki.

### 2. Ekran Nastaw (Widełki Pracy)
Dostęp do szybkich korekt algorytmu.
* **Próg MIN:** Ustawienie dolnego prądu. Gdy prąd po przeciążeniu spadnie do tej wartości, maszyna w trybie AUTO wznowi pracę.
* **Próg MAX:** Główny cel regulatora PID. Maszyna będzie przyspieszać podajniki tak, aby prąd silnika utrzymywał się tuż pod tą granicą.
* **Przyciski [+] i [-]:** Służą do skokowej zmiany parametrów o 0.10A.

### 3. Ekran Diagnostyki (Page 4 i 7)
* **IP Maszyny:** Adres sieciowy, po wpisaniu którego w telefonie otworzy się panel WWW.
* **MQTT Tryb:** Pokazuje, czy maszyna wysyła do serwera pełne logi (*MAX*), czy oszczędza pakiet danych (*ECO*).
* **Szczegóły (Page 7):** Precyzyjne odczyty z licznika (Moc Czynna, Bierna, Cosinus Fi) oraz wolne miejsce na karcie SD w Gigabajtach.

---

## ROZDZIAŁ 2: Lokalny Panel WWW Maszyny (Przez WiFi)
Panel dostępny dla inżynierów utrzymania ruchu po wpisaniu w przeglądarce telefonu adresu IP maszyny (np. `192.168.5.1` w trybie awaryjnym lub np. `http://granulator_01.local`).

### 1. Diagnostyka Hardware (Serce Systemu)
Zakładka "Diagnostyka" posiada system sygnalizacji świetlnej (ONLINE / BŁĄD), monitorujący fizyczne przewody w szafie:
* **PZEM-004T:** `ONLINE` oznacza, że maszyna poprawnie mierzy prąd silnika 400V. `BŁĄD` to najpewniej przepalony bezpiecznik zasilający moduł lub zerwany kabel UART.
* **Nextion:** `ONLINE` wskazuje na trwającą wymianę danych z ekranem na drzwiach szafy.
* **ADS1115 (Zadajnik):** `ONLINE` oznacza poprawne połączenie I2C z przetwornikiem "szpiegującym" napięcie oryginalnej gałki (krytyczne dla płynnego startu).
* **GP8403 (DAC):** `ONLINE` potwierdza łączność z układem generującym napięcie 0-10V na falowniki.
* **ISO1540 (Izolator):** Jeśli moduły I2C są Online, oznacza to sprawność izolatora chroniącego procesor przed przebiciem.
* **DHT11:** Odczyt klimatu w szafie.
* **Karta SD:** `ONLINE` wskazuje, że karta uSD jest włożona i maszyna poprawnie zapisuje logi usterkowe w formacie CSV.

### 2. Zakładka Nastawy (Menu Inżynierskie)
Daje dostęp do głębokich parametrów, niewidocznych na małym ekranie.
* **Proporcje Falowników:** Pozwala ustawić asymetrię silników (np. 100% dla DAC1 i 90% dla DAC2).
* **Strojenie PID (Kp, Ki, Kd):** Parametry dynamiki. Modyfikować tylko po analizie wykresów z karty SD.
* **Fizyczne Limity Napięcia:** Zabezpieczenie chroniące wejście falownika przed podaniem zbyt wysokiego napięcia.
* **Korekta DAC (Offset):** Służy do kalibracji. Jeśli procesor wysyła 5.0V, a miernik na falowniku pokazuje 4.8V (straty na długim kablu), wpisujemy tu `0.20V` w celu zniwelowania błędu.
* **Zarządzanie Pamięcią:** Przycisk *Zapisz Domyślne* trwale wypala obecne nastawy w układzie scalonym jako nowe ustawienia fabryczne.

### 3. 🚨 Ukryty Tryb Testowy (Kalibracja Na Sucho)
W celu zestrojenia algorytmu PID bez włączania głównych silników i przepalania materiału, zastosowano tryb serwisowy:
* **Aktywacja:** Należy otworzyć szafkę sterowniczą i wcisnąć fizyczny przycisk `BOOT` na płytce ESP32, trzymając go wciśniętego przez min. 3 sekundy.
* **Działanie:** Dioda LED zacznie szybko migać. Maszyna całkowicie zignoruje odczyty prądu z układu PZEM-004T. W zamian za to podepnie wirtualny prąd pobierany z małego potencjometru testowego zamontowanego na płycie (PIN 3). 
* **Zastosowanie:** Pozwala to symulować obciążenie maszyny ręką i obserwować reakcję napięć na wyjściach falowników na żywo.

---

## ROZDZIAŁ 3: Serwer HUB (Centrum Dowodzenia)
Panel dla Głównego Technologa. Dostępny z komputera w biurze (`http://granulator-serwer.local`). Nie steruje niczym fizycznie – działa jako "Pilot" wydający polecenia maszynom przez chmurę MQTT.

### 1. Dashboard Floty
Początkowy ekran nasłuchujący przestrzeni roboczej. 
* System automatycznie wykrywa każdą zasilaną maszynę podłączoną do Internetu.
* Obok nazwy wyświetlany jest licznik od ostatniego kontaktu (Ping). Jeśli przekroczy 30 sekund, znacznik zmieni się na czerwony, sygnalizując awarię zasilania na hali lub zator w sieci.

### 2. Panel Zarządzania Wybraną Maszyną
Kliknięcie w maszynę otwiera pełny dostęp zdalny do jej parametrów.
* **Zdalne Strojenie:** Wszelkie zmiany w zakładce *Nastawy* są natychmiast wysyłane jako pakiety sterujące (Payload) do wybranej maszyny. Zmiany są bezzwłoczne.
* **Zdalny Restart:** Pozwala twardo zresetować procesor ESP32 na hali bez wstawania od biurka.
* **Eksplorator Karty SD:** Umożliwia pobranie pliku CSV z logami z ostatnich 24 godzin prosto na komputer, w celu obróbki w Excelu.
* **Zdalne OTA:** Krytyczna funkcja serwisowa. Wklejenie w to pole bezpośredniego linku do pliku `.bin` z nowym oprogramowaniem na platformie GitHub (Raw Link) wymusi na pracującej maszynie asynchroniczne pobranie nowego kodu, flashowanie pamięci i samoczynny restart w celu wdrożenia nowej wersji oprogramowania.
