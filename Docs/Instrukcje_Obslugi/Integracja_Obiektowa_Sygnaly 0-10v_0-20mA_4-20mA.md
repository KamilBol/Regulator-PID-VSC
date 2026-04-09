# 🔌 PROCEDURY INTEGRACJI OBIEKTOWEJ: Adaptacja Sygnałów Sterujących

Niniejszy dokument określa procedury przyłączeniowe układu Regulatora PID do infrastruktury zastanej na hali produkcyjnej. System został zaprojektowany w absolutnej architekturze *Plug & Play*, co oznacza, że jest w stanie wysterować **przenośniki nadawy** niezależnie od standardu komunikacyjnego, w jakim pracuje oryginalny **zadajnik** maszyny, bez konieczności ingerencji w kod źródłowy czy używania lutownicy na obiekcie.

## 🧠 Złota Zasada: Abstrakcja Sprzętowa (Hardware Abstraction)
System wykorzystuje koncepcję wirtualizacji wejść sprzętowych. Przetwornik pomiarowy ADS1115 został na stałe wyposażony w dwa niezależne, wlutowane fabrycznie kanały pomiarowe:
* **KANAŁ A0 (Napięciowy):** Posiada wlutowany na stałe sprzętowy dzielnik napięcia (2x 10 kOhm). Służy do czytania sygnałów z przedziału 0-10V.
* **KANAŁ A1 (Prądowy):** Posiada wlutowany na stałe precyzyjny rezystor bocznikowy (250 Ohm). Służy do czytania pętli prądowych (mA).

**Procedura "Ustaw i Zapomnij":** Operator na hali produkcyjnej dokonuje jedynie pomiaru multimetrem złączy starego zadajnika, wpina przewód do odpowiedniego terminala w szafce (A0 lub A1) i jednorazowo zatwierdza wybór na panelu WWW lub ekranie HMI. Mikrokontroler zapisuje ten wybór w nieulotnej pamięci Flash i od tego momentu autonomicznie przelicza sygnały, opierając się na fizyce obwodów (Prawo Ohma).

Zależnie od wyników pomiarów multimetrem na obiekcie, należy zastosować jeden z trzech poniższych wariantów wpięcia.

---

## 🟢 WARIANT A: Standard Napięciowy (0-10V)
Najczęściej spotykany w starszych układach falownikowych. Sygnałem nośnym określającym żądaną prędkość przenośników nadawy jest zmiana napięcia (Volty).

### 1. Odczyt z Zadajnika (Wejście)
* **Podłączenie fizyczne:** Sygnał sterujący z oryginalnego zadajnika maszyny wpinamy wyłącznie do portu połączonego z pinem **`A0`** (wbudowany dzielnik napięcia).
* **Aktywacja programowa:** W panelu nastaw systemu wybieramy Typ Zadajnika: **Napięciowy (0-10V)**. Procesor trwale zapamiętuje konfigurację `typZadajnika = 0`.
* **Mechanika działania:** 10V z zadajnika maszyny zostaje sprzętowo zbite przez dzielnik do bezpiecznych 5.0V. Wewnętrzna logika procesora automatycznie mnoży ten wynik (x2), odzyskując oryginalne 10.0V na potrzeby obliczeń PID.

### 2. Wyjście na Falownik (Wysterowanie Przenośników Nadawy)
* Moduł DAC (GP8403) natywnie generuje sygnał w standardzie 0-10V.
* Jeżeli układ falownika pracuje w standardzie napięciowym, moduł konwertujący GLK należy pominąć.
* **Konfiguracja Falownika (np. Optidrive E3):** Parametr wejścia analogowego w falowniku maszyny należy ustawić na **`U 0-10`** (Tryb napięciowy).

---

## 🔵 WARIANT B: Standard Prądowy (0-20mA)
Nowoczesny standard przemysłowy, wysoce odporny na zakłócenia elektromagnetyczne i spadki napięć na długich kablach.

### 1. Odczyt z Zadajnika (Wejście)
* **Podłączenie fizyczne:** Sygnał sterujący z oryginalnego zadajnika wpinamy wyłącznie do portu połączonego z pinem **`A1`** (wbudowany rezystor bocznikowy 250 Ohm).
* **Aktywacja programowa:** W panelu nastaw systemu wybieramy Typ Zadajnika: **Prądowy (mA)**. Procesor trwale zapamiętuje konfigurację `typZadajnika = 1`.
* **Mechanika działania:** Zgodnie z prawem Ohma (U = I * R), gdy zadajnik wyśle maksymalny prąd 20mA (0.02A), przepłynie on przez rezystor 250 Ohm, generując na nim spadek napięcia równy idealnie **5.0V** (0.02A * 250 Ohm = 5V). Procesor mnoży ten odczyt w locie, wystawiając żądane sygnały sterujące.

### 2. Wyjście na Falownik (Wysterowanie Przenośników Nadawy)
* Czysty sygnał 0-10V wypracowany przez wewnętrzny algorytm PID prowadzimy do przemysłowego izolatora wejściowego **GLK na szynę DIN (Input 0-10V -> Output 0-20mA)**.
* Moduł GLK sprzętowo zamieni 10V z procesora z powrotem na wysoce odporny sygnał prądowy 20mA i wyśle go na listwę sterującą przenośników.
* **Konfiguracja Falownika:** Parametr wejścia analogowego falownika na obiekcie bezwzględnie przestawiamy na **`A 0-20`** (Tryb prądowy).

---

## 🟣 WARIANT C: Standard Prądowy Przesunięty (4-20mA)
Najbardziej zaawansowany standard *Fail-Safe* w automatyce. Spadek sygnału prądowego do 0 mA fizycznie oznacza zerwanie kabla w hali, na co falownik potrafi natychmiastowo zareagować zatrzymaniem awaryjnym maszyny.

### 1. Odczyt z Zadajnika (Wejście)
* **Podłączenie fizyczne:** Identyczne jak w Wariancie B (port podłączony do pinu **`A1`**).
* **Aktywacja programowa:** Identyczna jak w Wariancie B (Typ Zadajnika: **Prądowy (mA)** / `typZadajnika = 1`).
* **Mechanika działania:** Gdy zadajnik spoczywa na absolutnym minimum, wysyła w pętlę prąd **4 mA**. Przechodząc przez wbudowany rezystor bocznikowy 250 Ohm, generuje on napięcie wynoszące **1.0V**.

### 2. Wyjście na Falownik (Wysterowanie Przenośników Nadawy)
System sprzętowo rozwiązuje problem przesunięcia skali, dowodząc poprawności ułożonej matematyki. Do obsługi tego standardu wykorzystywany jest dokładnie ten sam izolator GLK co w Wariancie B (0-10V -> 0-20mA).

**Dowód idealnego wysterowania bezuderzeniowego (Bumpless Transfer):**
1. Zadajnik podaje minimum (4 mA).
2. Procesor przez rezystor (1.0V) oraz wewnętrzny mnożnik (x2) interpretuje to i wystawia na własny przetwornik DAC napięcie równo **2.0V**.
3. Zewnętrzny moduł GLK na szynie DIN otrzymuje od procesora 2.0V. Ponieważ 2.0V stanowi dokładnie **20%** jego maksymalnego wejścia (10V), moduł GLK fizycznie wypuszcza na przenośniki nadawy **20%** prądu ze swojego zakresu 0-20mA.
4. 20% z 20mA = **dokładnie 4 mA!**

*Wniosek:* Sygnał oryginalny z hali (4mA) został wprowadzony do izolowanego mikrokontrolera, przeliczony i idealnie odtworzony (4mA) na jego wyjściu. Maszyna nie zwalnia ani nie przyspiesza w ułamku sekundy, w którym następuje przejęcie kontroli przez algorytm PID.

* **Konfiguracja Falownika:** Parametr wejścia analogowego w falowniku należy ustawić twardo na standard **`t 4-20`** (Tryb prądowy z detekcją zera sprzętowego na poziomie 4mA).
