# 🔌 PROCEDURY INTEGRACJI OBIEKTOWEJ: Adaptacja Sygnałów Sterujących

Niniejszy dokument określa procedury przyłączeniowe układu Regulatora PID do infrastruktury zastanej na hali produkcyjnej. System został zaprojektowany w architekturze *Plug & Play*, co oznacza, że jest w stanie wysterować **przenośniki nadawy** niezależnie od standardu komunikacyjnego, w jakim pracuje oryginalny **zadajnik** maszyny.

## 🧠 Złota Zasada Oprogramowania (Brak Ingerencji w Kod)
Największą zaletą zaprojektowanego systemu jest fakt, że **nie wymaga on absolutnie żadnych modyfikacji w kodzie źródłowym (`main.cpp`)** przy zmianie obiektu. 
Zmienna korygująca `WSPOLCZYNNIK_DZIELNIKA` (ustawiona na `~2.0`) w połączeniu z fizyką obwodów (Prawo Ohma) sprawia, że procesor poprawnie zinterpretuje każdy rodzaj sygnału, opierając się wyłącznie na zmianie jednego fizycznego komponentu elektronicznego na wejściu (rezystora).

Zależnie od wyników pomiarów multimetrem na obiekcie, należy zastosować jeden z trzech poniższych wariantów podłączenia.

---

## 🟢 WARIANT A: Standard Napięciowy (0-10V)
Najczęściej spotykany w starszych układach falownikowych. Sygnałem nośnym dla przenośników nadawy jest zmiana napięcia.

### 1. Odczyt z Zadajnika (Wejście do ADS1115)
* **Problem:** Przetwornik ADS1115 zasilany jest napięciem 5V i podanie na niego pełnych 10V z zadajnika uszkodziłoby układ.
* **Rozwiązanie sprzętowe:** Należy zastosować klasyczny **Dzielnik Napięcia**. Pomiędzy sygnał zadajnika a masę (GND) wpinamy szeregowo dwa identyczne rezystory (np. `10 kOhm` i `10 kOhm`). 
* **Podłączenie:** Sygnał do pinu `A0` w przetworniku ADS pobieramy ze środka dzielnika (spomiędzy rezystorów).
* **Działanie:** 10V z zadajnika zostaje sprzętowo zbite do 5.0V. Kod mnoży to przez x2 i odzyskuje oryginalne 10.0V wewnątrz logiki procesora.

### 2. Wyjście na Falownik (Wysterowanie Przenośników Nadawy)
* Moduł DAC (GP8403) natywnie generuje sygnał 0-10V.
* Jeżeli układ falownika pracuje w standardzie napięciowym, moduł GLK (Konwerter na mA) należy **pominąć** lub zastosować przemysłowy izolator typu *Napięcie-Napięcie (0-10V to 0-10V)*.
* **Konfiguracja Falownika (np. Optidrive E3):** Parametr wejścia analogowego należy ustawić na **`U 0-10`** (Tryb napięciowy).

---

## 🔵 WARIANT B: Standard Prądowy (0-20mA)
Nowoczesny standard przemysłowy, wysoce odporny na zakłócenia kablowe.

### 1. Odczyt z Zadajnika (Wejście do ADS1115)
* **Problem:** ADS1115 nie mierzy prądu (mA), potrafi mierzyć wyłącznie napięcie (V).
* **Rozwiązanie sprzętowe:** Należy użyć precyzyjnego rezystora bocznikowego o wartości **`250 Ohm`**.
* **Podłączenie:** Rezystor ten wpinamy wkładając jedną nóżkę do wejścia sygnału zadajnika, a drugą nóżkę do masy (GND) - połączenie równoległe przed wejściem `A0`.
* **Działanie:** Zgodnie z prawem Ohma (U = I * R), gdy zadajnik wyśle maksymalny prąd 20mA (0.02A), na rezystorze 250 Ohm odłoży się idealnie **5.0V** (0.02 * 250 = 5). Kod mnoży to x2 (10V) i wysyła na wyjście.

### 2. Wyjście na Falownik (Wysterowanie Przenośników Nadawy)
* Czysty sygnał 0-10V z naszego układu DAC prowadzimy do przemysłowego izolatora wejściowego **GLK na szynę DIN (Input 0-10V -> Output 0-20mA)**.
* Moduł GLK zamieni 10V z procesora z powrotem na prąd 20mA i wyśle go w kierunku falownika przenośników.
* **Konfiguracja Falownika:** Parametr wejścia analogowego bezwzględnie przestawiamy na **`A 0-20`** (Tryb prądowy).

---

## 🟣 WARIANT C: Standard Prądowy Przesunięty (4-20mA)
Najbardziej zaawansowany standard *Fail-Safe* w automatyce. Spadek sygnału do 0 mA oznacza fizyczne zerwanie kabla, na co falownik potrafi zareagować alarmem.

### 1. Odczyt z Zadajnika (Wejście do ADS1115)
* **Rozwiązanie sprzętowe:** Identyczne jak w Wariancie B. Używamy rezystora bocznikowego **`250 Ohm`**.
* **Działanie (Matematyka w locie):** * Gdy zadajnik stoi na minimum, wysyła **4 mA**. Na rezystorze odkłada się napięcie **1.0V**.
  * Mikrokontroler czyta 1.0V, system (przez mnożnik x2) interpretuje to jako **2.0V** wysterowania.

### 2. Wyjście na Falownik (Wysterowanie Przenośników Nadawy)
Sytuacja ta udowadnia kunszt matematyczny ułożonego kodu. Nie potrzeba żadnego specjalnego modułu GLK dedykowanego do 4-20mA. Wykorzystujemy ten sam moduł GLK co w Wariancie B (0-10V -> 0-20mA).

**Dowód poprawnego przejścia bezuderzeniowego (Bumpless Transfer):**
1. Zadajnik podaje minimum (4 mA).
2. Procesor przez rezystor interpretuje to i wystawia na przetwornik DAC napięcie **2.0V**.
3. Moduł GLK otrzymuje 2.0V. Ponieważ 2.0V stanowi dokładnie **20%** zakresu (z 10V), moduł GLK fizycznie wypuszcza na przenośniki nadawy **20%** swojego prądu (z 20mA).
4. 20% z 20mA = **dokładnie 4 mA!**
*Sygnał oryginalny został idealnie odtworzony. Przenośniki nadawy nie zwalniają ani nie przyspieszają w momencie załączenia układu PID.*

* **Konfiguracja Falownika:** Parametr wejścia analogowego w falowniku (np. P-16) należy ustawić twardo na standard **`t 4-20`** (Tryb prądowy z detekcją zera na poziomie 4mA).
