// ================================================================
// DOŁĄCZANIE BIBLIOTEK (Narzędzia do obsługi modułów)
// ================================================================
#include <Arduino.h>              // Podstawowa biblioteka środowiska Arduino
#include <Wire.h>                 // Biblioteka do komunikacji I2C (piny SDA/SCL)
#include <SPI.h>                  // Biblioteka do komunikacji SPI (np. karta SD)
#include <SD.h>                   // Biblioteka do obsługi kart SD (zapis/odczyt)
#include <PZEM004Tv30.h>          // Biblioteka do miernika prądu PZEM
#include <EasyNextionLibrary.h>   // Biblioteka do obsługi ekranu dotykowego Nextion
#include <DFRobot_GP8403.h>       // Biblioteka do modułu DAC (nasz generator napięcia)
#include <Adafruit_ADS1X15.h>     // Biblioteka do modułu ADC (nasz czytnik napięcia ADS1115)
#include <PID_v1.h>               // Biblioteka do algorytmu Regulatora PID
#include <DHT.h>                  // Biblioteka do czujnika temperatury

// ================================================================
// PINOLOGIA (Nadawanie przyjaznych nazw numerom pinów)
// ================================================================
#define PIN_PZEM_RX     4         // Pin odbierający dane z PZEM
#define PIN_PZEM_TX     5         // Pin wysyłający dane do PZEM

#define PIN_I2C_SDA     2         // Pin danych I2C (Strefa Brudna)
#define PIN_I2C_SCL     1         // Pin zegara I2C (Strefa Brudna)

#define PIN_RELAY_1     47        // Pin sterujący pierwszym przekaźnikiem
#define PIN_RELAY_2     38        // Pin sterujący drugim przekaźnikiem (bezpieczny pin)

#define PIN_NEXT_RX     13        // Pin odbierający dane z ekranu Nextion
#define PIN_NEXT_TX     14        // Pin wysyłający dane do ekranu Nextion
#define PIN_DHT         20        // Pin czujnika temperatury DHT

// Piny do czytnika kart SD (protokół SPI)
#define PIN_SD_CS       15        // Pin wyboru urządzenia (Chip Select)
#define PIN_SD_SCK      16        // Pin zegara (Clock)
#define PIN_SD_MOSI     17        // Pin wysyłania danych na kartę
#define PIN_SD_MISO     18        // Pin odbierania danych z karty

// Stany logiczne dla przekaźników (żeby nie mylić HIGH z LOW)
#define RELAY_ON        LOW       // Włączenie przekaźnika (Low-Trigger zwiera do masy)
#define RELAY_OFF       HIGH      // Wyłączenie przekaźnika

// ================================================================
// TWORZENIE OBIEKTÓW (Powołanie modułów do życia)
// ================================================================
HardwareSerial NextionSerial(1);  // Tworzymy port szeregowy nr 1 dla ekranu
HardwareSerial PzemSerial(2);     // Tworzymy port szeregowy nr 2 dla PZEM

EasyNex myNex(NextionSerial);     // Przypisujemy port 1 do biblioteki Nextion
PZEM004Tv30 pzem(PzemSerial, PIN_PZEM_RX, PIN_PZEM_TX); // Konfigurujemy PZEM na porcie 2
DHT dht(PIN_DHT, DHT11);          // Konfigurujemy czujnik DHT11

DFRobot_GP8403 dac(&Wire, 0x58);  // Tworzymy obiekt DAC przypisany do adresu 0x58
Adafruit_ADS1115 ads;             // Tworzymy obiekt miernika ADC ADS1115

// Obiekty i zmienne do obsługi regulatora PID
double Setpoint = 5.0;            // Wartość, do której PID będzie dążył (np. 5 Amperów)
double Input;                     // Aktualna wartość (np. obecne Ampery z PZEM)
double Output;                    // To, co wypluje PID (zmienione napięcie dla DAC)
PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT); // Konfiguracja PID

// ================================================================
// ZMIENNE GLOBALNE (Pojemniki na dane dostępne w całym kodzie)
// ================================================================
float minLimit = 10.0;            // Minimalny limit zabezpieczenia
float maxLimit = 40.0;            // Maksymalny limit prądu
bool systemON = false;            // Prawda/Fałsz: Czy regulator jest włączony (Przekaźniki ON)
bool modeAUTO = true;             // Prawda/Fałsz: Czy PID steruje sam (Auto) czy ręcznie

float napiecieZadajnika = 0.0;    // Tu trzymamy wygładzone napięcie odczytane z maszyny
float currentDac1 = 0.0;          // Napięcie, które chcemy podać na DAC 1
float currentDac2 = 0.0;          // Napięcie, które chcemy podać na DAC 2
String logFileName = "/log_000.csv"; // Nazwa pliku na karcie SD

// Zegarki (Timery) do planowania zadań w czasie
unsigned long lastUpdate = 0;     // Kiedy ostatnio odświeżono ekran (milisekundy)
unsigned long lastLogTime = 0;    // Kiedy ostatnio zapisano na kartę SD
unsigned long lastFastUpdate = 0; // Kiedy ostatnio zrobiono szybki pomiar (nowy timer!)
unsigned long resetPressTime = 0; // Kiedy wciśnięto przycisk resetu
bool isResetPressed = false;      // Czy przycisk resetu jest teraz wciśnięty

// Dane do kalibracji sprzętu
const float WSPOLCZYNNIK_DZIELNIKA = 1.982; // Matematyczna korekta z Twoich pomiarów
float filtr_waga = 0.15;          // Siła wygładzania (15% nowego pomiaru, 85% starego)

// ================================================================
// FUNKCJE POMOCNICZE (Małe bloki kodu wykonujące jedno zadanie)
// ================================================================

// Funkcja skanująca, czy urządzenia I2C są prawidłowo podłączone
void scanI2C() {
    Serial.println("[I2C] Skanowanie magistrali...");
    byte error, address;
    int nDevices = 0; // Licznik znalezionych urządzeń
    
    // Pętla sprawdzająca każdy możliwy adres od 1 do 127
    for(address = 1; address < 127; address++ ) {
        Wire.beginTransmission(address); // Próba nawiązania połączenia
        error = Wire.endTransmission();  // Zapisz wynik próby (0 = sukces)
        if (error == 0) {
            Serial.print("[I2C] Znaleziono urzadzenie pod adresem: 0x");
            if (address<16) Serial.print("0"); // Dodaj zero z przodu dla ładnego formatu
            Serial.println(address,HEX);       // Wyświetl adres w formacie szesnastkowym (HEX)
            nDevices++;                        // Zwiększ licznik urządzeń
        }
    }
    if (nDevices == 0) Serial.println("[I2C] Brak urzadzen! Sprawdz kable.");
    else Serial.println("[I2C] Koniec skanowania.");
}

// Funkcja uruchamiająca kartę SD i tworząca nowy plik do zapisu
void initSD() {
    SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS); // Uruchom protokół SPI
    if(!SD.begin(PIN_SD_CS)) { // Jeśli karta nie odpowiada
        Serial.println("[SD] BLAD INICJALIZACJI! (Sprawdz format FAT32)");
        myNex.writeStr("sd.txt", "ERR"); // Pokaż błąd na ekranie
        return; // Przerwij dalsze wykonywanie tej funkcji
    }
    
    int i = 0;
    // Pętla szukająca pierwszej wolnej nazwy pliku (np. log_0.csv, log_1.csv...)
    while(true) {
        logFileName = "/log_" + String(i) + ".csv"; // Sklejanie nazwy
        if(!SD.exists(logFileName)) break; // Jeśli plik NIE istnieje, wyjdź z pętli (mamy nazwę!)
        i++;
    }
    
    File f = SD.open(logFileName, FILE_WRITE); // Otwórz nowy plik do zapisu
    if(f) {
        f.println("Czas;PID;Prad;Nap;Moc;Zadajnik;DAC1;DAC2"); // Zapisz nagłówki tabeli
        f.close(); // Zawsze zamykaj plik po zapisie!
        Serial.println("[SD] Utworzono plik: " + logFileName);
    }
}

// ================================================================
// LOGIKA STEROWANIA SYSTEMEM
// ================================================================

// Funkcja włączająca system (Przejęcie maszyny)
void startRegulator() {
    systemON = true;                      // Zmień status systemu na WŁĄCZONY
    myNex.writeStr("pidonoff.txt", "ON"); // Zaktualizuj przycisk na ekranie
    
    digitalWrite(PIN_RELAY_1, RELAY_ON);  // Fizyczne zwarcie styków przekaźnika 1
    digitalWrite(PIN_RELAY_2, RELAY_ON);  // Fizyczne zwarcie styków przekaźnika 2
    
    Serial.println("[SYS] START REGULATORA (Przejecie plynne)");
}

// Funkcja wyłączająca system (Oddanie kontroli zadajnikowi ręcznemu)
void stopRegulator() {
    systemON = false;                      // Zmień status systemu na WYŁĄCZONY
    myNex.writeStr("pidonoff.txt", "OFF"); // Zaktualizuj przycisk na ekranie
    
    digitalWrite(PIN_RELAY_1, RELAY_OFF);  // Rozwarcie styków przekaźnika 1
    digitalWrite(PIN_RELAY_2, RELAY_OFF);  // Rozwarcie styków przekaźnika 2
    
    Serial.println("[SYS] STOP REGULATORA (Powrot do trybu recznego)");
}

// Funkcja aktualizująca liczby na ekranie Ustawień
void updateSettingsScreen() {
    myNex.writeStr("min.txt", String(minLimit, 1)); // Wyślij limit minimum z 1 miejscem po przecinku
    myNex.writeStr("max.txt", String(maxLimit, 1)); // Wyślij limit maksimum
}

// Funkcja reagująca na kliknięcia przycisków plus/minus na ekranie
void processButtonAction(int id) {
    if (id == 1) { minLimit += 0.1; if(minLimit > maxLimit) minLimit = maxLimit; updateSettingsScreen(); }
    if (id == 2) { minLimit -= 0.1; if(minLimit < 0) minLimit = 0; updateSettingsScreen(); }
    if (id == 3) { maxLimit += 0.1; if(maxLimit > 100) maxLimit = 100; updateSettingsScreen(); }
    if (id == 4) { maxLimit -= 0.1; if(maxLimit < minLimit) maxLimit = minLimit; updateSettingsScreen(); }
}

// ================================================================
// PARSER DOTYKU (Czytanie tego, co klika użytkownik na ekranie)
// ================================================================
int activeButtonID = 0;          // ID wciśniętego przycisku
unsigned long buttonHoldTimer = 0; // Czas dla funkcji przytrzymania przycisku
bool isButtonHeld = false;       // Czy przycisk jest nadal trzymany

void handleNextionInput() {
    while (NextionSerial.available()) { // Dopóki przychodzą dane z ekranu
        byte b = NextionSerial.read();  // Odczytaj jeden bajt
        if (b == 0x65) {                // 0x65 to specjalny kod Nextiona oznaczający dotyk
            delay(5);                   // Daj mu chwilę na przysłanie reszty danych
            if (NextionSerial.available() >= 3) {
                byte pageId = NextionSerial.read(); // Z jakiej strony kliknięto
                byte cmpId  = NextionSerial.read(); // ID przycisku
                byte event  = NextionSerial.read(); // Rodzaj zdarzenia (Wciśnięto = 0x01, Puszczono = 0x00)
                while (NextionSerial.available()) NextionSerial.read(); // Wyczyść resztę bufora

                // LOGIKA DLA STRONY GŁÓWNEJ (ID = 0)
                if (pageId == 0) {
                    if (cmpId == 11 && event == 0x01) { // Kliknięto przycisk ON/OFF
                        if(systemON) stopRegulator(); else startRegulator(); // Przełącz stan
                    }
                    if (cmpId == 12 && event == 0x01) { // Kliknięto przycisk AUTO/MAN
                        modeAUTO = !modeAUTO; // Odwróć wartość (jeśli true to zrób false)
                        myNex.writeStr("pracaautoman.txt", modeAUTO ? "AUT" : "MAN");
                    }
                    if (cmpId == 8) { // Przycisk RESET
                        if (event == 0x01) { isResetPressed = true; resetPressTime = millis(); myNex.writeStr("logi0.txt", "Trzymaj 5s..."); }
                        else if (event == 0x00) { isResetPressed = false; myNex.writeStr("logi0.txt", "..."); }
                    }
                }
                
                // LOGIKA DLA STRONY USTAWIEŃ (ID = 2)
                if (pageId == 2) {
                    if (event == 0x01) { // Przycisk wciśnięty
                        activeButtonID = cmpId; 
                        isButtonHeld = true; 
                        processButtonAction(activeButtonID); // Wykonaj akcję raz
                        buttonHoldTimer = millis() + 400;    // Ustaw opóźnienie dla przytrzymania
                    }
                    else if (event == 0x00) { // Przycisk puszczony
                        activeButtonID = 0; 
                        isButtonHeld = false; 
                    }
                }
            }
        }
    }
    
    // Jeśli przycisk jest trzymany dłużej, powtarzaj akcję (szybkie dodawanie)
    if (isButtonHeld && activeButtonID > 0 && millis() > buttonHoldTimer) {
        processButtonAction(activeButtonID);
        buttonHoldTimer = millis() + 100; // Akcja co 100ms
    }
    
    // Jeśli trzymano Reset przez 5 sekund, uruchom ponownie układ
    if (isResetPressed && (millis() - resetPressTime > 5000)) {
        myNex.writeStr("logi0.txt", "RESTART!"); 
        delay(500); 
        ESP.restart(); // Wymuś sprzętowy restart mikrokontrolera
    }
}

// ================================================================
// SEKCJA SETUP (Uruchamia się jeden jedyny raz po restarcie)
// ================================================================
void setup() {
    // 1. ZABEZPIECZENIE PRZEKAŹNIKÓW (Zawsze na samym początku!)
    pinMode(PIN_RELAY_1, OUTPUT);         // Ustaw pin jako wyjście napięcia
    pinMode(PIN_RELAY_2, OUTPUT);
    digitalWrite(PIN_RELAY_1, RELAY_OFF); // Wymuś stan WYŁĄCZONY
    digitalWrite(PIN_RELAY_2, RELAY_OFF);

    delay(1000); // Daj zasilaczom sekundę na ustabilizowanie napięć
    Serial.begin(115200); // Uruchom monitor portu szeregowego do komputera
    Serial.println("\n--- SYSTEM V9.1 (Szybki Pomiar Analog) ---");

    // 2. URUCHOMIENIE EKRANU
    NextionSerial.begin(9600, SERIAL_8N1, PIN_NEXT_RX, PIN_NEXT_TX);
    myNex.begin(9600);
    
    // 3. URUCHOMIENIE PZEM
    PzemSerial.begin(9600, SERIAL_8N1, PIN_PZEM_RX, PIN_PZEM_TX);
    
    // 4. URUCHOMIENIE CZUJNIKA TEMPERATURY
    dht.begin();

    // 5. URUCHOMIENIE STREFY BRUDNEJ (DAC + Miernik na pinach 1 i 2)
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL); // Włączenie I2C
    scanI2C();                            // Sprawdzenie czy widać sprzęt
    
    // Inicjalizacja Miernika Napięcia (ADS1115 na adresie 0x48)
    ads.setGain(GAIN_TWOTHIRDS); // Konfiguracja wzmocnienia: czytamy napięcia do max 6.1V
    if (!ads.begin(0x48)) {
        Serial.println("[ADS] Blad komunikacji! Sprawdz adres 0x48.");
    } else {
        Serial.println("[ADS] Polaczono poprawnie.");
    }

    // Inicjalizacja Generatora Napięcia (DAC na adresie 0x58)
    if(dac.begin() != 0) {
        Serial.println("[DAC] Blad komunikacji! (Adres inny niz 0x58?)");
    } else {
        Serial.println("[DAC] Polaczono poprawnie.");
        dac.setDACOutRange(dac.eOutputRange10V); // Ustaw zakres pracy 0-10V
        dac.setDACOutVoltage(0, 0);              // Wymuś 0V na kanale 0
        dac.setDACOutVoltage(0, 1);              // Wymuś 0V na kanale 1
    }

    // 6. URUCHOMIENIE KARTY SD
    initSD();

    // Resetowanie napisów na ekranie do ustawień domyślnych
    myNex.writeStr("pidonoff.txt", "OFF");
    myNex.writeStr("pracaautoman.txt", "AUT");
    stopRegulator(); // Dla pewności jeszcze raz wyłączamy przekaźniki
    
    Serial.println("SYSTEM GOTOWY");
}

// ================================================================
// GŁÓWNA PĘTLA PROGRAMU (Kręci się bez przerwy)
// ================================================================
void loop() {
    handleNextionInput(); // Cały czas sprawdzaj, czy naciśnięto ekran

    // -------------------------------------------------------------
    // SZYBKA PĘTLA (Uruchamia się co 50 milisekund)
    // Służy tylko do czytania maszyny i aktualizacji DAC
    // -------------------------------------------------------------
    if (millis() - lastFastUpdate >= 50) {
        lastFastUpdate = millis(); // Zresetuj zegarek

        // KROK 1: Odczyt napięcia z pinu ADC
        int16_t adc_surowe = ads.readADC_SingleEnded(0); 
        float napiecie_na_pinie = ads.computeVolts(adc_surowe); // Zamiana na Volty
        
        // KROK 2: Kalibracja (Mnożymy przez wyliczony współczynnik, ucinamy błędy)
        float aktualny_odczyt = napiecie_na_pinie * WSPOLCZYNNIK_DZIELNIKA; 
        if(aktualny_odczyt < 0.05) aktualny_odczyt = 0.0;
        if(aktualny_odczyt > 10.5) aktualny_odczyt = 10.5;

        // KROK 3: Filtracja. Miesza stary wynik z nowym, usuwając szybkie skoki napięcia.
        if (napiecieZadajnika == 0.0 && aktualny_odczyt > 0.0) {
            napiecieZadajnika = aktualny_odczyt; // Od razu łapie pierwsze napięcie
        } else {
            napiecieZadajnika = (aktualny_odczyt * filtr_waga) + (napiecieZadajnika * (1.0 - filtr_waga));
        }

        // KROK 4: Logika wysyłania na DAC
        if (!systemON) {
            // Jeśli system jest WYŁĄCZONY, DAC kopiuje ręczny zadajnik (przygotowanie do przejęcia)
            currentDac1 = napiecieZadajnika;
            currentDac2 = currentDac1 * 0.90; // Proporcja 90% na drugi kanał
        } else {
            // Jeśli system jest WŁĄCZONY, tutaj docelowo będzie rządził PID.
            // Zanim zaprogramujemy PID, DAC po prostu trzyma ostatnią wartość,
            // dlatego napięcie jest stabilne. Nie zmieniamy zmiennych currentDac.
        }

        // KROK 5: Ostateczne wysłanie polecenia do fizycznego sprzętu DAC (Wymaga miliwoltów)
        // Sprawdzamy, czy napięcie nie przekracza 10V (10000 mV). Jeśli tak, blokujemy na 10000.
        uint16_t mv_dac1 = (currentDac1 <= 10.0) ? (currentDac1 * 1000) : 10000;
        uint16_t mv_dac2 = (currentDac2 <= 10.0) ? (currentDac2 * 1000) : 10000;

        // Wysłanie danych (Napięcie, Numer kanału)
        // Robimy to ZAWSZE, żeby układ nie "zgubił" napięcia przy włączonych przekaźnikach!
        dac.setDACOutVoltage(mv_dac1, 0); 
        dac.setDACOutVoltage(mv_dac2, 1);
    }


    // -------------------------------------------------------------
    // WOLNA PĘTLA (Uruchamia się co 1000 milisekund = 1 sekunda)
    // Służy do odświeżania ekranu, czytania powolnych czujników i logów
    // -------------------------------------------------------------
    if (millis() - lastUpdate >= 1000) {
        lastUpdate = millis(); // Zresetuj zegarek

        // Pobranie powolnych odczytów z zewnętrznych czujników
        float u = pzem.voltage();
        float i = pzem.current();
        float p = pzem.power();
        float pf = pzem.pf();
        float t = dht.readTemperature();
        float h = dht.readHumidity();

        // AKTUALIZACJA EKRANU: Moduł Prądu (PZEM)
        if(!isnan(u)) { // Jeśli napięcie to poprawna liczba (czyli jest kontakt z PZEM)
            float s = u * i; // Obliczanie Mocy Pozornej
            float q = (s > p) ? sqrt(s*s - p*p) : 0; // Obliczanie Mocy Biernej
            
            char buf[16]; // Pudełko na posklejany tekst
            
            // sprintf skleja liczbę i literkę w ładny napis i wkleja do 'buf'
            sprintf(buf, "%.3f A", i); 
            myNex.writeStr("granampery.txt", buf); // Wyślij na ekran
            myNex.writeStr("natgr.txt", buf);
            
            sprintf(buf, "%.1f V", u); myNex.writeStr("napgr.txt", buf);
            sprintf(buf, "%.0f W", p); myNex.writeStr("mocczy.txt", buf);
            sprintf(buf, "%.0f VA", s); myNex.writeStr("mocpoz.txt", buf);
            sprintf(buf, "%.0f Var", q); myNex.writeStr("mocbie.txt", buf);
            sprintf(buf, "%.2f", pf); myNex.writeStr("wspmoc.txt", buf);
            
            // Kontrolny napis do komputera z przeliczonym aktualnym napięciem Zadajnika
            Serial.printf("[PZEM] OK | I=%.3f A | Zadajnik=%.2f V\n", i, napiecieZadajnika);
        } else { // Jeśli PZEM nie odpowiada
            myNex.writeStr("napgr.txt", "ERR");
            Serial.printf("[PZEM] Brak komunikacji! | Zadajnik=%.2f V\n", napiecieZadajnika);
        }

        // AKTUALIZACJA EKRANU: Temperatura
        if(!isnan(t)) { 
            myNex.writeStr("temperatura.txt", String(t, 1));
            myNex.writeStr("wilgotnosc.txt", String(h, 0));
        }

        // AKTUALIZACJA EKRANU: Pokazywanie, co obecnie wysyła nasz generator (DAC)
        char dacBuf[10];
        sprintf(dacBuf, "%.2f V", currentDac1); // Utnij do dwóch miejsc po przecinku
        myNex.writeStr("pod1.txt", dacBuf);
        myNex.writeStr("dac1.txt", dacBuf);
        
        sprintf(dacBuf, "%.2f V", currentDac2);
        myNex.writeStr("pod2.txt", dacBuf);
        myNex.writeStr("dac2.txt", dacBuf);

        // LOGOWANIE DANYCH: Zapis wyników na małą kartę SD
        if(SD.cardType() != CARD_NONE) { // Jeśli fizycznie jest karta w czytniku
             float gb = SD.totalBytes() / (1024.0*1024.0*1024.0); // Przelicz pojemność
             myNex.writeStr("sd.txt", String(gb, 1) + " GB"); // Wyświetl ją na ekranie
             
             if(millis() - lastLogTime >= 2000) { // Co 2 sekundy dopisz linijkę
                 lastLogTime = millis();
                 File f = SD.open(logFileName, FILE_APPEND); // Otwórz plik w trybie dodawania
                 if(f) {
                     // Zapisz dane oddzielone średnikami (Format CSV dla Excela)
                     f.print(millis()); f.print(";");
                     f.print(systemON); f.print(";");
                     f.print(i); f.print(";");
                     f.print(u); f.print(";");
                     f.print(p); f.print(";");
                     f.print(napiecieZadajnika); f.print(";");
                     f.print(currentDac1); f.print(";");
                     f.println(currentDac2); // println na końcu robi znak nowej linii (Enter)
                     f.close(); // Pamiętaj o zamknięciu pliku!
                 }
             }
        } else {
            myNex.writeStr("sd.txt", "NO SD"); // Komunikat o braku karty
        }
    }
}