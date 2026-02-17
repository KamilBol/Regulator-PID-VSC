#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <PZEM004Tv30.h>
#include <EasyNextionLibrary.h>
#include <DFRobot_GP8403.h>
#include <PID_v1.h>

// ================================================================
// PINOUT (Ten na którym mrugały diody)
// ================================================================
// PZEM: Używamy HardwareSerial 2
#define PZEM_RX_PIN 4  // Tu podłącz TX modułu PZEM
#define PZEM_TX_PIN 5  // Tu podłącz RX modułu PZEM

// NEXTION
#define NEXT_RX 13
#define NEXT_TX 14

// SD CARD
#define SD_CS   15
#define SD_SCK  16
#define SD_MOSI 17
#define SD_MISO 18

// DAC
#define DAC_SDA 41
#define DAC_SCL 40

// ================================================================
// OBIEKTY
// ================================================================

// 1. Definiujemy Serial sprzętowy dla PZEM
HardwareSerial PzemSerial(2); 

// 2. Przekazujemy go do biblioteki
PZEM004Tv30 pzem(PzemSerial, PZEM_RX_PIN, PZEM_TX_PIN);

// 3. Nextion na Serialu 1
EasyNex myNex(Serial1); 

// 4. DAC
DFRobot_GP8403 dac(&Wire, 0x58);

// 5. PID
double Setpoint = 5.0, Input, Output;
PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);

unsigned long lastUpdate = 0;

// ================================================================
// SETUP
// ================================================================
void setup() {
    // 1. Zabezpieczenie przed pętlą restartów
    delay(1000); 
    Serial.begin(115200);
    Serial.println("\n\n--- START ESP32 (RECOVERY MODE) ---");

    // 2. Start PZEM (To jest najważniejsze dla diod)
    // Ręcznie odpalamy port, żeby wymusić piny 4 i 5
    PzemSerial.begin(9600, SERIAL_8N1, PZEM_RX_PIN, PZEM_TX_PIN);
    delay(200);
    Serial.println("PZEM Serial Start (Pin 4/5)...");

    // 3. Start Nextion
    Serial1.begin(9600, SERIAL_8N1, NEXT_RX, NEXT_TX);
    myNex.begin(9600);
    myNex.writeStr("logi.txt", "Start...");

    // 4. Start SD (Z obsługą błędów, żeby nie resetował płytki)
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS)) {
        Serial.println("SD: BŁĄD (Brak karty lub zły format)");
        myNex.writeStr("sd.txt", "ERR FAT32");
        // Nie robimy tu "return", niech program idzie dalej!
    } else {
        Serial.println("SD: OK");
        myNex.writeStr("sd.txt", "OK");
    }

    // 5. Start DAC (Naprawa błędu "Bus already started")
    // Wire.begin() jest wołane wewnątrz dac.begin() w niektórych wersjach,
    // ale dla pewności na S3 wołamy raz tutaj:
    Wire.begin(DAC_SDA, DAC_SCL);
    
    if (dac.begin() != 0) {
        Serial.println("DAC: Błąd komunikacji I2C");
    } else {
        Serial.println("DAC: OK");
        dac.setDACOutRange(dac.eOutputRange10V);
    }
    
    Serial.println("SETUP ZAKONCZONY. Wchodze w LOOP.");
}

// ================================================================
// LOOP
// ================================================================
void loop() {
    // Nextion non-stop
    myNex.NextionListen();

    if (millis() - lastUpdate >= 1000) {
        lastUpdate = millis();

        // Wywołanie odczytu - TO POWODUJE MRUGANIE DIODY TX NA MODULE
        Serial.print("Pytam PZEM... ");
        float v = pzem.voltage();
        float i = pzem.current();

        if (isnan(v)) {
            Serial.println("Brak odpowiedzi (Err)");
            myNex.writeStr("napgr.txt", "ERR");
            myNex.writeStr("granampery.txt", "ERR");
            
            // Logujemy błąd ale nie panikujemy
            myNex.writeStr("logi0.txt", "PZEM Timeout");
        } else {
            Serial.printf("OK! V=%.1f I=%.3f\n", v, i);
            
            // Formatowanie i wysyłka
            char buf[10];
            
            sprintf(buf, "%.1f V", v);
            myNex.writeStr("napgr.txt", buf);
            
            sprintf(buf, "%.3f A", i);
            myNex.writeStr("granampery.txt", buf);
            
            myNex.writeStr("logi0.txt", "OK");
        }
    }
}