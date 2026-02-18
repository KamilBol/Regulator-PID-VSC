#include <Arduino.h>
#include <ModbusRTU.h>

#define RX_PIN 16
#define TX_PIN 17
#define DE_PIN 4
#define BTN_UP 18
#define BTN_DOWN 19

ModbusRTU mb;
uint16_t freq = 5000; // Start: 50.00 Hz
unsigned long lastBtn = 0;

void setup() {
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
    
    mb.begin(&Serial2, DE_PIN);
    mb.slave(1); // ID Falownika = 1
    mb.addHreg(0x1000, freq); // Rejestr częstotliwości
    
    pinMode(BTN_UP, INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);
    Serial.println("TESTER GOTOWY");
}

void loop() {
    mb.task();
    mb.Hreg(0x1000, freq); // Aktualizuj rejestr dla Mastera

    if(millis() - lastBtn > 200) {
        if(digitalRead(BTN_UP) == LOW) {
            freq += 500; // +5.00 Hz
            if(freq > 6000) freq = 6000;
            Serial.printf("SET: %.2f Hz\n", freq/100.0);
            lastBtn = millis();
        }
        if(digitalRead(BTN_DOWN) == LOW) {
            freq -= 500;
            if(freq < 0) freq = 0;
            Serial.printf("SET: %.2f Hz\n", freq/100.0);
            lastBtn = millis();
        }
    }
}