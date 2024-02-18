// Include Emon Library
#include "EmonLib.h"

// Create an instance
EnergyMonitor emon1;
unsigned long lastSend = 0;
unsigned long  now = 0;

#define IPIN 1
#define VPIN 2

void setup(){
    Serial.begin(115200);

        Serial.print("VPIN");
        Serial.print(',');
        Serial.print("IPIN");
        Serial.print(',');
        Serial.print("readVcc");
        Serial.println();
}


void loop() {
    now = millis();
    if (lastSend < now) {
        lastSend = now;
        Serial.print(analogRead(VPIN));
        Serial.print(',');
        Serial.print(analogRead(IPIN));
        Serial.print(',');
        Serial.print(emon1.readVcc());
        Serial.println();
    }
}
