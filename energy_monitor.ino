// Include Emon Library
#include "EmonLib.h"

// Create an instance
EnergyMonitor emon1;
int lastSend = 0;
int now = 0;

#define IPIN 1
#define VPIN 2

void setup(){
    Serial.begin(115200);

    Serial.println("VREAD,IREAD,VCC");
}


void loop() {
    now = millis();
    if (lastSend < now) {
        lastSend = now;
        Serial.print(analogRead(VPIN));
        Serial.print(',');
        Serial.print(analogRead(IPIN));
        Serial.print(',');
        Serial.println(emon1.readVcc());
    }
}
