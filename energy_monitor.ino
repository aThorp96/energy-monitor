// Include Emon Library
#include "./EmonLib.h"

// Create an instance
EnergyMonitor emon1;


void setup(){
    Serial.begin(115200);

    emon1.current(1, 111.1);

    while(!emon1.ready()) {
        emon1.loop();
    }
}

void loop() {
    emon1.loop();
    double Irms = emon1.calcIrms();  // Calculate Irms only
    //Serial.print(Irms*123.0);         // Apparent power
    //Serial.print(" ");
    Serial.println(Irms * 123);          // Irms
}
