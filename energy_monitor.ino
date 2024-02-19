// Include Emon Library
#include "EmonLib.h"

#define CMD_START 'S'
#define CMD_STOP 'X'
#define CMD_ACK 'A'
#define CMD_KAY 'K'

#define IPIN 1
#define VPIN 2

// Create an instance
EnergyMonitor emon1;

bool started = false;
char command = '_';
int voltageValue = 0;
int currentValue = 0;
long vcc = 0L;

char* vccBuff = (char*)&vcc;
char* vBuff = (char*)&voltageValue;
char* cBuff = (char*)&currentValue;


void setup(){
  Serial.begin(115200);
}


void loop() {
  if (Serial.available()) {
      command = Serial.read();
      if (command == CMD_START) {
        started = true;
        Serial.write(CMD_ACK);
        while (command != CMD_KAY) {
          command = Serial.read();
        }
      } else if (command == CMD_STOP) {
        started = false;
      }
  }

  if (started) {
    voltageValue = analogRead(VPIN);
    currentValue = analogRead(IPIN);
    vcc = emon1.readVcc();

    Serial.write(vBuff, 2);
    Serial.write(cBuff, 2);
    Serial.write(vccBuff, 4);
  }
}
