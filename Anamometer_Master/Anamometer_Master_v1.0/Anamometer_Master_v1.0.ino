//LORA CFM  MASTER
#include <SPI.h>
#include <LoRa_STM32.h>

// Define LoRa pins
#define NSS PB10
#define RESET PB3
#define DIO0 PA15

#define CFM_pin PB6
float calibrationFactor = 2.5;

int cfmFinalVal = 0;

typedef enum internalERR {
  WAIT,
  COMPLETE,
  FAILED
} internalERR;

int CFMcounter = 0;
unsigned long CFMCurrentTime = 0;
unsigned long CFMPreviousTime = 0;
unsigned long CFMInterval = 1000;
bool CFMCalFlag = false;
bool CFMCountFlag = false;

unsigned long previousMillis = 0;
const unsigned long loraSendinterval = 1000; 

void setup() {
 // Serial.begin(9600);
  LoRa.setPins(NSS, RESET, DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
  }
  pinMode(CFM_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(CFM_pin), CFMISR, CHANGE);
  Serial.println("LoRa Sender Ready");
}

void loop() {
  if (returnCFM(&cfmFinalVal) == COMPLETE) {
    cfmFinalVal = cfmFinalVal / calibrationFactor;
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= loraSendinterval) {
    previousMillis = currentMillis;
    sendOveLora(cfmFinalVal);
  }
}

void sendOveLora(unsigned int cfmFinalVal) {
  LoRa.beginPacket();
  LoRa.print(String(cfmFinalVal));
  LoRa.endPacket();
  Serial.print("CFM : ");
  Serial.println(cfmFinalVal);
}


internalERR returnCFM(int *cfm) {
  CFMCalFlag = true;
  CFMCurrentTime = millis();
  if ((CFMCurrentTime - CFMPreviousTime) >= CFMInterval) {
    CFMPreviousTime = CFMCurrentTime;
    double x = (double)CFMcounter / 8;
    *cfm = x * 60;
    CFMcounter = 0;
    return COMPLETE;
  }
  return WAIT;
}

void CFMISR() {
  if (CFMCalFlag) {
    if (digitalRead(CFM_pin) == HIGH) {
      CFMCountFlag = true;
    } else if ((digitalRead(CFM_pin) == LOW) && (CFMCountFlag == true)) {
      CFMCountFlag = false;
      CFMcounter++;
    }
  }
}
