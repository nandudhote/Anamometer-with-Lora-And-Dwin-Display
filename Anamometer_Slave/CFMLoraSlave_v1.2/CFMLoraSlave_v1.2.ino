
#include <SPI.h>
#include <LoRa_STM32.h>
#include <EEPROM.h>  // Include the EEPROM library for reading and writing data to the EEPROM
#include "DHTStable.h"

// Define LoRa pins
#define NSS PA4
#define RESET PB1
#define DIO0 PB0

#define TX_P 17
#define ENCRYPT 0x78

const char ledPin = PA3;

unsigned char CFMVALUE[8] = { 0x5a, 0xa5, 0x05, 0x82, 0x51, 0x00, 0x00, 0x00 };
unsigned char LowThresholdValue[8] = { 0x5a, 0xa5, 0x05, 0x82, 0x61, 0x00, 0x00, 0x00 };
unsigned char UpperThresholdValue[8] = { 0x5a, 0xa5, 0x05, 0x82, 0x62, 0x00, 0x00, 0x00 };

// 23 characters = 2(header) + 1(len) + 1(cmd) + 2(addr) + 23(data) = 29 bytes
unsigned char STRVALUE[29] = { 0x5A, 0xA5, 0x1A, 0x82, 0x10, 0x00 };

unsigned char Buffer[9];  // Buffer to store incoming serial data from the displayn

String recData = "";

int CFMvalue = 0;
int lowerThresholdValue = 0;
int upperThresholdValue = 0;

unsigned int previousTime = 0;
unsigned long lastLoRaCheck = 0;
unsigned long loraCheckInterval = 50;  // Check LoRa every 50 ms
unsigned long previousMillis = 0;
const unsigned long interval = 15000;  // 5 seconds

bool ledState = 0;


void setup() {
  delay(5000);
  Serial.begin(9600);
  Serial1.begin(115200);  // Start Serial1 at 115200 baud for communication with display

  LoRa.setTxPower(TX_P);
  LoRa.setSyncWord(ENCRYPT);
  LoRa.setPins(NSS, RESET, DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    //while (1) ;
  }
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  readThresholdsFromEEPROM();

  sendValueToDWIN(LowThresholdValue, lowerThresholdValue);
  delay(1000);
  sendValueToDWIN(UpperThresholdValue, upperThresholdValue);
}

void loop() {
  if (millis() - lastLoRaCheck >= loraCheckInterval) {
    lastLoRaCheck = millis();
    checkLoRaPackets();  // Check and handle LoRa packets
  }

  if (CFMvalue >= lowerThresholdValue && CFMvalue <= upperThresholdValue) {
    sendStringToDWIN("need to change the fan");
  } else {
    sendStringToDWIN("                      ");
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    resetAndResetLoRa();
  }
  receiveDataFromDisplay();
}

// Function to check LoRa packets (non-blocking)
void checkLoRaPackets() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    handleReceivedPacket();  // Handle the received data
  }
}

// Function to handle the received packet
void handleReceivedPacket() {
  recData = "";
  while (LoRa.available()) {
    recData += (char)LoRa.read();
  }
  // Serial.println("Received packet: " + recData);

  ledState = (ledState == 0) ? 1 : 0;
  digitalWrite(ledPin, ledState);

  CFMvalue = recData.toInt();

  // sendCFMToDWIN(CFMvalue);
  sendValueToDWIN(CFMVALUE, CFMvalue);
}

// LoRa reset function
void resetAndResetLoRa() {
  LoRa_reset();
  Serial.println("LoRa Receiver");
  LoRa.setTxPower(TX_P);
  LoRa.setSyncWord(ENCRYPT);
  LoRa.setPins(NSS, RESET, DIO0);
  if (!LoRa.begin(433E6)) {
    // Serial.println("Starting LoRa failed!");
    // NVIC_SystemReset();
  }
}

void LoRa_reset() {
  digitalWrite(RESET, LOW);
  delay(100);
  digitalWrite(RESET, HIGH);
  delay(100);
}

void sendValueToDWIN(uint8_t* commandBuffer, int value) {
  commandBuffer[6] = highByte(value);
  commandBuffer[7] = lowByte(value);
  Serial1.write(commandBuffer, 8);
}

void receiveDataFromDisplay() {
  int val = 0;
  int val1 = 0;
  if (Serial1.available() >= 9) {
    for (int i = 0; i < 9; i++) {
      Buffer[i] = Serial1.read();
      delay(1);
    }

    // // Print raw hex (for debugging)
    // for (int i = 0; i < 9; i++) {
    //   Serial.print("0x");
    //   Serial.print(Buffer[i], HEX);
    //   Serial.print(" ");
    // }
    // Serial.println();

    // Check if the data frame is valid and meant for relay control
    if (Buffer[0] == 0X5A) {    // Check for the header byte
      if (Buffer[3] == 0x83) {  // Check if it's a relay control message

        // for (int i = 0; i <= 8; i++) {
        //   Serial.print("0x");
        //   Serial.print(Buffer[i], HEX);  // Print the value in hexadecimal
        //   Serial.print(" ");             // Space between hex values
        // }
        // Serial.println();

        switch (Buffer[4]) {  // Identify which relay to control based on byte 5
          case 0x61:
            val = Buffer[7];
            val1 = Buffer[8];
            lowerThresholdValue = Buffer[7] * 256 + Buffer[8];
            //  Serial.println(lowerThresholdValue);
            break;

          case 0x62:
            Serial.println("upeer value");
            val = Buffer[7];
            val1 = Buffer[8];
            upperThresholdValue = Buffer[7] * 256 + Buffer[8];
            break;

          case 0x63:
            writeThresholdsToEEPROM(lowerThresholdValue, upperThresholdValue);
            break;
          default:
            break;  // Do nothing if the command is invalid
        }
      }
    }
  }
}

void writeThresholdsToEEPROM(int lowVal, int highVal) {
  // Save lowerThresholdValue at address 0 and 1
  EEPROM.write(0, highByte(lowVal));
  EEPROM.write(1, lowByte(lowVal));

  // Save upperThresholdValue at address 2 and 3
  EEPROM.write(2, highByte(highVal));
  EEPROM.write(3, lowByte(highVal));
}

void readThresholdsFromEEPROM() {
  lowerThresholdValue = (EEPROM.read(0) << 8) | EEPROM.read(1);  // Address 0 and 1
  upperThresholdValue = (EEPROM.read(2) << 8) | EEPROM.read(3);  // Address 2 and 3

  // Serial.print("Restored Lower Threshold: ");
  // Serial.println(lowerThresholdValue);

  // Serial.print("Restored Upper Threshold: ");
  // Serial.println(upperThresholdValue);
}


// Function to send up to 23-character string to DWIN using your format
void sendStringToDWIN(const char* str) {
  for (int i = 0; i < 23; i++) {
    if (str[i] == '\0') {
      STRVALUE[6 + i] = ' ';  // Fill remaining space with spaces
    } else {
      STRVALUE[6 + i] = str[i];
    }
  }
  Serial1.write(STRVALUE, sizeof(STRVALUE));  // Send full packet

  // Serial.print("Sent string to DWIN: ");
  // Serial.println(str);
}
