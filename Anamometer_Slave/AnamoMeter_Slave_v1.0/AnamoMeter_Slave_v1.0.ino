// Include required libraries
#include <SPI.h>         // For SPI communication (used by LoRa module)
#include <LoRa_STM32.h>  // Library to handle LoRa communication on STM32
#include <EEPROM.h>      // For reading and writing persistent data
#include "DHTStable.h"   // Likely for DHT sensor (not used in this code)

// ------------------ Pin definitions ------------------
#define NSS PA4    // LoRa module Chip Select pin
#define RESET PB1  // LoRa module Reset pin
#define DIO0 PB0   // LoRa module Interrupt pin

#define TX_P 17       // LoRa transmit power
#define ENCRYPT 0x78  // LoRa sync word (acts like encryption)

// LED pin definition
const char ledPin = PA3;

// ------------------ Command buffers for DWIN display ------------------
unsigned char CFMVALUE[8] = { 0x5a, 0xa5, 0x05, 0x82, 0x51, 0x00, 0x00, 0x00 };             // Command to update the CFM value on display
unsigned char LowThresholdValue[8] = { 0x5a, 0xa5, 0x05, 0x82, 0x61, 0x00, 0x00, 0x00 };    // Command to update the lower threshold value
unsigned char UpperThresholdValue[8] = { 0x5a, 0xa5, 0x05, 0x82, 0x62, 0x00, 0x00, 0x00 };  // Command to update the upper threshold value
unsigned char STRVALUE[29] = { 0x5A, 0xA5, 0x1A, 0x82, 0x10, 0x00 };                        // Command to send a string (max 23 characters) to display

// ------------------ Buffers and variables ------------------
// Buffer to store data received from display
unsigned char Buffer[9];

// String to store data received from LoRa
String recData = "";

// Variables to store values
int CFMvalue = 0;             // Measured airflow value
int lowerThresholdValue = 0;  // Lower threshold
int upperThresholdValue = 0;  // Upper threshold

// Timing variables
unsigned int previousTime = 0;
unsigned long lastLoRaCheck = 0;       // Last time we checked for LoRa packet
unsigned long loraCheckInterval = 50;  // Check LoRa every 50ms
unsigned long previousMillis = 0;
const unsigned long interval = 15000;  // Restart LoRa every 15 seconds

bool ledState = 0;  // LED state

// ------------------ Setup ------------------
void setup() {
  delay(5000);            // Wait for hardware to stabilize
  Serial.begin(9600);     // Debug serial
  Serial1.begin(115200);  // Serial to DWIN display

  // Initialize LoRa module
  LoRa.setTxPower(TX_P);
  LoRa.setSyncWord(ENCRYPT);
  LoRa.setPins(NSS, RESET, DIO0);
  if (!LoRa.begin(433E6)) {
    // Serial.println("Starting LoRa failed!");
    // Optional: infinite loop if LoRa fails
  }

  pinMode(ledPin, OUTPUT);    // Configure LED pin
  digitalWrite(ledPin, LOW);  // Turn LED off initially

  // Load saved thresholds from EEPROM
  readThresholdsFromEEPROM();

  // Send initial thresholds to display
  sendValueToDWIN(LowThresholdValue, lowerThresholdValue);
  delay(1000);
  sendValueToDWIN(UpperThresholdValue, upperThresholdValue);
}

// ------------------ Loop ------------------
void loop() {
  // Check LoRa packets at regular interval (non-blocking)
  if (millis() - lastLoRaCheck >= loraCheckInterval) {
    lastLoRaCheck = millis();
    checkLoRaPackets();
  }

  // If CFMvalue is within thresholds, show warning message on display
  if (CFMvalue >= lowerThresholdValue && CFMvalue <= upperThresholdValue) {
    sendStringToDWIN("need to change the fan");
  } else {
    sendStringToDWIN("                      ");  // Clear message
  }

  // Restart LoRa periodically
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    resetAndResetLoRa();
  }

  // Check for data from display
  receiveDataFromDisplay();
}

// ------------------ LoRa functions ------------------

// Check if a LoRa packet was received
void checkLoRaPackets() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    handleReceivedPacket();  // Handle the packet
  }
}

// Handle the received LoRa packet
void handleReceivedPacket() {
  recData = "";
  while (LoRa.available()) {
    recData += (char)LoRa.read();  // Read data into string
  }

  // Toggle LED as a sign of data reception
  ledState = (ledState == 0) ? 1 : 0;
  digitalWrite(ledPin, ledState);

  // Convert received string to integer airflow value
  CFMvalue = recData.toInt();

  // Send updated CFM value to DWIN display
  sendValueToDWIN(CFMVALUE, CFMvalue);
}

// Restart and reconfigure LoRa
void resetAndResetLoRa() {
  LoRa_reset();                     // Hardware reset
  Serial.println("LoRa Receiver");  // Debug message
  LoRa.setTxPower(TX_P);
  LoRa.setSyncWord(ENCRYPT);
  LoRa.setPins(NSS, RESET, DIO0);
  if (!LoRa.begin(433E6)) {
    // Serial.println("Starting LoRa failed!");
  }
}

// Perform LoRa hardware reset
void LoRa_reset() {
  digitalWrite(RESET, LOW);
  delay(100);
  digitalWrite(RESET, HIGH);
  delay(100);
}

// ------------------ Display communication ------------------

// Send integer value to DWIN display
void sendValueToDWIN(uint8_t* commandBuffer, int value) {
  commandBuffer[6] = highByte(value);  // High byte of value
  commandBuffer[7] = lowByte(value);   // Low byte of value
  Serial1.write(commandBuffer, 8);     // Send to display
}

// Receive and handle data from display
void receiveDataFromDisplay() {
  int val = 0, val1 = 0;

  // Wait until full 9-byte frame is available
  if (Serial1.available() >= 9) {
    for (int i = 0; i < 9; i++) {
      Buffer[i] = Serial1.read();  // Read each byte
      delay(1);                    // Small delay between reads
    }

    // Check for valid header
    if (Buffer[0] == 0X5A) {
      // Check if this is a control command
      if (Buffer[3] == 0x83) {
        switch (Buffer[4]) {
          case 0x61:  // Lower threshold update
            lowerThresholdValue = Buffer[7] * 256 + Buffer[8];
            break;

          case 0x62:  // Upper threshold update
            upperThresholdValue = Buffer[7] * 256 + Buffer[8];
            break;

          case 0x63:  // Command to save thresholds to EEPROM
            writeThresholdsToEEPROM(lowerThresholdValue, upperThresholdValue);
            break;

          default:
            break;  // Ignore unknown commands
        }
      }
    }
  }
}

// ------------------ EEPROM functions ------------------

// Save thresholds to EEPROM
void writeThresholdsToEEPROM(int lowVal, int highVal) {
  EEPROM.write(0, highByte(lowVal));   // Lower threshold high byte
  EEPROM.write(1, lowByte(lowVal));    // Lower threshold low byte
  EEPROM.write(2, highByte(highVal));  // Upper threshold high byte
  EEPROM.write(3, lowByte(highVal));   // Upper threshold low byte
}

// Read thresholds from EEPROM at startup
void readThresholdsFromEEPROM() {
  lowerThresholdValue = (EEPROM.read(0) << 8) | EEPROM.read(1);
  upperThresholdValue = (EEPROM.read(2) << 8) | EEPROM.read(3);
}

// ------------------ Send string to DWIN display ------------------

// Send up to 23-character string to display
void sendStringToDWIN(const char* str) {
  for (int i = 0; i < 23; i++) {
    if (str[i] == '\0') {
      STRVALUE[6 + i] = ' ';  // Fill unused space with spaces
    } else {
      STRVALUE[6 + i] = str[i];  // Copy character
    }
  }
  Serial1.write(STRVALUE, sizeof(STRVALUE));  // Send complete message
}
