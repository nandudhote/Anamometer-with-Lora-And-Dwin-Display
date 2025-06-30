
#include <SPI.h>
#include <LoRa_STM32.h>
#include <EEPROM.h>  // Include the EEPROM library for reading and writing data to the EEPROM
#include "DHTStable.h"

const byte cfm_add = 0x51;                                                          // Addresses for temperature and humidity data in the display memory
unsigned char CFMVALUE[8] = { 0x5a, 0xa5, 0x05, 0x82, cfm_add, 0x00, 0x00, 0x00 };  // Data frames to send temperature and humidity data to the display
unsigned int previousTime = 0;                                                      // Variable to store the previous time for periodic updates
const byte low_add = 0x61;                                                          
const byte upp_add = 0x62;                                                          

const byte str_add = 0x10;       // High byte of VP address
const byte str_addr_low = 0x00;  // Low byte of VP address


unsigned char LOWVALUE[8] = { 0x5a, 0xa5, 0x05, 0x82, low_add, 0x00, 0x00, 0x00 };  //

unsigned char UPPVALUE[8] = { 0x5a, 0xa5, 0x05, 0x82, upp_add, 0x00, 0x00, 0x00 };  //

// 23 characters = 2(header) + 1(len) + 1(cmd) + 2(addr) + 23(data) = 29 bytes
unsigned char STRVALUE[29] = {
  0x5A, 0xA5,   // Header
  0x1A,         // Length = 26 (1 cmd + 2 addr + 23 data bytes = 26 = 0x1A)
  0x82,         // Write variable command
  str_add,      // VP High byte
  str_addr_low  // VP Low byte
  // Data (23 ASCII characters) will be filled in dynamically
};

int CFMvalue = 0;
int lowerValue = 0;
int upperValue = 0;

// Define LoRa pins
#define NSS PA4
#define RESET PB1
#define DIO0 PB0

#define TX_P 17
#define ENCRYPT 0x78

unsigned char Buffer[9];  // Buffer to store incoming serial data from the display


unsigned long lastLoRaCheck = 0;
unsigned long loraCheckInterval = 50;  // Check LoRa every 50 ms

String recData = "";

unsigned long previousMillis = 0;
const unsigned long interval = 15000;  // 5 seconds

const char ledPin = PA3;
bool ledState = 0;

void setup() {
  delay(2000);
  Serial.begin(9600);
  Serial1.begin(115200);  // Start Serial1 at 115200 baud for communication with display
  // delay(2000);

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
  // controlDWIN(low_add, lowerValue);
  // controlDWIN(upp_add, upperValue);
  sendLOWToDWIN(lowerValue);
  sendHIGHToDWIN(upperValue);

  Serial.println(lowerValue);
  Serial.println(upperValue);


  Serial.println("LoRa Receiver Ready");
}

void loop() {


  if (millis() - lastLoRaCheck >= loraCheckInterval) {
    lastLoRaCheck = millis();
    checkLoRaPackets();  // Check and handle LoRa packets
  }

  if (CFMvalue >= lowerValue && CFMvalue <= upperValue) {
    sendStringToDWIN("need to change the fan");
  } else {
    sendStringToDWIN("                      ");
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    resetAndResetLoRa();
  }
  Data_Display_to_Arduino();
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

  sendCFMToDWIN(CFMvalue);
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

// Function to send integer value to DWIN
void sendCFMToDWIN(int cfmvalue) {
  CFMVALUE[6] = highByte(cfmvalue);
  CFMVALUE[7] = lowByte(cfmvalue);
  Serial1.write(CFMVALUE, 8);
}

// Function to send integer value to DWIN
void sendLOWToDWIN(int cfmvalue) {
  LOWVALUE[6] = highByte(cfmvalue);
  LOWVALUE[7] = lowByte(cfmvalue);
  Serial1.write(LOWVALUE, 8);
}

// Function to send integer value to DWIN
void sendHIGHToDWIN(int cfmvalue) {
  UPPVALUE[6] = highByte(cfmvalue);
  UPPVALUE[7] = lowByte(cfmvalue);
  Serial1.write(UPPVALUE, 8);
}

void Data_Display_to_Arduino() {
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
            //  Serial.println("lower value");
            val = Buffer[7];
            val1 = Buffer[8];
            lowerValue = Buffer[7] * 256 + Buffer[8];
            //  Serial.println(lowerValue);
            break;

          case 0x62:
            Serial.println("upeer value");
            val = Buffer[7];
            val1 = Buffer[8];
            upperValue = Buffer[7] * 256 + Buffer[8];
            // Serial.println(upperValue);
            break;

          case 0x63:
            Serial.println("set");
            saveThresholdsToEEPROM(lowerValue, upperValue);
            break;
            // default:
            //   break;  // Do nothing if the command is invalid
        }
      }
    }
  }
}

void saveThresholdsToEEPROM(int lowVal, int highVal) {
  // Save lowerValue at address 0 and 1
  EEPROM.write(0, highByte(lowVal));
  EEPROM.write(1, lowByte(lowVal));

  // Save upperValue at address 2 and 3
  EEPROM.write(2, highByte(highVal));
  EEPROM.write(3, lowByte(highVal));

  Serial.println("Thresholds saved using EEPROM.write()");
}

void readThresholdsFromEEPROM() {
  lowerValue = (EEPROM.read(0) << 8) | EEPROM.read(1);  // Address 0 and 1
  upperValue = (EEPROM.read(2) << 8) | EEPROM.read(3);  // Address 2 and 3

  // Serial.print("Restored Lower Threshold: ");
  // Serial.println(lowerValue);

  // Serial.print("Restored Upper Threshold: ");
  // Serial.println(upperValue);
}

// void controlDWIN(uint16_t vp_address, int Tdata) {
//   uint8_t command[] = { 0x5A, 0xA5, 0x05, 0x82, vp_address >> 8, vp_address & 0xFF, Tdata >> 8, Tdata & 0xFF };
//   command[6] = highByte(Tdata);
//   command[7] = lowByte(Tdata);
//   Serial.print("Command: ");
//   for (int i = 0; i < sizeof(command); i++) {
//     Serial.print("0x");
//     Serial.print(command[i], HEX);
//     Serial.print(" ");
//   }
//   Serial.println();

//   // Serial1.write(command, sizeof(command));
// }

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
