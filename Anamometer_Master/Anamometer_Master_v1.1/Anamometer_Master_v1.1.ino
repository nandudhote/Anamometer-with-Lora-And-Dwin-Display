
// Include required libraries
#include <SPI.h>         // For SPI communication (required by LoRa module)
#include <LoRa_STM32.h>  // Library for LoRa on STM32

// ------------------ LoRa pins ------------------
#define NSS PB10   // LoRa module Chip Select pin
#define RESET PB3  // LoRa Reset pin
#define DIO0 PA15  // LoRa interrupt pin

// ------------------ Airflow (CFM) measurement ------------------
#define CFM_pin PB6             // Pin connected to airflow pulse sensor
float calibrationFactor = 2.5;  // Calibration factor to adjust measured CFM value

// Define a custom type (enum) for status reporting
typedef enum internalERR {
  WAIT,      // Measurement is still running
  COMPLETE,  // Measurement complete
  FAILED     // Measurement failed (not used here)
} internalERR;

// ------------------ Variables ------------------
int cfmFinalVal = 0;  // Final airflow value to send over LoRa
int CFMcounter = 0;   // Counter for number of airflow pulses in the interval

// Timing variables for measurement interval
unsigned long CFMCurrentTime = 0;
unsigned long CFMPreviousTime = 0;
unsigned long CFMInterval = 1000;  // Interval in ms to calculate CFM (1 second)

// Flags used to control counting logic
bool CFMCalFlag = false;    // Enable measurement counting
bool CFMCountFlag = false;  // Track pulse edges

// LoRa sending interval
unsigned long previousMillis = 0;
const unsigned long loraSendinterval = 1000;  // Send data every 1 second

// ------------------ Setup ------------------
void setup() {
  // Serial.begin(9600);                // Debug serial
  // Initialize LoRa with defined pins
  LoRa.setPins(NSS, RESET, DIO0);

  // Start LoRa at frequency 433 MHz
  if (!LoRa.begin(433E6)) {
    // Serial.println("Starting LoRa failed!");  // Print error if LoRa init fails
  }

  pinMode(CFM_pin, INPUT);                                          // Set airflow sensor pin as input
  attachInterrupt(digitalPinToInterrupt(CFM_pin), CFMISR, CHANGE);  // Attach interrupt to CFM_pin: call CFMISR() on any edge (rising or falling)
}

// ------------------ Main loop ------------------
void loop() {
  // Try to calculate the current CFM value
  if (returnCFM(&cfmFinalVal) == COMPLETE) {
    cfmFinalVal = cfmFinalVal / calibrationFactor;  // Apply calibration to raw value
  }

  // Check if it's time to send data over LoRa
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= loraSendinterval) {
    previousMillis = currentMillis;  // Update last send time
    sendOveLora(cfmFinalVal);        // Send measured value
  }
}

// ------------------ Send data over LoRa ------------------
void sendOveLora(unsigned int cfmFinalVal) {
  LoRa.beginPacket();               // Start a new LoRa packet
  LoRa.print(String(cfmFinalVal));  // Add CFM value as string
  LoRa.endPacket();                 // Send the packet
  // Serial.print("CFM : ");
  // Serial.println(cfmFinalVal);
}

// ------------------ CFM measurement calculation ------------------
internalERR returnCFM(int *cfm) {
  CFMCalFlag = true;  // Enable counting in ISR

  CFMCurrentTime = millis();  // Get current time
  if ((CFMCurrentTime - CFMPreviousTime) >= CFMInterval) {
    CFMPreviousTime = CFMCurrentTime;   // Update time
    double x = (double)CFMcounter / 8;  // Calculate CFM: (number of pulses / 8 blades) * 60 seconds
    *cfm = x * 60;                      // Convert to per-minute flow
    CFMcounter = 0;                     // Reset pulse counter
    return COMPLETE;                    // Measurement complete
  }
  return WAIT;  // Still measuring
}

// ------------------ Interrupt Service Routine ------------------
// Called on every signal edge (HIGH or LOW) of CFM_pin
void CFMISR() {
  if (CFMCalFlag) {  // Only count if measurement is active
    if (digitalRead(CFM_pin) == HIGH) {
      CFMCountFlag = true;  // Remember rising edge
    } else if ((digitalRead(CFM_pin) == LOW) && (CFMCountFlag == true)) {
      CFMCountFlag = false;  // Reset flag on falling edge
      CFMcounter++;          // Count one full pulse
    }
  }
}
