#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Ticker.h>

// Create a TinyGPS++ object
TinyGPSPlus gps;

// Use Serial2 for GPS communication
HardwareSerial SerialGPS(1);

// Timer
Ticker gpsTimer;

// PPS Pin (optional)
const int ppsPin = 18;

// Variables to track PPS (optional)
volatile bool ppsTriggered = false;
volatile unsigned long lastPpsTime = 0;

void IRAM_ATTR handlePPS() {
  ppsTriggered = true;
  lastPpsTime = millis();
}

void readGPSData() {
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }

  if (gps.location.isUpdated()) {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Altitude: ");
    Serial.println(gps.altitude.meters());
    Serial.print("Speed: ");
    Serial.println(gps.speed.kmph());
    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());
  }
}

void setup() {
  // Start the Serial Monitor
  Serial.begin(115200);

  // Initialize the Serial1 for GPS communication
  SerialGPS.begin(9600, SERIAL_8N1, 4, 5); // RX1, TX1

  // Initialize the PPS pin (optional)
  pinMode(ppsPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(ppsPin), handlePPS, RISING);

  // Initialize the timer to read GPS data every 60 seconds
  gpsTimer.attach(60, readGPSData);
}

void loop() {
  // Nothing to do here, GPS data is read every minute by the timer
}
