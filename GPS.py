#include <Adafruit_GPS.h>
#include <SPI.h>
#include "SdFat.h"
#include <Adafruit_NeoPixel.h>

//  Pins 
#define GPSSerial Serial1
#define SD_CS_PIN 23
#define VBATPIN A3
#define NEOPIXEL_PIN PIN_NEOPIXEL

//  Objects 
Adafruit_GPS GPS(&GPSSerial);
SdFat SD;
FsFile logFile;
Adafruit_NeoPixel pixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);

//  Globals 
uint32_t timer = 0;
uint32_t batteryTimer = 0;
const long BATTERY_INTERVAL = 5000;
String filename = "gps_temp.csv";
bool renamed = false;

// Battery % 
int batteryPercent(float v) {
  if (v >= 4.15) return 100;
  if (v >= 4.00) return 90;
  if (v >= 3.85) return 75;
  if (v >= 3.70) return 50;
  if (v >= 3.55) return 25;
  if (v >= 3.40) return 10;
  return 0;
}

//  NeoPixel % color
void showBatteryStatus(int percent) {
  static bool ledOn = false;
  static uint32_t lastBlink = 0;
  const uint16_t blinkInterval = 1000;  // 1 second blink interval

  uint32_t color;
  if (percent >= 75) color = pixel.Color(0, brightness, 0);         // Green
  else if (percent >= 50) color = pixel.Color(brightness, brightness, 0); // Yellow
  else if (percent >= 25) color = pixel.Color(brightness, brightness / 2, 0); // Orange
  else color = pixel.Color(brightness, 0, 0);                       // Red

  if (millis() - lastBlink > blinkInterval) {
    ledOn = !ledOn;
    lastBlink = millis();
    if (ledOn) {
      pixel.setPixelColor(0, color);
    } else {
      pixel.setPixelColor(0, 0);  // Off
    }
    noInterrupts(); pixel.show(); interrupts();
  }

  Serial.print("Battery Percent: ");
  Serial.print(percent);
  Serial.println("%");
}

// Read battery voltage from A3 
float readBatteryVoltage() {
  float v = analogRead(VBATPIN);
  v /= 1023.0;
  v *= 3.3;     // ADC reference voltage (on RP2040, 3.0V)
  v *= 2.0;     // Voltage divider (100k / 100k)
  return v;
}

// ----- Convert UTC to EST -----
int convertToEST(int utcHour) {
  int est = utcHour - 4;
  if (est < 0) est += 24;
  return est;
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Starting GPS Logger with Battery Monitoring...");

  // NeoPixel setup
  pixel.begin();
  pixel.setBrightness(80);
  pixel.setPixelColor(0, 0); noInterrupts(); pixel.show(); interrupts();

  // Start GPS
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);

  // Init SD card
  Serial.print("Initializing SD card... ");
  if (!SD.begin(config)) {
    Serial.println("FAILED");
    pixel.setPixelColor(0, pixel.Color(255, 0, 0)); // red = SD error
    noInterrupts(); pixel.show(); interrupts();
    while (1);
  }
  Serial.println("OK");

  // Create initial file
  if (!SD.exists(filename.c_str())) {
    logFile = SD.open(filename.c_str(), FILE_WRITE);
    if (logFile) {
      logFile.println("Time(EST),Date,Fix,Lat,Lon,Speed(knots),Angle,Altitude,Satellites");
      logFile.close();
      Serial.println("Temp log file created: gps_temp.csv");
    }
  }

  timer = millis();
  batteryTimer = millis();
}

void loop() {
  char c = GPS.read();

  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) return;

    // Rename file using GPS RTC
    if (!renamed && GPS.hour > 0 && GPS.day > 0 && GPS.month > 0) {
      int estHour = convertToEST(GPS.hour);
      char newName[32];
      snprintf(newName, sizeof(newName), "%02d_%02d_20%02d_%02d_%02d.csv",
        GPS.month, GPS.day, GPS.year, estHour, GPS.minute);

      if (!SD.exists(newName)) {
        Serial.print("Renaming gps_temp.csv to ");
        Serial.println(newName);
        SD.rename("gps_temp.csv", newName);
        filename = String(newName);
      } else {
        Serial.println("Filename already exists. Keeping gps_temp.csv.");
      }
      renamed = true;
    }
  }

  // GPS logging every 2s
  if (millis() - timer > 2000) {
    timer = millis();
    if (GPS.fix) {
      int estHour = convertToEST(GPS.hour);

      Serial.print("\nTime (EST): ");
      if (estHour < 10) Serial.print('0');
      Serial.print(estHour); Serial.print(':');
      if (GPS.minute < 10) Serial.print('0');
      Serial.print(GPS.minute); Serial.print(':');
      if (GPS.seconds < 10) Serial.print('0');
      Serial.print(GPS.seconds); Serial.print('.');
      Serial.println(GPS.milliseconds);

      Serial.print("Date: ");
      Serial.print(GPS.day); Serial.print('/');
      Serial.print(GPS.month); Serial.print("/20");
      Serial.println(GPS.year);

      Serial.print("Fix: "); Serial.print((int)GPS.fix);
      Serial.print(" quality: "); Serial.println((int)GPS.fixquality);

      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);

      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      Serial.print("Antenna status: "); Serial.println((int)GPS.antenna);

      logFile = SD.open(filename.c_str(), FILE_WRITE);
      if (logFile) {
        if (estHour < 10) logFile.print('0');
        logFile.print(estHour); logFile.print(':');
        if (GPS.minute < 10) logFile.print('0');
        logFile.print(GPS.minute); logFile.print(':');
        if (GPS.seconds < 10) logFile.print('0');
        logFile.print(GPS.seconds); logFile.print('.');
        logFile.print(GPS.milliseconds); logFile.print(',');

        logFile.print(GPS.day); logFile.print('/');
        logFile.print(GPS.month); logFile.print("/20");
        logFile.print(GPS.year); logFile.print(',');

        logFile.print((int)GPS.fix); logFile.print(',');
        logFile.print(GPS.latitude, 6); logFile.print(GPS.lat); logFile.print(',');
        logFile.print(GPS.longitude, 6); logFile.print(GPS.lon); logFile.print(',');
        logFile.print(GPS.speed, 2); logFile.print(',');
        logFile.print(GPS.angle, 2); logFile.print(',');
        logFile.print(GPS.altitude, 2); logFile.print(',');
        logFile.println((int)GPS.satellites);
        logFile.close();
      } else {
        Serial.println("Error writing to log file!");
      }
    } else {
      Serial.println("Waiting for GPS fix...");
    }
  }

  // Battery check every 5s
  if (millis() - batteryTimer > BATTERY_INTERVAL) {
    batteryTimer = millis();
    float voltage = readBatteryVoltage();
    int percent = batteryPercent(voltage);

    Serial.print("Battery Voltage: ");
    Serial.print(voltage, 2);
    Serial.println(" V");

    showBatteryStatus(percent);
  }
}


