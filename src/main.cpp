#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include <SimpleKalmanFilter.h>
#include <WMM_Tinier.h>
#include "SensorBMM150.hpp"
#include "SensorWireHelper.h"

#ifndef SENSOR_SDA
#define SENSOR_SDA 2  // Zmiana na GPIO2
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL 5  // Zmiana na GPIO5
#endif

#ifndef SENSOR_IRQ
#define SENSOR_IRQ -1
#endif

#ifndef SENSOR_RST
#define SENSOR_RST -1
#endif

#define BMM150_I2C_ADDRESS 0x13  // Zmiana na adres 0x13

SensorBMM150 bmm;

// Inicjalizacja filtru Kalmana dla osi X i Y magnetometru
SimpleKalmanFilter kalmanFilterX(1.0, 1.0, 0.01);
SimpleKalmanFilter kalmanFilterY(1.0, 1.0, 0.01);

// Inicjalizacja WMM_Tinier
WMM_Tinier wmm;

// Offsety magnetometru
float magnetometerXOffset = 0.0;
float magnetometerYOffset = 0.0;
float magnetometerZOffset = 0.0;

// Funkcja kalibracji magnetometru (tylko do ustawienia offsetów)
void calibrateMagnetometer() {
    const int numSamples = 1000; // Liczba próbek do pobrania

    float sumX = 0.0, sumY = 0.0, sumZ = 0.0;
    for (int i = 0; i < numSamples; ++i) {
        int16_t mx, my, mz;
        bmm.getMag(mx, my, mz);

        sumX += mx;
        sumY += my;
        sumZ += mz;

        delay(10); // Opóźnienie między próbkami
    }

    // Obliczanie średniej wartości dla offsetów
    magnetometerXOffset = sumX / numSamples;
    magnetometerYOffset = sumY / numSamples;
    magnetometerZOffset = sumZ / numSamples;
}

void setup() {
    Serial.begin(115200);
    while (!Serial);

    // Using I2C interface
    if (!bmm.init(Wire, SENSOR_SDA, SENSOR_SCL, BMM150_I2C_ADDRESS)) {
        Serial.print("Failed to init BMM150 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }

    Serial.println("Init BMM150 Sensor success!");

    // Set magnetometer run mode
    bmm.setMode(SensorBMM150::POWERMODE_NORMAL);

    // Kalibracja magnetometru
    Serial.println("Starting magnetometer calibration...");
    calibrateMagnetometer();
    Serial.println("Magnetometer calibration completed");
}

void loop() {
    Serial.println("Loop start");
    int16_t mx, my, mz;
    if (bmm.getMag(mx, my, mz)) {
        // Korekta pomiarów magnetometru na podstawie offsetów
        float correctedMX = mx - magnetometerXOffset;
        float correctedMY = my - magnetometerYOffset;
        float correctedMZ = mz - magnetometerZOffset;

        //float filteredX = kalmanFilterX.updateEstimate(correctedMX);
        //float filteredY = kalmanFilterY.updateEstimate(correctedMY);
        float filteredX = correctedMX;
        float filteredY = correctedMY;

        // Założenie, że dane geograficzne i daty są dostępne
        float latitude = 52.4064;  // Szerokość dla Poznania
        float longitude = 16.9252;  // Długość dla Poznania
        uint16_t year = 2023;
        uint8_t month = 4, day = 15;  // Przykładowa data

        float declinationAngle = wmm.magneticDeclination(latitude, longitude, year, month, day);
        float heading = atan2(filteredY, filteredX);
        heading += declinationAngle * PI / 180.0;
        if (heading < 0) heading += 2 * PI;
        if (heading > 2 * PI) heading -= 2 * PI;

        float headingDegrees = heading * 180 / PI;
        Serial.print("Heading: ");
        Serial.print(headingDegrees);
        Serial.println(" degrees");
    } else {
        Serial.println("Sensor data unavailable");
    }
    Serial.println("Loop end");
    delay(100);  // Skrócenie opóźnienia w pętli
}
