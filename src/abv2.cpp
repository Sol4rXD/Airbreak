#include <Arduino.h>
#include "definition.h"

void setup() {
    Serial.begin(115200);
}

void loop() {

}

double calculate_time(double velo, double accel) {
    double v = 0.0; 
    double time = (v - velo) / accel; 
    return time;
}

double calculate_apogee(double time, double velo, double accel, double altitude) {
    double initial_altitude = altitude; 
    double u = 0.0; 

    double apogee = initial_altitude + (velo * time) + (0.5 * accel * time * time);
    return apogee;
}

void airbreak_up() {
    driver.shaft(false);

    for (uint16_t i = 2500; i>0; i--) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(500);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(500);
        Serial.println(i);
    }
}

void airbreak_down() {
    driver.shaft(true);

    for (uint16_t i = 2500; i>0; i--) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(500);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(500);
        Serial.println(i);
    }
}


