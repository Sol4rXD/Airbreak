#include <Arduino.h>
#include "definition.h"

void setup() {
    Serial.begin(115200);

    pinMode(TopSW, INPUT_PULLUP);
    pinMode(BottomSW, INPUT_PULLUP);

    pinMode(EN_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW);      

    SERIAL_PORT.begin(115200);      // HW UART drivers

    driver.begin();                 //  SPI: Init CS pins and possible SW SPI pins
    driver.toff(5);                 // Enables driver in software
    driver.rms_current(0.961);        // Set motor RMS current
    driver.microsteps(0);           // Set microsteps to 1/16th

    driver.pwm_autoscale(true);     // Needed for stealthChop
}

void loop() {
    CURRENT_TIME = calculate_time(CURRENT_VELOCITY, CURRENT_ACCEL);
    APOGEE = calculate_apogee(CURRENT_TIME, CURRENT_VELOCITY, CURRENT_ACCEL, CURRENT_ALTITUDE);

    if(APOGEE >= 3200; airbreak_check = false) {
        airbreak_up();
        airbreak_check = true;
    }
    else if(APOGEE < 2800; airbreak_check = true) {
        airbreak_down();
        airbreak_check = false;
    }
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


