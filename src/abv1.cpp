#include <Arduino.h>
#include "definition.h"

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

extern double calculate_velocity(double velo, double accel, double timer);
extern double calculate_apogee(double time, double velo, double accel, double altitude);
extern double calculate_time(double velo, double accel);

extern void airbreak_up();
extern void airbreak_down();
extern void init_clock();
extern void elapse_timer();
extern void apogee_generate();

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
    init_clock();
    elapse_timer();
    apogee_generate();

    if(APOGEE >= setpoint && !airbreak_check) {
        distance = APOGEE - setpoint;

        unsigned long timer_airbreak = millis();
        
        airbreak_up();    

        if(!airbreak_break) {
            airbreak_check = true;
        }  
    }   
    else if(APOGEE < setpoint && airbreak_check) {
        airbreak_down();
        airbreak_check = false;
    }
}

void apogee_generate() {
    CURRENT_VELOCITY = calculate_velocity(LAST_VELOCITY, CURRENT_ACCEL, REAL_TIMER);
    LAST_VELOCITY = CURRENT_VELOCITY;

    CURRENT_TIME = calculate_time(CURRENT_VELOCITY, CURRENT_ACCEL);
    APOGEE = calculate_apogee(CURRENT_TIME, CURRENT_VELOCITY, CURRENT_ACCEL, CURRENT_ALTITUDE);
}

void init_clock() {
    if(CURRENT_ACCEL >= 300 && !init_check) {
        TIMER = millis();
        init_check = true;
    }
}

void elapse_timer() {
    REAL_TIMER = millis();
    REAL_TIMER = REAL_TIMER - TIMER;
}

double calculate_velocity(double velo, double accel, double timer) {
    double velocity = velo + (accel * timer);
    return velocity;
}

double calculate_time(double velo, double accel) {
    double v = 0.0; 
    double time = (v - velo) / accel; 
    return abs(time);
}

// Using calculate time 
double calculate_apogee(double time, double velo, double accel, double altitude) {
    double initial_altitude = altitude; 

    double apogee = initial_altitude + (velo * time) + (0.5 * accel * time * time);
    return apogee;
}

void airbreak_up() {
    driver.shaft(false);

    for (uint16_t i = step; i>0; i--) {
        apogee_generate();

        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(500);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(500);
        Serial.println(i);

        if(APOGEE <= (distance / 2 + setpoint)) {
            step = step - i;
            airbreak_break = true;
            break;
        }
    }

    if(airbreak_break) {
        driver.shaft(true);

        for (uint16_t i = step; i>0; i--) {
            digitalWrite(STEP_PIN, HIGH);
            delayMicroseconds(500);
            digitalWrite(STEP_PIN, LOW);
            delayMicroseconds(500);
            Serial.println(i);
        }
        airbreak_check = false;
    }
}

void airbreak_down() {
    driver.shaft(true);

    for (uint16_t i = step; i>0; i--) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(500);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(500);
        Serial.println(i);
    }
}


