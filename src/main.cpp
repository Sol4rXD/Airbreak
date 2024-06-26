#include <Arduino.h>
#include "definition.h"

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

extern void airbreak_up_auto();
extern void airbreak_down_auto();

extern void airbreak_up();
extern void airbreak_down();

double counter = 2500;

bool closeFirst = false;

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
    if (Serial.available() > 0) {
        int incomingInt = Serial.parseInt();
    
        if (Serial.read() == '\n') {
            Serial.print("Received valid integer: ");
            ACCEL = incomingInt;
            // Serial.println(ACCEL);

            if (ACCEL == 1) {
                driver.shaft(false);
                airbreak_up_auto();
            }
            else if (ACCEL == 0) {
                driver.shaft(true);
                airbreak_down_auto();
            }
        }
    }
    // UP
    if (digitalRead(TopSW) == LOW && !airbreak_check) {

        airbreak_up();
        
        counter -= 1;
        if(counter == 0) {
            airbreak_check = true;
        }

    // DOWN
    } else if (digitalRead(BottomSW) == LOW && airbreak_check) {
        airbreak_down();
        counter += 1;

        if(counter == 2500) {
            airbreak_check = false;
        }
    } 

    if (digitalRead(BottomSW) == LOW && digitalRead(TopSW) == LOW && counter < 2500) {
        closeFirst = true;
    }

    if (digitalRead(BottomSW) == LOW && digitalRead(TopSW) == LOW && closeFirst) {
        driver.shaft(true);

        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(500);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(500);

        counter += 1;

        if(counter == 2500) {
            closeFirst = false;
            airbreak_check = false;
        }

    } else if(digitalRead(BottomSW) == LOW && digitalRead(TopSW) == LOW && !closeFirst) {
        if(!airbreak_check) {
            airbreak_up();
            counter -= 1;

            if(counter == 0) {
                airbreak_check = true;
            }
        } else {
            airbreak_down();
            counter += 1;

            if(counter == 2500) {
                airbreak_check = false;
            }
        }
    }
}

void airbreak_up() {
    driver.shaft(false);

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(500);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(500);
}

void airbreak_down() {
    driver.shaft(true);

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(500);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(500);
}

void airbreak_up_auto() {
    driver.shaft(false);

    for (uint16_t i = 2500; i>0; i--) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(500);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(500);
        //Serial.println("UP");
        Serial.println(i);
    }
}

void airbreak_down_auto() {
    driver.shaft(true);

    for (uint16_t i = 2500; i>0; i--) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(500);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(500);
        //Serial.println("DOWN");
        Serial.println(i);
    }
}