#include <Arduino.h>
#include <TMCStepper.h>

// PID Parameter
const double Kp = 1;
const double Ki = 1;
const double Kd = 1;

double TARGET_HEIGHT = 3000;

uint16_t PRESSURE;

// Stepper
#define TopSW            5
#define BottomSW         6
#define EN_PIN           4 // Enable
#define DIR_PIN          3 // Direction
#define STEP_PIN         2 // Step

#define SERIAL_PORT Serial2 
#define DRIVER_ADDRESS 0b00 

#define R_SENSE 0.11f 

bool shaft = false;
bool TopSWTrig = false;
bool BottomSWTrig = false;

uint16_t ACCEL = 120;
uint16_t ALT = 2000;

// abV1
double CURRENT_TIME;
double timer;
double CURRENT_VELOCITY;
double INIT_VELOCITY;
double LAST_VELOCITY = 0;
double CURRENT_ALTITUDE;
double CURRENT_ACCEL;
double APOGEE;

bool airbreak_check;
bool init_check = false;

long TIMER = 0;
long REAL_TIMER = 0;

double setpoint = 3000;





