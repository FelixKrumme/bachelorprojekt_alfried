#include <Arduino.h>
#include <SD.h>
#include <ADSWeather.h>

// Activate Serial Output over USB
#define DEBUGGING


#define ANEMOMETER_PIN A0
#define VANE_PIN A1
#define MEASUREMENT_PIN A2
#define MOSFET1 0
#define MOSFET2 1
#define MOSFET3 2
#define MOSFET4 3
#define MOSFET5 7
#define MOSFET6 6
#define MOSFET7 8
#define MOSFET8 9

// Timing for Windssensor
#define CALC_INTERVAL_SENSOR 1000
// Timing for Hill-Climbing
#define CALC_INTERVAL_RESISTOR 10

unsigned long nextCalcSensor;
unsigned long nextCalcResistance;
unsigned long timer;

// Variables for comparing and saving voltages
int old_power;
int new_power;

// Variables for controlling the MOSFETs State to determine which MOSFETS are on,
// rising cyclye to determine if the loop is in an ascending run or descending.
//  MOSFETPINS for correct Mapping
int state;
bool rising_res_cycle;
const char MOSFETPINS[8] = {MOSFET1, MOSFET2, MOSFET3, MOSFET4, MOSFET5, MOSFET6, MOSFET7, MOSFET8};



int calculate_power(int voltage, int state_i);

int count_up(int state_i);

int count_down(int state_i);

void switch_transistors(int state_i);



ADSWeather adsWeather(VANE_PIN, ANEMOMETER_PIN);


void setup() {
    // Use a Higher Resolution for the ADCs (8 would be standard)
    analogReadResolution(12);

    // Interrupt for Windspeed Measurement
    attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), adsWeather.countAnemometer,
                    FALLING); //.countAnemometer is the ISR for the anemometer.

    nextCalcSensor = millis() + CALC_INTERVAL_SENSOR;

    // Look if the SD-Card is reachable
    if (!SD.begin(SDCARD_SS_PIN)) {
        pinMode(1, OUTPUT);
        digitalWrite(1, HIGH);
    } else {
        if (SD.exists("datalog.txt")) {
            File dataFile = SD.open("datalog.txt", FILE_WRITE);
            dataFile.println("New Initialization");
            dataFile.close();
        }
    }
    pinMode(MOSFET1, OUTPUT);
    pinMode(MOSFET2, OUTPUT);
    pinMode(MOSFET3, OUTPUT);
    pinMode(MOSFET4, OUTPUT);
    pinMode(MOSFET5, OUTPUT);
    pinMode(MOSFET6, OUTPUT);
    pinMode(MOSFET7, OUTPUT);
    pinMode(MOSFET8, OUTPUT);
    pinMode(MEASUREMENT_PIN, INPUT);

    state = 255;
    rising_res_cycle = true;

#ifdef DEBUGGING
    Serial.begin(9600);
#endif
}

void loop() {
    timer = millis();

    long windDirection;
    long windSpeed;
    int windGust;

    String dataString = "";

    // Update Sensor Values, call as often as possible.
    adsWeather.update();

    //
    if(timer > nextCalcResistance) {
        // Calculate the next time to change the Resistor Cascade
        nextCalcResistance = timer + CALC_INTERVAL_RESISTOR;
        // Calculate the current generated Power, with the current state and the new measured voltage.
        new_power = calculate_power(analogRead(MEASUREMENT_PIN), state);

        // Hill-Climbing Decision
        if(new_power > old_power) {
            if(rising_res_cycle){
                state = count_down(state);
            } else{
                state = count_up(state);
            }
        } else if(new_power < old_power) {
            if (rising_res_cycle) {
                state = count_up(state);
            } else {
                state = count_down(state);
            }
            rising_res_cycle = (!rising_res_cycle);
        }

        // Switch the Output Pins leading to the MOSFETs accordingly
        switch_transistors(state);
        // Save the current power for the next iteration
        old_power = new_power;
    }

    if (timer > nextCalcSensor) {
        // Calc the next time to measure again
        nextCalcSensor = timer + CALC_INTERVAL_SENSOR;

        // Get the Windinfos
        windSpeed = adsWeather.getWindSpeed();
        windDirection = adsWeather.getWindDirection();
        windGust = adsWeather.getWindGust();

        // Generate one string to be written to SD-Card
        dataString +=
                String(windSpeed) + "," + String(windGust) + "," + String(windDirection) + "," + String(new_power) +
                "," + String(state);

        // Open the File and Write to it
        File dataFile = SD.open("datalog.txt", FILE_WRITE);
        if (dataFile) {
            dataFile.println(dataString);
            dataFile.close();
        }

    #ifdef DEBUGGING
        Serial.println(dataString);
    #endif
    }

}
int calculate_power(int voltage, int state_i){
    // Calculates the electrical power from a voltage and a resistance, by Using the Formula P = U*I where I = U/R
    return (voltage * voltage / state_i);
}

int count_up(int state_i) {
    //Increase the State as long as it's less than 255
    if(state_i == 255){
        return 255;
    }
    return (state_i + 1);
}

int count_down(int state_i) {
    //Decrease the State as long as it's greater than zero
    if(state_i == 0){
        return 0;
    }
    return (state_i - 1);
}

void switch_transistors(int state_i) {
    //Calculate the binary of the State and switch the Mosfets accordingly (255 all on, 254 all on but the first off...)
    for (char i : MOSFETPINS) {
        if (state_i % 2) {
            digitalWrite(i, HIGH);
        } else {
            digitalWrite(i, LOW);
        }
        state_i = state_i / 2;
    }
}
