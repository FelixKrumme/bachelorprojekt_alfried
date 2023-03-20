#include <Arduino.h>
#include <SD.h>
#include <RTCZero.h>
#include <ADSWeather.h>
#include <cmath>
#include <bitset>

// Activate Serial Output over USB
// #define DEBUGGING

//Ax Pins for using the Wind Measurement, Digtal Pins for controlling the MOSFETS (via the Logic Level Shifter)
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

// Possible Options depending where the Jumper is placed 1; .27; .132; .055
#define VOLTAGE_DIVIDER 1

// Timeframe (ms) for Wind-sensor calculation and writing to the SD-Card
#define CALC_INTERVAL_SENSOR 1000
// Timeframe (ms) for the Hill-Climbing Algorithm thus the change of resistance
#define CALC_INTERVAL_RESISTOR 100

// Variables for the Timeframe Calculations
unsigned long nextCalcSensor;
unsigned long nextCalcResistance;
unsigned long timer;

// Variables for comparing and saving generated power
double old_power;
double new_power;

// Variable to map from a State to the current resistance and to determine which MOSFETs are on
// rising_res_cycle to determine if the Hill-Climb is climbing or descending
//  MOSFETPINS for correct Mapping to Outputs
int state;
bool rising_res_cycle;
const char MOSFETPINS[8] = {MOSFET1, MOSFET2, MOSFET3, MOSFET4, MOSFET5, MOSFET6, MOSFET7, MOSFET8};

// Time object for using the clock
RTCZero rtc;
/* Change these values to set the current initial time */
const byte seconds = 0;
const byte minutes = 0;
const byte hours = 0;
/* Change these values to set the current initial date */
const byte day = 5;
const byte month = 3;
const byte year = 22;

double calculate_power(double voltage, int state_i);

int count_up(int state_i);

int count_down(int state_i);

void switch_transistors(int state_i);

// Initialize the Class for the Weather Station
ADSWeather adsWeather(VANE_PIN, ANEMOMETER_PIN);

void setup() {
    // Use a Higher Resolution for the ADCs (8 would be standard)
    analogReadResolution(12);
    // Interrupt for Wind speed Measurement
    attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), adsWeather.countAnemometer,
                    FALLING); //.countAnemometer is the ISR for the anemometer.
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
    // Initialize Output Pins
    pinMode(MOSFET1, OUTPUT);
    pinMode(MOSFET2, OUTPUT);
    pinMode(MOSFET3, OUTPUT);
    pinMode(MOSFET4, OUTPUT);
    pinMode(MOSFET5, OUTPUT);
    pinMode(MOSFET6, OUTPUT);
    pinMode(MOSFET7, OUTPUT);
    pinMode(MOSFET8, OUTPUT);
    pinMode(MEASUREMENT_PIN, INPUT);
    // Starting value for the Hill Climb, (255 equals lowest possible resistance, thus we try to climb)
    state = 255;
    rising_res_cycle = true;
    // initialize RTC, set Time and Date
    rtc.begin();
    rtc.setTime(hours, minutes, seconds);
    rtc.setDate(day, month, year);
    // Calculate the next time to write to the SD-Card and calculate Wind-Speed
    nextCalcSensor = millis() + CALC_INTERVAL_SENSOR;

#ifdef DEBUGGING
    Serial.begin(9600);
#endif
}

void loop() {
    timer = millis();

    long windDirection;
    long windSpeed;
    int windGust;

    String dataString = ""; // For Outputting

    // Update Sensor Values, call as often as possible.
    adsWeather.update();

    if (timer > nextCalcResistance) {
        // Calculate the next time to change the Resistor Cascade.
        nextCalcResistance = timer + CALC_INTERVAL_RESISTOR;
        // Calculate the current generated Power, with the current state and the new measured voltage.
        double volt = analogRead(MEASUREMENT_PIN) * 3.3 / 4095;
        new_power = calculate_power(volt, state);

        // Hill-Climbing Decision, if the previous climb/fall was useful continue, else turn in the other direction.
        if (new_power > old_power) {
            if (rising_res_cycle) {
                state = count_down(state);
            } else {
                state = count_up(state);
            }
        } else if (new_power < old_power) {
            if (rising_res_cycle) {
                state = count_up(state);
            } else {
                state = count_down(state);
            }
            rising_res_cycle = (!rising_res_cycle);
        }

        // Switch the MOSFETs according to the previous made decision.
        switch_transistors(state);
        // Save the current power for the next decision in the next iteration.
        old_power = new_power;
    }

    if (timer > nextCalcSensor) {
        // Calc the next time to measure again
        nextCalcSensor = timer + CALC_INTERVAL_SENSOR;

        // Get the Windinfos
        windSpeed = adsWeather.getWindSpeed();
        windDirection = adsWeather.getWindDirection();
        windGust = adsWeather.getWindGust();
        // Calculate voltage.
        double voltage = analogRead(MEASUREMENT_PIN) * 3.3 / 4095;
        // Generate one string to be written to SD-Card
        dataString +=
                String(windSpeed) + "," + String(windGust) + "," + String(windDirection) + "," + new_power +
                "," + String(state) + "," + voltage + "," + rtc.getMonth() + "/" + rtc.getDay() + "," + rtc.getHours() +
                ":" + rtc.getMinutes() + ":" + rtc.getSeconds();
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

double calculate_power(double voltage, int state_i) {
    /** Calculates the power corresponding to the given voltage with the resistance given by a State. Therefore first
     * calculates the corresponding Resistance. (a MOSFET has a Resistance of arround 20mOhms if turned on). **/
    double resistance = 0;
    voltage = voltage / VOLTAGE_DIVIDER;
    std::bitset<8> bin_x(state_i);
    // If i is one the resistance of the MOSFET is taken else the resistance of the corresponding resistor given by 2^i.
    for (int i = 0; i < 8; i++) {
        if (bin_x[0] == 1) {
            resistance += 0.02;
        } else {
            resistance += pow(2, i);
        }
        bin_x >>= 1;
    }
    return (pow(voltage, 2) / resistance);
}

int count_up(int state_i) {
    /** Increase the State state_i by one as long as it's less than 255. **/
    if (state_i == 255) {
        return 255;
    }
    return (state_i + 1);
}

int count_down(int state_i) {
    /** Decrease the State state_i by one as long as it's greater than 0. **/
    if (state_i == 0) {
        return 0;
    }
    return (state_i - 1);
}

void switch_transistors(int state_i) {
    /** Switch the MOSFETs according to the State state_i, for state_i = 0 all MOSFETs should be off thus biggest
     * resistance possible, for state_i = 255 all should be on thus lowest resistance possible. **/
    std::bitset<8> bin_x(state_i);
    for (int i = 0; i < 8; i++) {
        if (bin_x[i] == 1) {
            digitalWrite(MOSFETPINS[i], HIGH);
        } else {
            digitalWrite(MOSFETPINS[i], LOW);
        }
    }
}
