#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_Unified.h>
#include <TinyGPS++.h>
#include <MPU6050.h>
#include <Servo.h>
#include <math.h>
#include <MMC5603NJ.h>

// Sensor and GPS objects
TinyGPSPlus gps;
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
MPU6050 mpu;
MMC5603NJ magnetometer;
Servo esc1, esc2, esc3, esc4;

// PID control variables
float pid_pitch_output, pid_roll_output, pid_altitude_output;
float setpoint_pitch = 0, setpoint_roll = 0, setpoint_altitude = 10.0;
float Kp = 1.0, Ki = 0.0, Kd = 0.0;
float previous_error_pitch = 0, previous_error_roll = 0, previous_error_altitude = 0;
float integral_pitch = 0, integral_roll = 0, integral_altitude = 0;

// Home coordinates and base motor speed
float home_latitude;
float home_longitude;
const int BASE_SPEED = 1500;

// Return to home step
int return_to_home_step = 0;
bool rth_activated = false;

// Sensor fusion variables
float filtered_altitude = 0;
const float alpha = 0.9; // Simple complementary filter constant

// Battery and LED settings
const int batteryPin = A0;
const int ledPin = 7;
const float lowVoltageThreshold = 3.2;
bool firstBlinkIgnored = false;
unsigned long blinkStartTime = 0;

// Calibration states
bool in_flight_calibration = false;
int current_esc_calibration = 0;

// Flight controller signal input pins
const int fc_pin1 = 2;
const int fc_pin2 = 3;
const int fc_pin3 = 18; // Use interrupt-capable pins
const int fc_pin4 = 19;

volatile int fc_signal1, fc_signal2, fc_signal3, fc_signal4;
unsigned long last_fc_signal1, last_fc_signal2, last_fc_signal3, last_fc_signal4;

// Ultimate gain and period for Ziegler-Nichols tuning
float Ku; // Ultimate gain
float Tu; // Oscillation period

void setup() {
    Serial.begin(9600);
    Wire.begin();

    // Initialize GPS
    Serial1.begin(9600);
    while (!gps.location.isValid()) {
        while (Serial1.available() > 0) {
            gps.encode(Serial1.read());
        }
        Serial.print("Waiting for GPS fix...");
        delay(1000);
    }
    home_latitude = gps.location.lat();
    home_longitude = gps.location.lng();
    Serial.print("Home coordinates set: ");
    Serial.print(home_latitude, 6);
    Serial.print(", ");
    Serial.println(home_longitude, 6);

    // Initialize Barometer
    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP085 sensor, check wiring!");
        while (1);
    }

    // Initialize MPU6050
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }

    // Initialize Magnetometer (MMC5603)
    if (!magnetometer.begin()) {
        Serial.println("Could not find a valid MMC5603 sensor, check wiring!");
        while (1);
    }

    // Initialize ESCs
    esc1.attach(9);
    esc2.attach(10);
    esc3.attach(11);
    esc4.attach(12);

    // Calibrate ESCs
    esc1.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000);
    esc4.writeMicroseconds(1000);
    delay(2000);

    // Initialize LED pin
    pinMode(ledPin, INPUT);

    // Attach interrupts for flight controller signals
    attachInterrupt(digitalPinToInterrupt(fc_pin1), readFcSignal1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(fc_pin2), readFcSignal2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(fc_pin3), readFcSignal3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(fc_pin4), readFcSignal4, CHANGE);

    // Automatically find ultimate gain and period
    findUltimateGainAndPeriod();
}

void loop() {
    // Read GPS data
    while (Serial1.available() > 0) {
        gps.encode(Serial1.read());
    }

    // Read Barometer data
    sensors_event_t event;
    bmp.getEvent(&event);
    float current_altitude = bmp.pressureToAltitude(1013.25, event.pressure);

    // Apply complementary filter for altitude estimation
    filtered_altitude = alpha * filtered_altitude + (1 - alpha) * current_altitude;

    // Read IMU data
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Read Magnetometer data for heading
    float x, y, z;
    magnetometer.read(&x, &y, &z);
    float heading = atan2(y, x) * 180 / M_PI;
    if (heading < 0) heading += 360;

    // Convert IMU data to pitch and roll
    float pitch = atan2(ay, az) * 180 / M_PI;
    float roll = atan2(ax, az) * 180 / M_PI;

    // Adaptive PID tuning
    adaptivePIDTuning(filtered_altitude);

    // Calculate PID for pitch
    float error_pitch = setpoint_pitch - pitch;
    integral_pitch += error_pitch;
    float derivative_pitch = error_pitch - previous_error_pitch;
    pid_pitch_output = Kp * error_pitch + Ki * integral_pitch + Kd * derivative_pitch;
    previous_error_pitch = error_pitch;

    // Calculate PID for roll
    float error_roll = setpoint_roll - roll;
    integral_roll += error_roll;
    float derivative_roll = error_roll - previous_error_roll;
    pid_roll_output = Kp * error_roll + Ki * integral_roll + Kd * derivative_roll;
    previous_error_roll = error_roll;

    // Calculate PID for altitude
    float error_altitude = setpoint_altitude - filtered_altitude;
    integral_altitude += error_altitude;
    float derivative_altitude = error_altitude - previous_error_altitude;
    pid_altitude_output = Kp * error_altitude + Ki * integral_altitude + Kd * derivative_altitude;
    previous_error_altitude = error_altitude;

    // Combine signals from flight controller and Arduino
    int motor_speed1 = combineSignals(fc_signal1, pid_pitch_output, -pid_roll_output, pid_altitude_output);
    int motor_speed2 = combineSignals(fc_signal2, pid_pitch_output, pid_roll_output, pid_altitude_output);
    int motor_speed3 = combineSignals(fc_signal3, -pid_pitch_output, pid_roll_output, pid_altitude_output);
    int motor_speed4 = combineSignals(fc_signal4, -pid_pitch_output, -pid_roll_output, pid_altitude_output);

    // Output combined signals to ESCs
    esc1.writeMicroseconds(motor_speed1);
    esc2.writeMicroseconds(motor_speed2);
    esc3.writeMicroseconds(motor_speed3);
    esc4.writeMicroseconds(motor_speed4);

    // Check battery voltage and LED blinking for RTH activation
    checkBatteryAndLED();

    // Call the return_to_home function if activated
    if (rth_activated) {
        return_to_home();
    }

    // Handle in-flight ESC calibration
    if (in_flight_calibration) {
        calibrate_escs_in_flight();
    }
}

void readFcSignal1() {
    if (digitalRead(fc_pin1) == HIGH) {
        last_fc_signal1 = micros();
    } else {
        fc_signal1 = micros() - last_fc_signal1;
    }
}

void readFcSignal2() {
    if (digitalRead(fc_pin2) == HIGH) {
        last_fc_signal2 = micros();
    } else {
        fc_signal2 = micros() - last_fc_signal2;
    }
}

void readFcSignal3() {
    if (digitalRead(fc_pin3) == HIGH) {
        last_fc_signal3 = micros();
    } else {
        fc_signal3 = micros() - last_fc_signal3;
    }
}

void readFcSignal4() {
    if (digitalRead(fc_pin4) == HIGH) {
        last_fc_signal4 = micros();
    } else {
        fc_signal4 = micros() - last_fc_signal4;
    }
}

int combineSignals(int fc_signal, float pitch_output, float roll_output, float altitude_output) {
    int combined_signal = constrain(BASE_SPEED + fc_signal + pitch_output + roll_output + altitude_output, 1000, 2000);
    return combined_signal;
}

void findUltimateGainAndPeriod() {
    // Initial values
    Ki = 0.0;
    Kd = 0.0;
    Kp = 1.0; // Start with a small Kp and gradually increase

    bool oscillating = false;
    unsigned long startTime = millis();
    float lastPitch = 0.0;
    float pitchAmplitude = 0.0;

    while (!oscillating) {
        // Read IMU data and calculate pitch as in the main loop
        int16_t ax, ay, az;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        float pitch = atan2(ay, az) * 180 / M_PI;

        // Calculate error
        float error_pitch = setpoint_pitch - pitch;

        // PID control with current Kp
        float pid_output = Kp * error_pitch;

        // Simulate applying the PID output to the system (adjust motor speeds)
        // ... (apply pid_output to motors)

        // Check for oscillations
        if (pitchAmplitude < fabs(pitch - lastPitch)) {
            pitchAmplitude = fabs(pitch - lastPitch);
        } else {
            // If the pitch amplitude does not increase significantly over time, assume oscillation
            if (millis() - startTime > 2000) { // 2 seconds for observation
                oscillating = true;
                Ku = Kp;
                Tu = millis() - startTime;
            }
        }

        lastPitch = pitch;
        Kp += 0.1; // Gradually increase Kp
        delay(100); // Short delay for stabilization
    }

    // Set Kp, Ki, Kd using Ziegler-Nichols formula
    Kp = 0.6 * Ku;
    Ki = 2 * Kp / Tu;
    Kd = Kp * Tu / 8;
}

void adaptivePIDTuning(float currentAltitude) {
    if (currentAltitude < 5.0) {
        // Low altitude tuning
        Kp = 1.0;
        Ki = 0.5;
        Kd = 0.05;
    } else if (currentAltitude < 10.0) {
        // Medium altitude tuning
        Kp = 1.5;
        Ki = 0.75;
        Kd = 0.1;
    } else {
        // High altitude tuning
        Kp = 2.0;
        Ki = 1.0;
        Kd = 0.2;
    }
}

void checkBatteryAndLED() {
    int batteryValue = analogRead(batteryPin);
    float batteryVoltage = batteryValue * (5.0 / 1023.0);

    if (batteryVoltage < lowVoltageThreshold) {
        if (!firstBlinkIgnored) {
            blinkStartTime = millis();
            firstBlinkIgnored = true;
        }
        if (millis() - blinkStartTime < 1000) {
            digitalWrite(ledPin, HIGH);
        } else {
            digitalWrite(ledPin, LOW);
            rth_activated = true;
        }
    } else {
        digitalWrite(ledPin, LOW);
        firstBlinkIgnored = false;
    }
}

void return_to_home() {
    switch (return_to_home_step) {
        case 0:
            // Ascend to a safe altitude (e.g., 20 meters)
            setpoint_altitude = 20.0;
            if (filtered_altitude >= 19.5) {
                return_to_home_step = 1;
            }
            break;
        case 1:
            // Navigate to home coordinates
            float distance_to_home = gps.distanceBetween(gps.location.lat(), gps.location.lng(), home_latitude, home_longitude);
            float angle_to_home = gps.courseTo(gps.location.lat(), gps.location.lng(), home_latitude, home_longitude);
            setpoint_pitch = cos(angle_to_home * DEG_TO_RAD) * 5.0;
            setpoint_roll = sin(angle_to_home * DEG_TO_RAD) * 5.0;
            if (distance_to_home <= 5.0) {
                return_to_home_step = 2;
            }
            break;
        case 2:
            // Descend to the ground (e.g., 0.5 meters)
            setpoint_altitude = 0.5;
            if (filtered_altitude <= 0.7) {
                return_to_home_step = 3;
            }
            break;
        case 3:
            // Land the drone
            setpoint_pitch = 0.0;
            setpoint_roll = 0.0;
            rth_activated = false; // Reset RTH flag
            break;
    }
}

void calibrate_escs_in_flight() {
    switch (current_esc_calibration) {
        case 0:
            esc1.writeMicroseconds(1000);
            delay(2000);
            current_esc_calibration++;
            break;
        case 1:
            esc2.writeMicroseconds(1000);
            delay(2000);
            current_esc_calibration++;
            break;
        case 2:
            esc3.writeMicroseconds(1000);
            delay(2000);
            current_esc_calibration++;
            break;
        case 3:
            esc4.writeMicroseconds(1000);
            delay(2000);
            in_flight_calibration = false; // Calibration done
            break;
    }
}
