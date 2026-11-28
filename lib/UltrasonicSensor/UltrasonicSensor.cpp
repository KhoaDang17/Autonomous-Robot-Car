#include "UltrasonicSensor.h"

void initUltrasonicSensors() {
    for (uint8_t i = 0; i < NUM_ULTRASONIC_SENSORS; i++) {
        pinMode(TRIG_PINS[i], OUTPUT);
        pinMode(ECHO_PINS[i], INPUT);
        digitalWrite(TRIG_PINS[i], LOW);
    }
}

float readSingleUltrasonicCm(uint8_t sensor_index) {
    if (sensor_index >= NUM_ULTRASONIC_SENSORS) {
        return MAX_VALID_DISTANCE_CM;
    }

    int trig_pin = TRIG_PINS[sensor_index];
    int echo_pin = ECHO_PINS[sensor_index];

    // Send a 10-microsecond HIGH trigger pulse
    digitalWrite(trig_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_pin, LOW);

    // Measure echo pulse width in microseconds (bounded by timeout)
    unsigned long duration_us = pulseIn(echo_pin, HIGH, ULTRASONIC_TIMEOUT_US);

    // Treat missing echo (0 us) or out-of-range readings as clear space (Milestone 5)
    if (duration_us == 0) {
        return MAX_VALID_DISTANCE_CM;
    }

    // Distance (cm) = (time_us * speed_of_sound_0.0343_cm_per_us) / 2
    float distance_cm = (duration_us * 0.0343f) / 2.0f;

    if (distance_cm <= 0.0f || distance_cm > MAX_VALID_DISTANCE_CM) {
        return MAX_VALID_DISTANCE_CM;
    }

    return distance_cm;
}

UltrasonicArrayData readAllUltrasonicSensors() {
    UltrasonicArrayData result;

    // Trigger sensors one by one with a short pause to prevent echo cross-talk
    result.left_cm = readSingleUltrasonicCm(US_LEFT_IDX);
    delay(ULTRASONIC_INTER_PING_MS);

    result.center_cm = readSingleUltrasonicCm(US_CENTER_IDX);
    delay(ULTRASONIC_INTER_PING_MS);

    result.right_cm = readSingleUltrasonicCm(US_RIGHT_IDX);

    return result;
}

bool isObstacleDetected(float distance_cm, float threshold_cm) {
    return (distance_cm > 0.0f && distance_cm < threshold_cm);
}
