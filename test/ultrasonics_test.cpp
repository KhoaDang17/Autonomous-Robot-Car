#include <Arduino.h>
#include <cmath>

// constants
constexpr uint8_t NUM_SENSORS = 3;
constexpr float SOUND_SPEED_CM_US = 0.0343f;     // Speed of sound 
constexpr unsigned long ECHO_TIMEOUT_US = 25000; // range timeout
constexpr unsigned long CROSSTALK_DELAY_MS = 15; // Wait for echoes to dissipate

struct UltrasonicSensor {
  const char* name;
  uint8_t trigPin;
  uint8_t echoPin;
  float angleDeg;     // Mounting offset from center (-15, 0, +15)
  float distanceCm;   // Direct radial distance (-1.0 if out of range)
  float xCm;          // Lateral offset (+ is right, - is left)
  float yCm;          // Forward distance straight ahead
};

// Initialize Left (-15°), Center (0°), and Right (+15°) sensors
UltrasonicSensor sensors[NUM_SENSORS] = {
  {"Left",   19, 20, -15.0f, -1.0f, 0.0f, 0.0f},
  {"Center", 18, 6,   0.0f, -1.0f, 0.0f, 0.0f},
  {"Right",  3, 2,  15.0f, -1.0f, 0.0f, 0.0f}
};

void readSensor(UltrasonicSensor& sensor) {
  digitalWrite(sensor.trigPin, LOW);
  delayMicroseconds(2);

  // Send a 15-microsecond pulse to trigger the ultrasonic sensor
  digitalWrite(sensor.trigPin, HIGH);
  delayMicroseconds(15);
  digitalWrite(sensor.trigPin, LOW);

  // Measure echo duration with timeout so a missed echo won't freeze the loop
  unsigned long duration = pulseIn(sensor.echoPin, HIGH, ECHO_TIMEOUT_US);

  if (duration == 0) {
    sensor.distanceCm = -1.0f; // Out of range or no echo received
    sensor.xCm = 0.0f;
    sensor.yCm = 0.0f;
    return;
  }

  // Calculate radial distance (divide by 2 for round-trip travel)
  sensor.distanceCm = (duration * SOUND_SPEED_CM_US) * 0.5f;

  // Project onto 2D plane using the 15-degree mounting offset
  float rad = sensor.angleDeg * DEG_TO_RAD;
  sensor.xCm = sensor.distanceCm * sinf(rad);
  sensor.yCm = sensor.distanceCm * cosf(rad);
}

void setup() {
  Serial.begin(115200);
  delay(3000); // Allow time for serial monitor to open
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensors[i].trigPin, OUTPUT);
    pinMode(sensors[i].echoPin, INPUT);
    digitalWrite(sensors[i].trigPin, LOW);
  }
}

void loop() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    readSensor(sensors[i]);

    // Pause between pings so residual sound waves die down before the next sensor fires
    delay(CROSSTALK_DELAY_MS);
  }

  // Print formatted readings
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensors[i].name);
    Serial.print(" (");
    Serial.print(sensors[i].angleDeg, 0);
    Serial.print(" deg): ");

    if (sensors[i].distanceCm < 0) {
      Serial.print("Out of range");
    } else {
      Serial.print(sensors[i].distanceCm, 1);
      Serial.print(" cm [X=");
      Serial.print(sensors[i].xCm, 1);
      Serial.print(", Y=");
      Serial.print(sensors[i].yCm, 1);
      Serial.print("]");
    }

    if (i < NUM_SENSORS - 1) {
      Serial.print("  |  ");
    }
  }
  Serial.println();
}
