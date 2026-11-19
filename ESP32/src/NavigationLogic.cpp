#include "NavigationLogic.h"
#include "PinConfig.h"
#include <Arduino.h>

static float minFloat(float a, float b) {
    return (a < b) ? a : b;
}

MotorCommand computeAutonomousMove(const SensorData& sensor_in) {
    MotorCommand cmd = {0.0f, 0.0f, 0.0f};

    // ------------------------------------------------------------------------
    // Step 1: Multisensor Fusion (3 Trapezoid Ultrasonics + 2D LiDAR)
    // Combine each sector conservatively using min() so obstacles detected by
    // either sensor technology (e.g., glass via Ultrasonic or thin legs via LiDAR)
    // are respected.
    // ------------------------------------------------------------------------
    float effective_front_cm = sensor_in.ultrasonic_front_cm;
    float effective_left_cm  = sensor_in.ultrasonic_left_cm;
    float effective_right_cm = sensor_in.ultrasonic_right_cm;

    if (sensor_in.is_lidar_valid) {
        effective_front_cm = minFloat(effective_front_cm, sensor_in.lidar_front_cm);
        effective_left_cm  = minFloat(effective_left_cm,  sensor_in.lidar_left_cm);
        effective_right_cm = minFloat(effective_right_cm, sensor_in.lidar_right_cm);
    }

    float min_front_zone_cm = minFloat(
        effective_front_cm,
        minFloat(sensor_in.ultrasonic_left_cm, sensor_in.ultrasonic_right_cm)
    );

    // ------------------------------------------------------------------------
    // Step 2: Emergency Escape (Obstacle dangerously close in front trapezoid)
    // ------------------------------------------------------------------------
    if (min_front_zone_cm < EMERGENCY_STOP_DISTANCE_CM) {
        cmd.vx = AVOIDANCE_BACKUP_SPEED; // Back up slightly
        cmd.vy = 0.0f;
        // Rotate toward whichever side has more open space
        cmd.omega = (effective_left_cm > effective_right_cm)
                    ? -AVOIDANCE_TURN_SPEED
                    :  AVOIDANCE_TURN_SPEED;
        return cmd;
    }

    // ------------------------------------------------------------------------
    // Step 3: Front-Center Blocked -> Stop & Turn Toward Greater Open Space
    // ------------------------------------------------------------------------
    if (effective_front_cm < SAFE_FRONT_DISTANCE_CM) {
        cmd.vx = 0.0f;
        cmd.vy = 0.0f;

        if (effective_left_cm > effective_right_cm &&
            effective_left_cm > MIN_SIDE_CLEARANCE_CM) {
            cmd.omega = -AVOIDANCE_TURN_SPEED; // Rotate Left (CCW)
        } else if (effective_right_cm > MIN_SIDE_CLEARANCE_CM) {
            cmd.omega = AVOIDANCE_TURN_SPEED;  // Rotate Right (CW)
        } else {
            // Both sides narrow (dead-end) -> Rotate clockwise to search for exit
            cmd.omega = AVOIDANCE_TURN_SPEED;
        }
        return cmd;
    }

    // ------------------------------------------------------------------------
    // Step 4: Angled Side Obstacle (Detected by Left/Right Trapezoid Sensors)
    // Gently strafe and steer away while slowing down forward speed.
    // ------------------------------------------------------------------------
    if (sensor_in.ultrasonic_left_cm < SAFE_FRONT_DISTANCE_CM &&
        sensor_in.ultrasonic_left_cm <= sensor_in.ultrasonic_right_cm) {
        cmd.vx    = CRUISE_FORWARD_SPEED * 0.5f;
        cmd.vy    = AVOIDANCE_STRAFE_SPEED * 0.6f; // Strafe Right
        cmd.omega = AVOIDANCE_TURN_SPEED * 0.5f;   // Steer Right
        return cmd;
    }

    if (sensor_in.ultrasonic_right_cm < SAFE_FRONT_DISTANCE_CM) {
        cmd.vx    = CRUISE_FORWARD_SPEED * 0.5f;
        cmd.vy    = -AVOIDANCE_STRAFE_SPEED * 0.6f; // Strafe Left
        cmd.omega = -AVOIDANCE_TURN_SPEED * 0.5f;   // Steer Left
        return cmd;
    }

    // ------------------------------------------------------------------------
    // Step 5: Path Clear -> Cruise Forward
    // ------------------------------------------------------------------------
    cmd.vx = CRUISE_FORWARD_SPEED;
    cmd.vy = 0.0f;
    cmd.omega = 0.0f;
    return cmd;
}
