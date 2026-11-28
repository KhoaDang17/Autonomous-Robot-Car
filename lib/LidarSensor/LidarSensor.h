#ifndef LIDAR_SENSOR_H
#define LIDAR_SENSOR_H

#include <Arduino.h>

// Sector clearance summary extracted from a 360-degree 2D LiDAR scan (cm)
struct LidarSectorSummary {
    float front_cm;  // Minimum obstacle distance in the forward cone
    float left_cm;   // Minimum obstacle distance on the left flank
    float right_cm;  // Minimum obstacle distance on the right flank
    bool is_valid;   // True when recent RPLIDAR C1 packets have been parsed
};

// Initialize Hardware Serial1 for the Slamtec RPLIDAR C1 at LIDAR_BAUD (460800 bps).
void initLidarSensor();

// Poll and parse incoming RPLIDAR C1 scan points into directional sectors.
LidarSectorSummary readLidarSectors();

#endif // LIDAR_SENSOR_H
