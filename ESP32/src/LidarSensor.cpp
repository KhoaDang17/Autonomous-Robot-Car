#include "LidarSensor.h"
#include "PinConfig.h"

// Cached LiDAR sector clearances (updated as scan packets arrive)
static LidarSectorSummary latest_lidar_summary = {
    MAX_VALID_DISTANCE_CM, // front_cm
    MAX_VALID_DISTANCE_CM, // left_cm
    MAX_VALID_DISTANCE_CM, // right_cm
    false                  // is_valid (becomes true once valid packets are decoded)
};

void initLidarSensor() {
    // Hardware UART1 dedicated to Slamtec RPLIDAR C1 (RX=17, TX=16, 460800 bps)
    Serial1.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
}

LidarSectorSummary readLidarSectors() {
    // TODO (Milestone 6): Decode RPLIDAR C1 Express/Standard Scan packets from Serial1:
    // 1. Send the Start Scan command (0xA5 0x20) during setup if not using an SDK library.
    // 2. Parse angle (deg), distance (mm -> convert to cm), and quality for each point.
    // 3. Track the minimum valid distance per directional sector:
    //      - Front sector: 330..360 deg and 0..30 deg
    //      - Right sector:  30..120 deg
    //      - Left sector:  240..330 deg
    // 4. Set latest_lidar_summary.is_valid = true whenever fresh scan frames arrive.

    return latest_lidar_summary;
}
