#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include "pico/stdlib.h"
#include <array>

class LineSensor {
public:
    // Constructor: takes array of GPIO pins for the 4 sensor outputs
    LineSensor(const std::array<uint, 4>& pins, bool active_low = true);
    
    // Initialize all sensors
    void init();
    
    // Read all 4 sensors at once
    std::array<bool, 4> read_all() const;
    
    // Read individual sensor
    bool read_sensor(uint index) const;
    
    // Get sensor values as a 4-bit pattern (useful for line following algorithms)
    uint get_pattern() const;
    
    // Calibration functions
    void calibrate_white();
    void calibrate_black();
    
    // Get calibrated threshold (for debugging)
    uint16_t get_calibrated_threshold(uint8_t index) const;
    
    // Sensor information
    uint8_t get_num_sensors() const { return 4; }
    bool is_active_low() const { return active_low; }
    
    // Line position estimation (returns -100 to +100, 0 = centered)
    int16_t get_line_position() const;
    
    // Simple line detection
    bool line_detected() const;
    
private:
    std::array<uint8_t, 4> sensor_pins;
    std::array<uint16_t, 4> white_calibration;
    std::array<uint16_t, 4> black_calibration;
    std::array<uint16_t, 4> thresholds;
    bool active_low;
    bool calibrated;
    
    uint16_t read_raw(uint8_t index) const;
};

#endif // LINE_SENSOR_H
