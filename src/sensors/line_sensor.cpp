#include "line_sensor.h"
#include <stdio.h>

LineSensor::LineSensor(const std::array<uint, 4>& pins, bool active_low)
    : sensor_pins(pins), active_low(active_low), calibrated(false) {
    
    // Initialize calibration arrays
    white_calibration.fill(0);
    black_calibration.fill(4095); // Max for 12-bit ADC
    thresholds.fill(2048); // Midpoint default
}

void LineSensor::init() {
    printf("Initializing line sensor on pins: %d, %d, %d, %d\n",
           sensor_pins[0], sensor_pins[1], sensor_pins[2], sensor_pins[3]);
    
    for (uint i = 0; i < 4; i++) {
        gpio_init(sensor_pins[i]);
        gpio_set_dir(sensor_pins[i], GPIO_IN);
        
        // Enable pull-up resistor for active-low sensors
        if (active_low) {
            gpio_pull_up(sensor_pins[i]);
        }
        
        // Optional: Add a small delay between initializations
        sleep_ms(1);
    }
    
    printf("Line sensor initialized (active %s)\n", 
           active_low ? "low" : "high");
}

std::array<bool, 4> LineSensor::read_all() const {
    std::array<bool, 4> values;
    
    for (uint i = 0; i < 4; i++) {
        values[i] = read_sensor(i);
    }
    
    return values;
}

bool LineSensor::read_sensor(uint index) const {
    if (index >= 4) {
        return false; // Error case
    }
    
    bool raw_value = gpio_get(sensor_pins[index]);
    
    // Invert if active-low
    return active_low ? !raw_value : raw_value;
}

uint8_t LineSensor::get_pattern() const {
    uint8_t pattern = 0;
    auto values = read_all();
    
    for (uint8_t i = 0; i < 4; i++) {
        if (values[i]) {
            pattern |= (1 << i);
        }
    }
    
    return pattern;
}

uint16_t LineSensor::read_raw(uint index) const {
    // For digital sensors, just return 0 or 1
    return gpio_get(sensor_pins[index]) ? 1 : 0;
}

void LineSensor::calibrate_white() {
    printf("Calibrating on WHITE surface...\n");
    
    for (uint8_t i = 0; i < 4; i++) {
        // Take multiple readings and average
        uint32_t sum = 0;
        const uint8_t samples = 10;
        
        for (uint8_t s = 0; s < samples; s++) {
            sum += read_raw(i);
            sleep_ms(10);
        }
        
        white_calibration[i] = sum / samples;
        printf("  Sensor %d: %d\n", i, white_calibration[i]);
    }
    
    calibrated = true;
}

void LineSensor::calibrate_black() {
    printf("Calibrating on BLACK line...\n");
    
    for (uint8_t i = 0; i < 4; i++) {
        // Take multiple readings and average
        uint32_t sum = 0;
        const uint8_t samples = 10;
        
        for (uint8_t s = 0; s < samples; s++) {
            sum += read_raw(i);
            sleep_ms(10);
        }
        
        black_calibration[i] = sum / samples;
        thresholds[i] = (white_calibration[i] + black_calibration[i]) / 2;
        printf("  Sensor %d: black=%d, threshold=%d\n", 
               i, black_calibration[i], thresholds[i]);
    }
}

uint16_t LineSensor::get_calibrated_threshold(uint index) const {
    if (index >= 4) return 0;
    return thresholds[index];
}

int16_t LineSensor::get_line_position() const {
    auto values = read_all();
    
    // Weighted position calculation
    // Sensor positions: -30, -10, +10, +30 (adjust as needed)
    const int16_t weights[4] = {-30, -10, 10, 30};
    
    int16_t weighted_sum = 0;
    uint8_t active_count = 0;
    
    for (uint8_t i = 0; i < 4; i++) {
        if (values[i]) {
            weighted_sum += weights[i];
            active_count++;
        }
    }
    
    if (active_count == 0) {
        return 0; // No line detected
    }
    
    return weighted_sum / active_count;
}

bool LineSensor::line_detected() const {
    auto values = read_all();
    
    for (uint i = 0; i < 4; i++) {
        if (values[i]) {
            return true;
        }
    }
    
    return false;
}
