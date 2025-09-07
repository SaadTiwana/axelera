#ifndef SYSTEM_STATUS_HPP
#define SYSTEM_STATUS_HPP

#include <Arduino.h>
#include "time_utils.hpp"


/**
 * Structure to hold a value with its timestamp
 */
struct TimestampedFloat {

    // Make sure the return type of function to get micro seconds is uint32_t
    // otherwise overflows will become an issue
    static_assert(std::is_same<decltype(_micros()), uint32_t>::value,
                  "_micros() must return uint32_t");
                  

    float value;           // The actual value (angle in degrees)
    uint32_t timestamp_us; // Timestamp in microseconds when value was last updated

    TimestampedFloat() : value(0.0f), timestamp_us(0) {}
    
    TimestampedFloat(float val) : value(val), timestamp_us(_micros()) {}

    // Overload assignment "=" operator for float
    TimestampedFloat& operator=(float val) {
        value = val;
        timestamp_us = _micros(); // Update timestamp
        return *this;
    }

    uint32_t getAge_us()
    {
        return _micros() - timestamp_us;
    }

    /*
    Checks if age is more than supplied value in microseconds based on stored timestamp
    returns true if stale, false if not stale
    */
    bool isStale(uint32_t max_age_us) {
        return getAge_us() > max_age_us;
    }
};


/**
 * SystemStatus class to manage system state values with timestamps
 * Provides functionality to update values and check their age
 */
class SystemStatus {
private:



public:

    // latest motor yaw, pitch angles, with timestamps of last update
    TimestampedFloat mot_angle_pitch_deg;
    TimestampedFloat mot_angle_yaw_deg;

    // current target angles for motor pitch and yaw
    float mot_angle_pitch_deg_target = 0.0f; // Target angle for pitch motor
    float mot_angle_yaw_deg_target = 0.0f;   // Target angle for yaw motor

    // Master software switch to stabilize or not
    bool stabilize = false;

    // Constructor
    SystemStatus() {
        // Initialize with initial values
        mot_angle_pitch_deg = TimestampedFloat(0.0f);
        mot_angle_yaw_deg = TimestampedFloat(0.0f);
    }

};


#endif