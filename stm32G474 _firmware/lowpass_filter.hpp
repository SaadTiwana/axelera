#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

// From: https://github.com/simplefoc/Arduino-FOC/blob/master/src/common/lowpass_filter.h

#include "time_utils.hpp"
#include "foc_utils.hpp"

/**
 *  Low pass filter class
 */
class LowPassFilter
{
public:
    /**
     * @param Tf - Low pass filter time constant
     */
    LowPassFilter(float Tf);
    ~LowPassFilter() = default;

    float operator() (float x);
    float Tf; //!< Low pass filter time constant

protected:
    unsigned long timestamp_prev;  //!< Last execution timestamp
    float y_prev; //!< filtered value in previous execution step 
};

#endif // LOWPASS_FILTER_H