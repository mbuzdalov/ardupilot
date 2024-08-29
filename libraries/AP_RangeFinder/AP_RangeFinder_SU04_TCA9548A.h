#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_MAXSONARI2CXL_ENABLED // Q&D leave someone else's define

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

#include <AP_HAL/I2CDevice.h>

class AP_RangeFinder_SU04_TCA9548A : public AP_RangeFinder_Backend
{
public:
    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_RangeFinder_Params &_params,
                                          AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                          AP_HAL::OwnPtr<AP_HAL::I2CDevice> swdev,
                                          uint8_t mux_bus);

    // update state
    void update(void) override;

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

private:
    // constructor
    AP_RangeFinder_SU04_TCA9548A(RangeFinder::RangeFinder_State &_state,
    								AP_RangeFinder_Params &_params,
                                    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                    AP_HAL::OwnPtr<AP_HAL::I2CDevice> swdev,
                                    uint8_t mux_bus);

    bool _init(void);
    void _timer(void);

    uint8_t _mux_bus;
    uint16_t distance;
    bool new_distance;

    // start a reading
    bool get_reading(uint16_t &reading_cm);
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _swdev;
};

#endif  // AP_RANGEFINDER_MAXSONARI2CXL_ENABLED
