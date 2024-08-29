/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_RangeFinder_MaxsonarI2CXL.cpp - Arduino Library for MaxBotix I2C XL sonar
 *       Code by Randy Mackay. DIYDrones.com
 *
 *       datasheet: http://www.maxbotix.com/documents/I2CXL-MaxSonar-EZ_Datasheet.pdf
 *
 *       Sensor should be connected to the I2C port
 */
#include "AP_RangeFinder_SU04_TCA9548A.h"

#if AP_RANGEFINDER_MAXSONARI2CXL_ENABLED

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

AP_RangeFinder_SU04_TCA9548A::AP_RangeFinder_SU04_TCA9548A(RangeFinder::RangeFinder_State &_state,
                                                           AP_RangeFinder_Params &_params,
                                                           AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                                           AP_HAL::OwnPtr<AP_HAL::I2CDevice> swdev,
                                                           uint8_t mux_bus)
    : AP_RangeFinder_Backend(_state, _params)
    , _dev(std::move(dev))
    , _swdev(std::move(swdev))
    , _mux_bus(mux_bus)
{
}

/*
   detect if a Maxbotix rangefinder is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
AP_RangeFinder_Backend *AP_RangeFinder_SU04_TCA9548A::detect(RangeFinder::RangeFinder_State &_state,
													  	     AP_RangeFinder_Params &_params,
                                                             AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                                             AP_HAL::OwnPtr<AP_HAL::I2CDevice> swdev,
                                                             uint8_t mux_bus)
{
    if (!dev || !swdev) {
        return nullptr;
    }

    AP_RangeFinder_SU04_TCA9548A *sensor
        = NEW_NOTHROW AP_RangeFinder_SU04_TCA9548A(_state, _params, std::move(dev), std::move(swdev), mux_bus);
    if (!sensor) {
        return nullptr;
    }

    if (!sensor->_init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

/*
  initialise sensor
 */
bool AP_RangeFinder_SU04_TCA9548A::_init(void)
{
    uint16_t reading_cm;
    if (!get_reading(reading_cm)) {
        return false;
    }

    _dev->register_periodic_callback(100000,
                                     FUNCTOR_BIND_MEMBER(&AP_RangeFinder_SU04_TCA9548A::_timer, void));

    return true;
}

// read - return last value measured by sensor
bool AP_RangeFinder_SU04_TCA9548A::get_reading(uint16_t &reading_cm)
{
    // Take locks both on the switch i2c device (multiplexer) and the main i2c device (sonar)
    // As they are on the same bus, this is actually the same semaphore, but just in case
    _swdev->get_semaphore()->take_blocking();
    _dev->get_semaphore()->take_blocking();

    // First, select the correct sonar
    uint8_t mask = uint8_t(1 << _mux_bus);
    bool sw_sent = _swdev->transfer(&mask, sizeof(mask), nullptr, 0);

    if (!sw_sent) {
        _dev->get_semaphore()->give();
        _swdev->get_semaphore()->give();
        return false;
    }

    // Second, read the measurements
    be16_t val = (be16_t)0xFFFF;
    bool ret = _dev->transfer(nullptr, 0, (uint8_t *) &val, sizeof(val));

    // Finally, unlock both devices
    _dev->get_semaphore()->give();
    _swdev->get_semaphore()->give();

    if (ret) {
        // combine results into distance
        reading_cm = be16toh(val);
    }

    return ret;
}

/*
  timer called at 10Hz
*/
void AP_RangeFinder_SU04_TCA9548A::_timer(void)
{
    uint16_t d;
    if (get_reading(d)) {
        WITH_SEMAPHORE(_sem);
        distance = d;
        new_distance = true;
        state.last_reading_ms = AP_HAL::millis();
    }
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_SU04_TCA9548A::update(void)
{
    WITH_SEMAPHORE(_sem);
    if (new_distance) {
        state.distance_m = distance * 0.01f;
        new_distance = false;
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > 300) {
        // if no updates for 0.3 seconds set no-data
        set_status(RangeFinder::Status::NoData);
    }
}

#endif  // AP_RANGEFINDER_MAXSONARI2CXL_ENABLED
