#ifndef __TLE5012B_H_
#define __TLE5012B_H_

#include "Arduino.h"
#include "SimpleFOC.h"

#define ENCODER_SS   PB0
#define ENCODER_SCK  PIN_SPI_SCK
#define ENCODER_MISO PIN_SPI_MISO
#define ENCODER_MOSI PIN_SPI_MOSI


// constants
#define CHANGE_UINT_TO_INT_15       0x8000    // Used to change unsigned 16 bit integer into signed
#define CHECK_BIT_14                0x4000    // Used to check the 14th bit
#define DELETE_BIT_15               0x7FFF    // Used to delete everything except the first 15 bits
#define ENCODER_ANGLE_REG      (0x0020U)
#define ENCODER_READ_COMMAND    0x8000 // 8000
#define ENCODER_SPEED_REG      (0x0030U)
#define GET_BIT_14_4                0x7FF0    // Used to check the 14th bit?
#define POW_2_7                     128.0     // 2^7
#define POW_2_15                    32768.0   // 2^15
#define RADS_IN_CIRCLE              6.28319

#define ENCODER_RESOLUTION           14    // 14bit
#define ENCODER_FULL_RANGE           16384 // 2^(ENCODER_RESOLUTION)
#define ENCODER_80_RANGE             13107 // 0.8*(ENCODER_FULL_RANGE)

class TLE5012B: public Sensor{
    public:
        TLE5012B();
        void init();

        int getRawCount();
        
        // Abstract functions of the Sensor class implementation
        /** get current angle (rad) */
        float getAngle() override;
        /**  get current angular velocity (rad/s) */
        float getVelocity() override; 

    public:
       // total angle tracking variables
        float full_rotation_offset; //!<number of full rotations made
        float angle_data_prev; //!< angle in previous position calculation step

        // velocity calculation variables
        float angle_prev; //!< angle in previous velocity calculation step
        long velocity_calc_timestamp; //!< last velocity calculation timestamp
        
        int bit_resolution; //!< the number of bites of angle data

};

#endif // __TLE5012B_H_