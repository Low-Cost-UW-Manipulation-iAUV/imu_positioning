#ifndef __USEFUL_CODE_SENSOR_INTEGRATION__
#define __USEFUL_CODE_SENSOR_INTEGRATION__

#include "ros/ros.h"
#include "imu_positioning/trapezoid_integration.hpp"

namespace useful_code {
class integration {
    public:
        integration();
        ~integration();

        void reset(void);

        void put_in(const double, const ros::Time);
        double get_out(void);

        void set_offset(double);
        void set_deadband(double);
        void set_zero_vel_precision(unsigned int);

    private:
        double apply_deadband(const double);
        double zero_velocity_detection(const double, const double);
        double do_offset(const double);
        double output;
        ros::Time last_time;

        double offset;
        double deadband;
        unsigned int zero_velocity_precision;
        unsigned int zero_input_counter;
        trapezoid_integration acc_to_vel;
        trapezoid_integration vel_to_pos;
};

} //End of namespace

#endif

