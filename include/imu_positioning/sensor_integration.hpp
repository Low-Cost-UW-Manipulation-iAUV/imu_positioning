#ifndef __USEFUL_CODE_SENSOR_INTEGRATION__
#define __USEFUL_CODE_SENSOR_INTEGRATION__

#include "ros/ros.h"
#include "imu_positioning/trapezoid_integration.hpp"

namespace useful_code {
class integration {
    public:
        integration();
        integration(const double);

        ~integration();

        void reset(void);
        void reset(const double);

        void put_in(const double, const int);
        double get_out(void);
        double get_out_deadbanded(void);
        double get_out_velocity(void);
        double get_out_velocity_zero_detect(void);


        void set_offset(double);
        void set_deadband(double);
        void set_zero_vel_precision(unsigned int);

    private:
        double apply_deadband(const double);
        double zero_velocity_detection(const double, const double);
        double do_offset(const double);
        double output;
        double data_dt;
        double data_offset;
        double data_dbed;
        double data_zeroed;
        int last_one;
        double offset;
        double deadband;
        unsigned int zero_velocity_precision;
        unsigned int zero_input_counter;
        trapezoid_integration acc_to_vel;
        trapezoid_integration vel_to_pos;
        double frequency;
};

} //End of namespace

#endif

