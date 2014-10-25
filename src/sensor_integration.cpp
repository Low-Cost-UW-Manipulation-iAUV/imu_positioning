#include "imu_positioning/sensor_integration.hpp"
#include "ros/ros.h"

namespace useful_code {

integration::integration() {
    frequency = 400;
    output = 0;
    last_one = 0;

    deadband = 0;
    zero_velocity_precision = 0;
    zero_input_counter = 0;
    
    offset = 0;
    acc_to_vel.reset();
    vel_to_pos.reset();
}

integration::integration(const double freq) {
    frequency = freq;
    output = 0;

    deadband = 0;
    zero_velocity_precision = 0;
    zero_input_counter = 0;
    
    offset = 0;
    acc_to_vel.reset();
    vel_to_pos.reset();
}
integration::~integration() {}

void integration::reset(void) {
    frequency = 400;
    output = 0;

    deadband = 0;
    zero_velocity_precision = 0;
    zero_input_counter = 0;
    
    offset = 0;
    acc_to_vel.reset();
    vel_to_pos.reset();
}

void integration::reset(const double freq) {
    output = 0;
    frequency = freq;

    deadband = 0;
    zero_velocity_precision = 0;
    zero_input_counter = 0;
    
    offset = 0;
    acc_to_vel.reset();
    vel_to_pos.reset();
}

void integration::put_in(const double new_data, const int sequence) {
    if(sequence > last_one) {
        ros::Duration time_passed = ros::Duration((sequence - last_one)/frequency);
        last_one = sequence;
 //ROS_INFO("sensor_integration: cycles skipped: %u", (sequence - last_one));

    double data_dt = 0;

    double data_offset = do_offset(new_data);
    double data_dbed = apply_deadband(data_offset);
    acc_to_vel.integrate(data_dbed, time_passed, &data_dt);
    double data_zeroed = zero_velocity_detection(data_dt, data_dbed);
    vel_to_pos.integrate(data_zeroed, time_passed, &output);        

} 

   

}

double integration::get_out(void) {
    return output;
}

void integration::set_deadband(const double d_band) {
    deadband = d_band;
}

void integration::set_zero_vel_precision(const unsigned int z_prec) {
    zero_velocity_precision = z_prec;
}

void integration::set_offset(double offs) {
    offset = offs;
}

double integration::do_offset(const double data) {

    return (data - offset);
}
/** apply_deadband(): sets data to 0 if it is within the deadband and deadband is set
*/
double integration::apply_deadband(const double data) {

    // if we are within the deadband range and the deadband is set, return 0.
    if ( (data <= deadband) && (data >= (deadband * -1)) && (deadband != 0) ) {
        return 0;
    } else {
        return data;
    }
}

/** zero_velocity_detection(): tries to detect a stopped motion and then pulls the velocity to 0
                    It does this by watching the acceleration and if its 0 for a number of
                    cycles, then it sets the velocity to 0
*/
double integration::zero_velocity_detection(const double data_dt, const double data) {

    // keep track of how many times in sequence the input to the class was 0
    if (data == 0) {

        zero_input_counter ++;
    } else {
        zero_input_counter = 0;
    }

    // if we are above our threshold, set the to 0
    if (zero_input_counter >= zero_velocity_precision) {

        return 0;
    } else {
        return data_dt;
    }
}


} // End of namespace