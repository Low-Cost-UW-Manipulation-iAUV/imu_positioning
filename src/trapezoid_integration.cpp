#include "imu_positioning/trapezoid_integration.hpp"

namespace useful_code {
trapezoid_integration::trapezoid_integration() {
    accumulator = 0;
    previous_data = 0;
    run_once = false;
}

trapezoid_integration::~trapezoid_integration() {}
void trapezoid_integration::integrate(const double new_data, const ros::Duration dt, double *output) {
    if (run_once == false) {
        previous_data = new_data;    // ensure that the integration starts at least in the range of new_data
        run_once = true;
    }
    accumulator += (  ( (new_data + previous_data) / 2) * dt.toSec());
    previous_data = new_data;    // ensure that the integration starts at least in the range of new_data
    *output = accumulator;
}

void trapezoid_integration::reset(void) {
    accumulator = 0;
    previous_data = 0;
    run_once = false;
}


} //end of namespace