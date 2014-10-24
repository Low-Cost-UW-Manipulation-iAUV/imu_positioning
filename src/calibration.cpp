#include "imu_positioning/calibration.hpp"

#include <cmath>
namespace useful_code {

calibration::calibration(unsigned int num_o) {
    num_of_samples = num_o;
    last_time = ros::Time::now();
    stored_data.clear();
    stored_data.reserve(num_of_samples);

    current_position = 0;

    deadband_multiplier = 1;
    offset = 0;
    standard_deviation = 0;
    deadband = 0;
}

calibration::calibration(void) {
    num_of_samples = 1000; // FIXME
    last_time = ros::Time::now();
    stored_data.clear();
    current_position = 0;

    deadband_multiplier = 1;
    offset = 0;
    standard_deviation = 0;
    deadband = 0;
}

void calibration::reset(void) {
    last_time = ros::Time::now();
    stored_data.clear();

    current_position = 0;

    deadband_multiplier = 1;
    offset = 0;
    standard_deviation = 0;
    deadband = 0;
}

void calibration::reset(unsigned int num_o) {
    num_of_samples = num_o;    
    last_time = ros::Time::now();
    stored_data.clear();
    stored_data.reserve(num_of_samples);

    current_position = 0;

    deadband_multiplier = 1;
    offset = 0;
    standard_deviation = 0;
    deadband = 0;
}

calibration::~calibration() {}

bool calibration::is_calibration_done(void) {
    // are we done yet?
    if ( (current_position + 1) >= num_of_samples) {
     return true;   
    } 
    return false;
}

int calibration::put_in( double new_data) {
    stored_data.push_back(new_data);
    current_position ++;

    offset = mean(stored_data);
    standard_deviation = std2(stored_data, offset);   
    deadband = standard_deviation * deadband_multiplier;
}

double calibration::get_offset(void) {
    return offset;
}

double calibration::get_std_dev(void) {
    return standard_deviation;
}

double calibration::get_deadband(void) {
    return deadband;
}

void calibration::set_deadband_width_multiplication(double mply) {
    deadband_multiplier = mply;
}

/** mean(): calculates the mean
*/
double calibration::mean(const std::vector<double> vec) {
    double sum = 0;
    unsigned int size = 0;
    for (typename std::vector<double>::const_iterator it = vec.begin(); it!=vec.end(); ++it, ++size) { 
        sum += (*it);
        // if size is not == 0, divide by size, else return 0
    }
    return (size)?(sum/size):0;
}

/** std2(): find the std_deviation across our current_range
*/
double calibration::std2(const std::vector<double> vec, const double mean) {
    double sum = 0;
    unsigned int size = 0;
    for(typename std::vector<double>::const_iterator it = vec.begin(); it != vec.end(); ++it) {
        sum+=std::pow(((*it) - mean),2);
         ++size;
    }
    // if size is not == 0, divide by size, else return 0            
    return size?std::sqrt(sum/size):-1;
    
} 
}  //end of namespace

