#include "imu_positioning/calibration.hpp"

#include <cmath>
namespace useful_code {

calibration::calibration(unsigned int num_o) {
    num_of_samples = num_o;
    last_time = ros::Time::now();
    data.clear();
    data.resize(num_of_samples,0);
    current_position = 0;

    deadband_multiplier = 1;
    offset = 0;
    standard_deviation = 0;
    deadband = 0;
}

calibration::calibration(void) {
    num_of_samples = 1000; // FIXME
    last_time = ros::Time::now();
    data.clear();
    data.resize(num_of_samples,0);
    current_position = 0;

    deadband_multiplier = 1;
    offset = 0;
    standard_deviation = 0;
    deadband = 0;
}

void calibration::reset(void) {
    last_time = ros::Time::now();
    data.clear();
    data.resize(num_of_samples,0);
    current_position = 0;

    deadband_multiplier = 1;
    offset = 0;
    standard_deviation = 0;
    deadband = 0;
}

void calibration::reset(unsigned int num_o) {
    num_of_samples = num_o;    
    last_time = ros::Time::now();
    data.clear();
    data.resize(num_of_samples,0);
    current_position = 0;

    deadband_multiplier = 1;
    offset = 0;
    standard_deviation = 0;
    deadband = 0;
}

calibration::~calibration() {}

bool calibration::is_calibration_done(void) {
    // are we done yet?
    if ( (current_position + 1) >= data.size()) {
     return EXIT_SUCCESS;   
    } 
    return EXIT_FAILURE;
}

int calibration::put_in(const double new_data) {
   if (current_position < num_of_samples) {     //we are within the allocated calibration time
        data[current_position] = new_data;
        current_position ++;
                /*
        Placeholder for handling time here..
                */
        offset = mean(data);
        standard_deviation = std2(data, offset);   
        deadband = standard_deviation * deadband_multiplier;

        return EXIT_SUCCESS;        
    } else {                                    //we are outside the planned calibration time.
        data.push_back(new_data);
        current_position ++;
                /*
        Placeholder for handling time here..
                */
        offset = mean(data);
        standard_deviation = std2(data, offset); 
        deadband = standard_deviation * deadband_multiplier;

        return EXIT_FAILURE;                
    }

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
double calibration::mean(const std::vector<double>& vec) {
    double sum(0);
    unsigned int size(0);
    for (typename std::vector<double>::const_iterator it = vec.begin(); it!=vec.end(); ++it,++size) { 
        sum += (*it);
        // if size is not == 0, divide by size, else return 0
        return (size)?(sum/size):0;
    }
}

/** std2(): find the std_deviation across our current_range
*/
double calibration::std2(const std::vector<double>& vec, const double mean) {
    double sum(0);
    unsigned int size(0);
    for(typename std::vector<double>::const_iterator it = vec.begin(); it != vec.end(); ++it, ++size) {
        sum+=std::pow(((*it) - mean),2);
        // if size is not == 0, divide by size, else return 0            
        return size?std::sqrt(sum/size):-1;
    }
} 
}  //end of namespace

