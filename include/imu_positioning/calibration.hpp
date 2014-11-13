#ifndef __USEFUL_CODE_CALIBRATION__
#define __USEFUL_CODE_CALIBRATION__

#include <vector>
#include "ros/ros.h"

namespace useful_code {
class calibration {
	public:
        // start with number of samples data to calibrate for.
        calibration(int);
        calibration(void);

        // start with time to calibrate for
        ~calibration();
        
        void reset(void);
        void reset(int);

        bool is_calibration_done(void);

        int put_in(double);
//        int set_calibration_length(unsigned int);
//        int set_calibration_length(timestamp time);
        void set_deadband_width_multiplication(double);

        double get_deadband(void);
        double get_offset(void);
        double get_std_dev(void);
        double get_variance(void);



    private:

        double mean(const std::vector<double>);
        double std2(const std::vector<double>, const double mean);
        ros::Time last_time;
        std::vector<double> stored_data;
        unsigned int current_position;

        double deadband_multiplier;

        double offset;
        double standard_deviation;
        double variance;
        double deadband;
        int num_of_samples;
};


}  //End of namespace
#endif

