#ifndef __USEFUL_CODE_TRAPEZOID_INTEGRATION__
#define __USEFUL_CODE_TRAPEZOID_INTEGRATION__

#include "ros/ros.h"
namespace useful_code {

class trapezoid_integration {
    public:
        trapezoid_integration();
        ~trapezoid_integration();
        void integrate(const double, const ros::Duration, double *);
        void reset(void);

    private:
    double accumulator;
    double previous_data;
    bool run_once;
};
} //end of namespace

#endif