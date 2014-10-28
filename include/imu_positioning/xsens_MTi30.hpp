/* Author: Raphael Nagel
    Desc: ros_control_interface for testing purpose of the ros_control_iso controller
    Date: 02/Sept/2014
*/

#ifndef __xsens_MTi30_positioning_
#define __xsens_MTi30_positioning_

#include <stdio.h>

#include "ros/ros.h"

#include "imu_positioning/xsens_MTi.h"
#include "imu_positioning/sensor_integration.hpp"
#include "imu_positioning/calibration.hpp"
#include "sensor_msgs/Imu.h"
#include "std_srvs/Empty.h"

namespace imu_positioning {
    class xsens {
    public:
        xsens(const ros::NodeHandle&);
        ~xsens();

        void imu_callback(const sensor_msgs::Imu::ConstPtr&);
        bool reset_integration(std_srvs::Empty::Request &, std_srvs::Empty::Response &);

    private:
        /// DOF feedback subscribers
        ros::Subscriber sub_imu;
        ros::Publisher pubber;
        ros::Publisher pubber_dbed;
        ros::Publisher pubber_vel;
        ros::Publisher pubber_vel_zeroed;

        ros::ServiceServer reset_integration_service;
        void get_parameters(void);


        ros::NodeHandle nh_;

        useful_code::calibration calibration_x;
        useful_code::calibration calibration_y;
        useful_code::calibration calibration_z;

        useful_code::integration integrate_x;
        useful_code::integration integrate_y;
        useful_code::integration integrate_z;

        bool x_calibrated;
        bool y_calibrated;
        bool z_calibrated;

        unsigned int counter;
// to be taken from the parameter server
        double base_frequency;
        std::vector<int> detect_zero_precision;
        std::vector<int>  number_of_samples;
        std::vector<double> deadband_width_multi;


    };
}

#endif
