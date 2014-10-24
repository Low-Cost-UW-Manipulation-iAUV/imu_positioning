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
namespace imu_positioning {
    class xsens {
    public:
        xsens(const ros::NodeHandle&);
        ~xsens();

        void imu_callback(const sensor_msgs::Imu::ConstPtr&);

    private:
        /// DOF feedback subscribers
        ros::Subscriber sub_imu;
        ros::Publisher pubber;

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

        double deadband_mult;
        unsigned int zero_vel_precision;
        unsigned int counter;


    };
}

#endif
