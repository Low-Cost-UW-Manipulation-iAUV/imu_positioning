

#include "imu_positioning/xsens_MTi30.hpp"
#include "ros/ros.h"
// Messages needed. FIXME remove the bottom two if they aren't needed, which I suspect.
//#include "sensor_msgs/Imu.h"
//#include "geometry_msgs/Vector3.h"

namespace imu_positioning {

xsens::xsens(const ros::NodeHandle &nh ) {
    nh_ = nh;

    sub_imu = nh_.subscribe<sensor_msgs::Imu>("imu/data",1, &xsens::imu_callback, this);
    unsigned int  number_of_samples = 1000;
    calibration_x.reset(number_of_samples);
    calibration_y.reset(number_of_samples);
    calibration_z.reset(number_of_samples);

    calibration_x.set_deadband_width_multiplication(3);
    calibration_y.set_deadband_width_multiplication(3);
    calibration_z.set_deadband_width_multiplication(3);

    integrate_x.reset();
    integrate_y.reset();
    integrate_z.reset();

    integrate_x.set_zero_vel_precision(3);
    integrate_y.set_zero_vel_precision(3);
    integrate_z.set_zero_vel_precision(3);

    x_calibrated = false;
    y_calibrated = false;
    z_calibrated = false;
    counter = 0;
    pubber = nh_.advertise<geometry_msgs::Vector3>("imu/position", 1000);        

}

xsens::~xsens() {}
void xsens::imu_callback(const sensor_msgs::Imu::ConstPtr& message) {
    // create the empty message
    geometry_msgs::Vector3 send_me;

    // if the axis is calibrated just integrate.
    if(x_calibrated == true) {
        integrate_x.put_in(message->linear_acceleration.x, message->header.stamp);
        // fill the message.
        send_me.x = integrate_x.get_out();
    } else { //if not calibrate...
    ROS_INFO("imu_positioning - xsens: calibration sample %u", counter);

        calibration_x.put_in(message->linear_acceleration.x);

        x_calibrated = calibration_x.is_calibration_done();
        if (x_calibrated == true) {
            integrate_x.set_offset( calibration_x.get_offset() );            
            integrate_x.set_deadband( calibration_x.get_deadband() );
            ROS_INFO("imu_positioning - xsens - x: offset: %f, deadband: %f, standard_deviation %f ",calibration_x.get_offset(), calibration_x.get_deadband(), calibration_x.get_std_dev());

        }
    } 

    if(y_calibrated == true) {
        integrate_y.put_in(message->linear_acceleration.y, message->header.stamp);
        // fill the message.
        send_me.y = integrate_y.get_out();
    } else { //if not calibrate...

        calibration_y.put_in(message->linear_acceleration.y);

        y_calibrated = calibration_y.is_calibration_done();
        if (y_calibrated == true) {
            integrate_y.set_offset( calibration_y.get_offset() );            
            integrate_y.set_deadband( calibration_y.get_deadband() );
            ROS_INFO("imu_positioning - xsens - y: offset: %f, deadband: %f, standard_deviation %f ",calibration_y.get_offset(), calibration_y.get_deadband(), calibration_y.get_std_dev());

        }
    }     
    if(z_calibrated == true) {
        integrate_z.put_in(message->linear_acceleration.z, message->header.stamp);
        // fill the message.
        send_me.z = integrate_z.get_out();
    } else { //if not calibrate...

        calibration_z.put_in(message->linear_acceleration.z);

        z_calibrated = calibration_z.is_calibration_done();
        if (z_calibrated == true) {
            integrate_z.set_offset( calibration_z.get_offset() );            
            integrate_z.set_deadband( calibration_z.get_deadband() );
            ROS_INFO("imu_positioning - xsens - z: offset: %f, deadband: %f, standard_deviation %f ",calibration_z.get_offset(), calibration_z.get_deadband(), calibration_z.get_std_dev());

        }
    } 
        counter ++;
        // send the message off.
        pubber.publish(send_me);
}



}  // end of namespace



int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_positioning");
    ros::NodeHandle nh;
    /// An Async spinner creates another thread which will handle the event of this node being executed.

    ros::AsyncSpinner spinner(2);
    spinner.start();

    // create the instance of the class
    imu_positioning::xsens orange_box(nh);

    // register the 
    ros::spin();


    ROS_INFO("imu_positioning: Shutting down ");
}