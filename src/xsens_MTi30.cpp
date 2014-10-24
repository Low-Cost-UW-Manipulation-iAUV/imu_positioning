

#include "imu_positioning/xsens_MTi30.hpp"
#include "ros/ros.h"
// Messages needed. FIXME remove the bottom two if they aren't needed, which I suspect.
//#include "sensor_msgs/Imu.h"
//#include "geometry_msgs/Vector3.h"

namespace imu_positioning {

xsens::xsens(const ros::NodeHandle &nh ) {
    nh_ = nh;

    sub_imu = nh_.subscribe<imu_positioning::xsens_MTi>("imu/data",1, &xsens::imu_callback, this);
unsigned int  number_of_samples = 1000;
    calibration_x.reset(number_of_samples);
    calibration_y.reset(number_of_samples);
    calibration_z.reset(number_of_samples);

    integrate_x.reset();
    integrate_y.reset();
    integrate_z.reset();

    integrate_x.set_zero_vel_precision(5);
    integrate_y.set_zero_vel_precision(5);
    integrate_z.set_zero_vel_precision(5);

    x_calibrated = false;
    y_calibrated = false;
    z_calibrated = false;

    pubber = nh_.advertise<geometry_msgs::Vector3>("imu/position", 1000);        

}

xsens::~xsens() {}
void xsens::imu_callback(const imu_positioning::xsens_MTi::ConstPtr& message) {
    // create the empty message
    geometry_msgs::Vector3 send_me;

    // if the axis is calibrated just integrate.
    if(x_calibrated == true) {
        integrate_x.put_in(message->basic.linear_acceleration.x, message->basic.header.stamp);

        // fill the message.
        send_me.x = integrate_x.get_out();
    } else { //if not calibrate...

        calibration_x.put_in(message->basic.linear_acceleration.x);
        x_calibrated = calibration_x.is_calibration_done();
        if (x_calibrated == true) {
            integrate_x.set_offset( calibration_x.get_offset() );            
            integrate_x.set_deadband( calibration_x.get_deadband() );
        }
    } 
       // integrate_y.put_in(message->basic.y, message->basic.header.stamp);
       // integrate_z.put_in(message->basic.z, message->basic.header.stamp);

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