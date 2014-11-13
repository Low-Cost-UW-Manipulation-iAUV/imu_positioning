

#include "imu_positioning/xsens_MTi30.hpp"
#include "ros/ros.h"
// Messages needed. FIXME remove the bottom two if they aren't needed, which I suspect.
//#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"

namespace imu_positioning {

xsens::xsens(const ros::NodeHandle &nh ) {
    nh_ = nh;
    get_parameters();



    calibration_x.reset(number_of_samples[0]);
    calibration_y.reset(number_of_samples[1]);
    calibration_z.reset(number_of_samples[2]);

    calibration_x.set_deadband_width_multiplication(deadband_width_multi[0]);
    calibration_y.set_deadband_width_multiplication(deadband_width_multi[1]);
    calibration_z.set_deadband_width_multiplication(deadband_width_multi[2]);

    integrate_x.reset(base_frequency);
    integrate_y.reset(base_frequency);
    integrate_z.reset(base_frequency);

    integrate_x.set_zero_vel_precision(detect_zero_precision[0]);
    integrate_y.set_zero_vel_precision(detect_zero_precision[1]);
    integrate_z.set_zero_vel_precision(detect_zero_precision[2]);

    x_calibrated = false;
    y_calibrated = false;
    z_calibrated = false;
    counter = 0;

    pubber = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("xsens/velocity", 1000);
    

    reset_integration_service = nh_.advertiseService("reset_integration", &xsens::reset_integration, this);
    sub_imu = nh_.subscribe<sensor_msgs::Imu>("imu/data",1, &xsens::imu_callback, this);

}

xsens::~xsens() {}
void xsens::imu_callback(const sensor_msgs::Imu::ConstPtr& message) {
    // create the empty message
    geometry_msgs::TwistWithCovarianceStamped send_me;

    // if the axis is calibrated just integrate.
    if(x_calibrated == true) {
        integrate_x.put_in(message->linear_acceleration.x, message->header.seq);
        // fill the message.
        send_me.twist.twist.linear.x = integrate_x.get_out_velocity_zero_detect();
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
        integrate_y.put_in(message->linear_acceleration.y, message->header.seq);
        // fill the message.
        send_me.twist.twist.linear.y = integrate_y.get_out_velocity_zero_detect();
       
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
        integrate_z.put_in(message->linear_acceleration.z, message->header.seq);
        // fill the message.
        send_me.twist.twist.linear.z = integrate_z.get_out_velocity_zero_detect();
       
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
        // send the message off if any of them is calibrated...
        if (z_calibrated || y_calibrated || x_calibrated) {
            send_me.header.seq = message->header.seq;
            send_me.header.stamp = ros::Time::now();
            send_me.header.frame_id = "odom";
            send_me.twist.covariance.fill(0.0);
            send_me.twist.covariance[1] = calibration_x.get_variance();
            send_me.twist.covariance[7] = calibration_y.get_variance();
            send_me.twist.covariance[14] = calibration_z.get_variance();

            pubber.publish(send_me);
        }
}
bool xsens::reset_integration(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    integrate_x.reset(base_frequency);
    integrate_y.reset(base_frequency);
    integrate_z.reset(base_frequency);

    integrate_x.set_zero_vel_precision(detect_zero_precision[0]);
    integrate_y.set_zero_vel_precision(detect_zero_precision[1]);
    integrate_z.set_zero_vel_precision(detect_zero_precision[2]);

    integrate_x.set_offset( calibration_x.get_offset() );     
    integrate_y.set_offset( calibration_y.get_offset() );            
    integrate_z.set_offset( calibration_z.get_offset() );            

    integrate_x.set_deadband( calibration_x.get_deadband() ); 
    integrate_y.set_deadband( calibration_y.get_deadband() ); 
    integrate_z.set_deadband( calibration_z.get_deadband() ); 
    ROS_INFO("imu_positioning - xsens: reset integration to base frequency %f, detect_zero_precision to [x,y,z]: %d, %d, %d and rewrote the offset and deadband from stored calibration",base_frequency,detect_zero_precision[0], detect_zero_precision[1], detect_zero_precision[2]);
    return 1;
}


void xsens::get_parameters(void) {


    if (!nh_.getParam("/imu_positioning/base_frequency", base_frequency)) {
        ROS_ERROR("imu_positioning - xsens: couldnt find base_frequency, assuming 400");
        base_frequency = 400;

      nh_.setParam("/imu_positioning/base_frequency", base_frequency);
    } else {
      ROS_INFO("imu_positioning - xsens: base_frequency is %f Hz", base_frequency);
    }

    number_of_samples.clear();
    number_of_samples.resize(3,0);
    if (!nh_.getParam("/imu_positioning/number_of_samples", number_of_samples)) {
        ROS_ERROR("imu_positioning - xsens: couldnt find number_of_samples, assuming 12000");
        number_of_samples[0] = 12000;
        number_of_samples[1] = 12000;
        number_of_samples[2] = 12000;

      nh_.setParam("/imu_positioning/number_of_samples", number_of_samples);
    } else {
      ROS_INFO("imu_positioning - xsens: number_of_samples is [x,y,z]: %d, %d, %d", number_of_samples[0],number_of_samples[1],number_of_samples[2]);
    }    

    deadband_width_multi.clear();
    deadband_width_multi.resize(3,0);
    if (!nh_.getParam("/imu_positioning/deadband_width_multi", deadband_width_multi)) {
        ROS_ERROR("imu_positioning - xsens: couldnt find deadband_width_multi, assuming 3");
        deadband_width_multi[0] = 3;
        deadband_width_multi[1] = 3;
        deadband_width_multi[2] = 3;


      nh_.setParam("/imu_positioning/deadband_width_multi", deadband_width_multi);
    } else {
      ROS_INFO("imu_positioning - xsens: deadband_width_multi is [x,y,z]: %f,%f,%f", deadband_width_multi[0],deadband_width_multi[1],deadband_width_multi[2]);
    }  

    detect_zero_precision.clear();
    detect_zero_precision.resize(3,0);
    if (!nh_.getParam("/imu_positioning/detect_zero_precision", detect_zero_precision)) {
        ROS_ERROR("imu_positioning - xsens: couldnt find detect_zero_precision, assuming 20");
        detect_zero_precision[0] = 20;
        detect_zero_precision[1] = 20;
        detect_zero_precision[2] = 20;


      nh_.setParam("/imu_positioning/detect_zero_precision", detect_zero_precision);
    } else {
      ROS_INFO("imu_positioning - xsens: detect_zero_precision is for DOF x: %u, y: %u, z: %u",detect_zero_precision[0],detect_zero_precision[1],detect_zero_precision[2]);
    }
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