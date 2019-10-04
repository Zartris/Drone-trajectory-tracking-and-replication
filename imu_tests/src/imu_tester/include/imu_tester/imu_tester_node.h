//
// Created by zartris on 3/13/19.
//

#ifndef IMU_TESTER_IMU_TESTER_NODE_H
#define IMU_TESTER_IMU_TESTER_NODE_H

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <boost/circular_buffer.hpp>


namespace imu_tester {
    class ImuTesterNode {
    public: // methods
        explicit ImuTesterNode(ros::NodeHandle handle);

        virtual ~ImuTesterNode() = default;

//        void callback_img(sensor_msgs::Image img);

        void callback_imu(sensor_msgs::Imu imu);

        void checkOutlier(boost::circular_buffer<double> cb, double value, std::string name, int limit) const;

    public: // variables
        int32_t time_difference = 0;
        int32_t time_difference_ns = 0;
        int32_t latest_img_time_ns = 0;

        int32_t latest_img_time = 0;
        int32_t latest_imu_time_ns = 0;

        int32_t latest_imu_time = 0;
        boost::circular_buffer<double> imu_la_x_hist = boost::circular_buffer<double>(9);
        boost::circular_buffer<double> imu_la_y_hist = boost::circular_buffer<double>(9);

        boost::circular_buffer<double> imu_la_z_hist = boost::circular_buffer<double>(9);
        boost::circular_buffer<double> imu_av_x_hist = boost::circular_buffer<double>(9);
        boost::circular_buffer<double> imu_av_y_hist = boost::circular_buffer<double>(9);

        boost::circular_buffer<double> imu_av_z_hist = boost::circular_buffer<double>(9);
        boost::circular_buffer<double> imu_o_x_hist = boost::circular_buffer<double>(9);
        boost::circular_buffer<double> imu_o_y_hist = boost::circular_buffer<double>(9);
        boost::circular_buffer<double> imu_o_z_hist = boost::circular_buffer<double>(9);

        boost::circular_buffer<double> imu_o_w_hist = boost::circular_buffer<double>(9);
        bool img_seen = false;

        bool imu_seen = false;
        ros::Subscriber sub_img;
        ros::Subscriber sub_imu;
        ros::Publisher pub_img;


        ros::Publisher pub_imu;
    };


}
#endif