//
// Created by zartris on 3/11/19.
//

#ifndef TIMESTAMP_SYNC_FIXER_TS_FIXER_H
#define TIMESTAMP_SYNC_FIXER_TS_FIXER_H

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <string>
#include <iostream>


namespace test_utils {
    class ts_fixer {
    public: // methods
        ts_fixer(ros::NodeHandle handle);

        virtual ~ts_fixer() = default;

        void callback_img(sensor_msgs::Image img);

        void callback_imu(sensor_msgs::Imu imu);

    public: // variables
        ros::Time imu_time;
        ros::Time img_time;

        long time_difference = 0;
        long time_difference_ns = 0;
        long latest_img_time_ns = 0;
        long latest_img_time = 0;

        long latest_imu_time_ns = 0;
        long latest_imu_time = 0;



        bool img_seen = false;
        bool imu_seen = false;

        ros::Subscriber sub_img;
        ros::Subscriber sub_imu;
        ros::Publisher pub_img;
        ros::Publisher pub_imu;
    };
}
#endif //TIMESTAMP_SYNC_FIXER_TS_FIXER_H
