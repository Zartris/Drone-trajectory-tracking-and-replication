//
// Created by zartris on 3/11/19.
//
#include <timestamp_sync_fixer/ts_fixer.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "ts_fixer");
    ros::NodeHandle nh;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    test_utils::ts_fixer node(nh);
    ros::spin();
}

namespace test_utils {

    ts_fixer::ts_fixer(ros::NodeHandle n) {
        sub_img = n.subscribe("/bebop/image_mono", 1000, &ts_fixer::callback_img, this);
        sub_imu = n.subscribe("/imu", 1000, &ts_fixer::callback_imu, this);

        pub_img = n.advertise<sensor_msgs::Image>("/tester/image_mono", 1000);
        pub_imu = n.advertise<sensor_msgs::Imu>("/tester/imu", 1000);
    }

    void ts_fixer::callback_img(sensor_msgs::Image img) {
        img_seen = true;
        img_time = img.header.stamp;
        auto time = img_time - imu_time;
//        latest_img_time = img.header.stamp.sec;
//        latest_img_time_ns = img.header.stamp.nsec;
//
////        if (imu_seen && time_difference == 0) {
//        time_difference_ns = latest_img_time_ns - latest_imu_time_ns;
//        time_difference = latest_img_time - latest_imu_time;
//        if (time_difference_ns < 0) {
//            if(time_difference)
//            time_difference -= 1;
//            time_difference_ns = 1000000000 + time_difference_ns;
//        }
        if (imu_seen) {
            std::cout << "Time difference \nsec :" << time.sec << " \nmili sec: " << ((0 <= time.toNSec()/1000000) ? " ":"") << time.toNSec()/1000000 <<" \nnano sec: "
                      << ((0 <= time.toNSec()) ? " ":"") <<time.nsec << "\n \n";
        }
//        }
        pub_img.publish(img);
    }

    void ts_fixer::callback_imu(sensor_msgs::Imu imu) {
        imu_seen = true;
        imu_time = imu.header.stamp;
        pub_imu.publish(imu);
//        latest_imu_time = imu.header.stamp.sec;
//        latest_imu_time_ns = imu.header.stamp.nsec;
//        if (img_seen && time_difference == 0) {
//            time_difference_ns = latest_img_time_ns - latest_imu_time_ns;
//            time_difference = latest_img_time - latest_imu_time;
//            if (time_difference_ns < 0) {
//                time_difference -= 1;
//                time_difference_ns = 1000000000 + time_difference_ns;
//            }
//            std::cout << "Time difference \nsec :" << time_difference << " \nnano sec: "
//                      << latest_img_time_ns << " - " << latest_imu_time_ns << " = " << time_difference_ns << "\n \n";
//
//        } else if (time_difference != 0) {
//            int sec = latest_imu_time + time_difference;
//            int nsec = latest_imu_time_ns + time_difference_ns;
//            if (nsec > 1000000000) {
//                sec += 1;
//                nsec -= 1000000000;
//            }
//            imu.header.stamp.sec = static_cast<uint32_t>(sec);
//
//            imu.header.stamp.nsec = static_cast<uint32_t>(nsec);
//        }
    }
}

