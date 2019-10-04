//
// Created by zartris on 3/13/19.
//

#include <imu_tester/imu_tester_node.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "ts_fixer");
    ros::NodeHandle nh;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ROS_INFO("RUNNING");

    imu_tester::ImuTesterNode node(nh);
    ros::spin();
}

namespace imu_tester {

    ImuTesterNode::ImuTesterNode(ros::NodeHandle n) {
//        sub_img = n.subscribe("/bebop/image_mono", 1000, &ImuTesterNode::callback_img, this);
        sub_imu = n.subscribe("/imu", 1000, &ImuTesterNode::callback_imu, this);

//        pub_img = n.advertise<sensor_msgs::Image>("/tester/image_mono", 1000);
//        pub_imu = n.advertise<sensor_msgs::Imu>("/tester/imu", 1000);
    }

//    void ImuTesterNode::callback_img(sensor_msgs::Image img) {
//        img_seen = true;
//        latest_img_time = img.header.stamp.sec;
//        latest_img_time_ns = img.header.stamp.nsec;
////        if (imu_seen && time_difference == 0) {
//        time_difference_ns = latest_img_time_ns - latest_imu_time_ns;
//        time_difference = latest_img_time - latest_imu_time;
//        if (time_difference_ns < 0) {
//            time_difference -= 1;
//            time_difference_ns = 1000000000 + time_difference_ns;
//        }
//        std::cout << "Time difference \nsec :" << time_difference << " \nnano sec: "
//                  << latest_img_time_ns << " - " << latest_imu_time_ns << " = " << time_difference_ns << "\n \n";
////        }
//        pub_img.publish(img);
//    }

    void ImuTesterNode::callback_imu(sensor_msgs::Imu imu) {
        if (imu_la_x_hist.size() == 9) {
            checkOutlier(imu_la_x_hist, imu.linear_acceleration.x, "la.x", 4);
            checkOutlier(imu_la_y_hist, imu.linear_acceleration.y, "la.y", 4);
            checkOutlier(imu_la_z_hist, imu.linear_acceleration.z, "la.z", 4);

            checkOutlier(imu_av_x_hist, imu.angular_velocity.x, "av.x", 1);
            checkOutlier(imu_av_y_hist, imu.angular_velocity.y, "av.y", 1);
            checkOutlier(imu_av_z_hist, imu.angular_velocity.z, "av.z", 1);

            checkOutlier(imu_o_x_hist, imu.orientation.x, "o.x", 1);
            checkOutlier(imu_o_y_hist, imu.orientation.y, "o.y", 1);
            checkOutlier(imu_o_z_hist, imu.orientation.z, "o.z", 1);
            checkOutlier(imu_o_w_hist, imu.orientation.w, "o.w", 1);
        }

        imu_la_x_hist.push_back(imu.linear_acceleration.x);
        imu_la_y_hist.push_back(imu.linear_acceleration.y);
        imu_la_z_hist.push_back(imu.linear_acceleration.z);

        imu_av_x_hist.push_back(imu.angular_velocity.x);
        imu_av_y_hist.push_back(imu.angular_velocity.y);
        imu_av_z_hist.push_back(imu.angular_velocity.z);

        imu_o_x_hist.push_back(imu.orientation.x);
        imu_o_y_hist.push_back(imu.orientation.y);
        imu_o_z_hist.push_back(imu.orientation.z);
        imu_o_w_hist.push_back(imu.orientation.w);
    }

    void ImuTesterNode::checkOutlier(boost::circular_buffer<double> cb, const double value, const std::string name,
                                     const int limit) const {
        std::vector<double> curVector(9);
        for (int i = 0; i < 9; ++i) {
            curVector[i] = cb[i];
        }

        sort(curVector.begin(), curVector.end());

        if (std::abs(curVector[4] - value) > limit) {
            std::cout << " !!!ALARM " << name << " is a outlier!!! \nmedian: " << std::__cxx11::to_string(curVector[4])
                      << " \nlast value: " << std::__cxx11::to_string(cb[8]) << " \ncurrent value: "
                      << std::__cxx11::to_string(value) << "\n";
//            for (int i = 0; i < 9; ++i) {
//                std::cout << " element " << i << ": " << std::to_string(cb[i]) << "\n";
//            }
        }
    }
}
