#include <boost/smart_ptr/shared_ptr.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <boost/circular_buffer.hpp>
#include <tf/transform_listener.h>

class controlBasedScalingNode {
public:
    controlBasedScalingNode() {
        // Initialization
        initParams();

        testHits = 0;
        estimateSamples = 0;
        total = 0;
        poseCount = 0;
        init = false;
        estimates = boost::circular_buffer<double>(40);

        poseSub = nh.subscribe("/orb_slam2/bebop_pose", 10,
                               &controlBasedScalingNode::poseCallback, this);
        controlSub = nh.subscribe("/bebop/cmd_vel", 10,
                                  &controlBasedScalingNode::twistCallback, this);
        scalePub = nh.advertise<std_msgs::Float32>("/scale_estimate", 10);
    }

    void initParams() {
        // nh.param<double>("squaredDistanceThreshold", squaredDistanceThreshold, 0.4)
        nh.param<int>("sampleThreshold", sampleThreshold, 20);
        nh.param<int>("hitThreshold", hitThreshold, 20);
    }

    void poseCallback(const geometry_msgs::PoseStamped pose) {
        if (!init) {
            lastPose = pose;
            init = true;
            ROS_DEBUG("Initialized!");
            poseCount++;
            return;
        }
        if (testHits > hitThreshold) {
            // Obtain the deltas
            double tD = (pose.header.stamp - lastPose.header.stamp).toSec();
            double pD = euclideanDistance(pose.pose.position, lastPose.pose.position);
            //Calculate pD / tD
            double scaleEstimate = pD / (tD * vectorLength(currentTest));
            ROS_DEBUG("%6.4lf", scaleEstimate);
            total += scaleEstimate;
            estimateSamples++;
            estimates.push_back(scaleEstimate);
            //ROS_DEBUG("tD: %6.4lf", tD);
            //ROS_DEBUG("pD: %6.4lf", pD);
            // TF based pD
//            try {
//
//                tf_listener.waitForTransform("map", "camera_link",
//                                             pose.header.stamp, ros::Duration(0.1));
//                geometry_msgs::PoseStamped pointStamped_camera_link_scaled;
//                tf_listener.transformPose("camera_link", lastPose, pointStamped_camera_link_scaled);
//
//                double pD = fabs(pointStamped_camera_link_scaled.pose.position.x);
//                //double pD = euclideanDistance(pose.pose.position,
//                //		lastPose.pose.position);
//                // Calculate pD/tD
//                double scaleEstimate = pD / (tD * vectorLength(currentTest));
//                ROS_DEBUG("%6.4lf", scaleEstimate);
//                total += scaleEstimate;
//                estimateSamples++;
//                estimates.push_back(scaleEstimate);
//                //ROS_DEBUG("tD: %6.4lf", tD);
//                //ROS_DEBUG("pD: %6.4lf", pD);
//            }
//            catch (tf::TransformException ex) {
//                ROS_ERROR("%s", ex.what());
//                ros::Duration(1.0).sleep();
//            }
        }
        if (poseCount % 10 == 0)
            lastPose = pose;
        poseCount++;
        if (estimateSamples > sampleThreshold) {
            std_msgs::Float32 msg;
            msg.data = total / estimateSamples;
            scalePub.publish(msg);

            double ssd = 0;
            for (int i = 0; i < estimates.size(); i++) {
                ssd += (pow(estimates[i] - msg.data, 2));
            }
            ROS_DEBUG("STDEV.: %6.4lf, Scale estimate: %6.4lf", sqrt(ssd / sampleThreshold), msg.data);
        }

    }

    void twistCallback(const geometry_msgs::Twist control) {
        if (currentTest.linear.x == control.linear.x
            && currentTest.linear.y == control.linear.y
            && currentTest.linear.z == control.linear.z) {
            testHits++;
        } else if (control.linear.x + control.linear.y + control.linear.z > 0) {
            currentTest = control;
            testHits = 0;
            ROS_DEBUG("New test");
        } else {
            testHits = 0;
        }
    }

    double euclideanDistance(geometry_msgs::Point p1, geometry_msgs::Point p2) {
        return sqrt(
                pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2) + pow(p2.z - p1.z, 2));
    }

    double vectorLength(geometry_msgs::Twist t) {
        return sqrt(
                pow(t.linear.x, 2) + pow(t.linear.y, 2) + pow(t.linear.z, 2));
    }

private:
    ros::NodeHandle nh;
    ros::Publisher scalePub;
    ros::Subscriber controlSub;
    ros::Subscriber poseSub;
    geometry_msgs::Twist currentTest;

    int testHits;
    int estimateSamples;
    double total;
    geometry_msgs::PoseStamped lastPose;
    int sampleThreshold;
    int hitThreshold;
    bool init;
    int poseCount;
    tf::TransformListener tf_listener;
    boost::circular_buffer<double> estimates;
};

int main(int argc, char **argv) {
    ROS_INFO("Initializing scaler...");
    ros::init(argc, argv, "control_based_scale_estimator");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                   ros::console::levels::Debug);
    controlBasedScalingNode node;
    ros::spin();
    return 0;
}
