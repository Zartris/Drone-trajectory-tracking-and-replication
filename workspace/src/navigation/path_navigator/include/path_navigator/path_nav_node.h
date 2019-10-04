#ifndef PATH_NAVIGATOR_PATH_NAV_NODE_H
#define PATH_NAVIGATOR_PATH_NAV_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <martin_msg_lib/Segment.h>
#include <std_msgs/Empty.h>
#include <path_navigator/line.h>
#include <path_navigator/vector3d.h>
#include <path_navigator/bebop_command.h>
#include <nav_msgs/Path.h>
//std
#include <thread>
#include "MPC.h"

namespace path_nav {
    class PathNavNode {
    public:
        // for debugging outside ros.
        PathNavNode();

        // Ros constructor
        PathNavNode(ros::NodeHandle &n, ros::NodeHandle &n_private);

        virtual ~PathNavNode() = default;

        // CallBack methods
        void updateSegmentCallback(const martin_msg_lib::Segment &segment);

        void navigatorCallback(const geometry_msgs::PoseStamped &poseStamped);

        void navigatorMPCCallback(const geometry_msgs::PoseStamped &poseStamped);


        void patherToggleCallback(const std_msgs::Empty &empty);

        void run() {
            ros::spin();
        }

        void runPathNavigator();

        void publishMovement(const float &publish_loop_rate);

        void initParams();

        Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

        Eigen::VectorXd polyfit3d(Eigen::VectorXd xvals, Eigen::VectorXd yvals, Eigen::VectorXd zvals, int degree);

        MPC mpc;
        //private:
        // ROS specifics:
        ros::NodeHandle nh;
        ros::NodeHandle nh_private;

        ros::Subscriber segment_sub;
        ros::Subscriber dronePose_sub;
        ros::Subscriber pather_toggled_sub;
        ros::Publisher bebop_pub;
        ros::Publisher bebop_pub_twistStamp;
        ros::Publisher bebop_pub_debug;
        ros::Publisher refTrack_pub;
        ros::Publisher predictedTrack_pub;
        float settingLoopRate;
        tf::TransformListener tf_listener;

        // Our variables
        bool isNavigationActive = false;

        ros::Time previous_stamp = ros::Time(0, 0);
        geometry_msgs::Pose previous_Pose;
        path_nav_math::Vector3d previous_velocity = path_nav_math::Vector3d(99, 0, 0);
        double previous_delta = 0.0;
        double previous_a_x = 0.0;
        double previous_a_y = 0.0;
        double previous_a_z = 0.0;
        double Lf = 0.1;

        martin_msg_lib::Segment currentSegment;

        geometry_msgs::Point start;
        geometry_msgs::Point goal;
        path_nav_math::Line line = path_nav_math::Line();
        geometry_msgs::Pose dronePose;
        bebop_command::BebopCommand latestMoveCommand;
        geometry_msgs::Twist STOP_command;
        std::vector<geometry_msgs::Twist> command_queue;

        double droneSpeed = 0.05;
        double droneTurnSpeed = 0.2;

        mutable std::mutex setValue_mutex;


        //For testing purpose
        ros::Time start_time;
        bool debug = false;
        path_nav_math::Vector3d debugRotated = path_nav_math::Vector3d(0, 0, 0);
        path_nav_math::Vector3d debugMoveVectorWorld = path_nav_math::Vector3d(0, 0, 0);

        void test();


    };
}
#endif //PATH_NAVIGATOR_PATH_NAV_NODE_H


