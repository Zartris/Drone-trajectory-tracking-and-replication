#include <boost/smart_ptr/shared_ptr.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <martin_msg_lib/Segment.h>
#include <nav_msgs/Path.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <std_msgs/Empty.h>
#include <fstream>

class pather_node_comm {
public:
    pather_node_comm() {
        // In an attempt to assume as little chaos as possible, don't be in sending mode at first
        isSending = false;

        latestPath = nav_msgs::Path();

        // Initialize distance threshold and path to file
        initParams();

        // Publisher of segments
        pub = nh.advertise<martin_msg_lib::Segment>("/path_segmenter/path_segment", 1000);

        // Publisher of toggles
        togglePub = nh.advertise<std_msgs::Empty>("/pather_toggled", 10);
        rangePub = nh.advertise<geometry_msgs::PoseStamped>("/bebop/range", 10);
        recordPath1Pub = nh.advertise<std_msgs::Empty>("/record_path_toggle1", 10);
        recordPath2Pub = nh.advertise<std_msgs::Empty>("/record_path_toggle2", 10);

        // Subscribers
        poseSub = nh.subscribe("/orb_slam2/scaled/camera_pose", 1000,
                               &pather_node_comm::poseCallback, this);
        pathSub = nh.subscribe("/orb_slam2/scaled/path1", 10,
                               &pather_node_comm::pathCallback, this);
        toggleSub = nh.subscribe("/mode_toggle", 10,
                                 &pather_node_comm::toggleCallback, this);
        recordSub = nh.subscribe("start_stop_recording", 10,
                                 &pather_node_comm::startStopRecording, this);
        savePathsSub = nh.subscribe("save_paths", 1,
                                    &pather_node_comm::saveLatestPath, this);
        startoverSub = nh.subscribe("startover", 1,
                                    &pather_node_comm::staroverCallback, this);
        // At first our status is "find the first point on the trajectory". A later addition may change this to
        // "find the closes point on the path and start from there"
        a = -1;
    }

    ///
    /// \brief Initializes the distance threshold and the path to the path file
    ///
    void initParams() {
        nh.param<std::string>("pathFile", pathToPath,
                              "~/Code/Drone_MARTIN/path.txt");
        nh.param<double>("squaredDistanceThreshold", squaredDistanceThreshold,
                         0.4); // TODO: Figure out a logical default for the squared distance threshold
        // Default navigation mode is single-flight
        nh.param("navigation_mode", navMode, 0);
        nh.param("load_path_from_file", loadPathFromFile, false);
    }

    ///
    /// \brief Save the path that is currently in memory to file
    ///
    void saveLatestPath(const std_msgs::Empty) {
        if (!latestPath.poses.empty()) {
            ROS_INFO("Saving new path...");
            std::ofstream out(pathToPath.data(),
                              std::ios_base::trunc | std::ios_base::out);
            for (int i = 0; i < latestPath.poses.size(); i++) {
                geometry_msgs::PoseStamped pose = latestPath.poses[i];
                out << pose.pose.position.x << " " << pose.pose.position.y
                    << " " << pose.pose.position.z << "\n";
            }
            out.close();
        } else {
            ROS_INFO(
                    "Latest recorded path is empty or has not yet been changed. Did you remember to switch to recording mode?");
        }
    }

    ///
    /// \brief Open the path file and attempt to load the path within into memory
    ///
    void loadPath() {
        path.clear();
        if (loadPathFromFile) {
            std::ifstream in(pathToPath.data(), std::ios_base::in);
            std::string line;
            ROS_INFO("Loading path file into memory...");
            ROS_INFO(pathToPath.data());
            if (!in.is_open())
                ROS_ERROR(
                        "Could not open path file. Maybe it never existed in the first place.");
            while (std::getline(in, line)) {
                std::istringstream iss(line);
                double x, y, z;
                if (!(iss >> x >> y >> z)) {
                    ROS_ERROR("Couldn't parse point");
                    break;
                } // Error
                geometry_msgs::Point newPoint;
                newPoint.x = x;
                newPoint.y = y;
                newPoint.z = z;
                path.push_back(newPoint);
            }
            // Do some print for my sake
            std::ostringstream oss;
            (oss << "Received " << path.size() << " points.");
            ROS_INFO(oss.str().data());
            oss.str("");
            oss.clear();
            oss << "\n";
            for (auto &i : path) {
                oss << i.x << ", " << i.y << ", " << i.z
                    << "\n";
                in.close();
            }
        } else {
            ROS_DEBUG("Loading in path from msgs");
            for (const geometry_msgs::PoseStamped &ps : latestPath.poses) {
                path.push_back(ps.pose.position);
            }
            ROS_DEBUG("DONE");
        }

        // TODO: At this point the path is full. We may prune the path however we wish before finishing.
    }

///
/// \brief Returns the squared euclidiean distance between points a and b
/// \param a First point
/// \param b Second point
/// \return The squared euclidean distance between a and b
///
    double squaredEuclideanDistance(geometry_msgs::Point a,
                                    geometry_msgs::Point b) {
        return pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2);
    }

///
/// \brief Call for whenever a call is made to the mode toggle
///
    void toggleCallback(const std_msgs::EmptyConstPtr &toggle) {
        std_msgs::Empty msg1;
        if (!isSending) {
            ROS_INFO("Switching to send mode...");
            loadPath();
            recordPath2Pub.publish(msg1);
        } else {
            ROS_INFO("Switching to map mode...");
            recordPath2Pub.publish(msg1);
        }
        std_msgs::Empty msg2;
        togglePub.publish(msg2);
        ROS_INFO(((!isSending) ? "isSending = true" : "isSending = false"));
        isSending = !isSending;
    }

///
/// \brief Call for when a call is made to start recording the path
///
    void startStopRecording(const std_msgs::EmptyConstPtr &poke) {
        if (isSending) {
            ROS_INFO(
                    "Node is currently in \"Send\" mode. Switch to \"Mapping\" mode to record a new path");
            return;
        }
        ROS_INFO(((!isRecording) ? "recording = true" : "recording = false"));

        if (isRecording) {
            ROS_INFO("%lu", latestPath.poses.size());
        }

        isRecording = !isRecording;
        std_msgs::Empty msg;
        recordPath1Pub.publish(msg);
    }

///
/// \brief Call for whenever a new pose enters. Publishes new path segment if the last one has been cleared
/// \param pose Camera pose
///
    void poseCallback(const geometry_msgs::PoseStamped pose) {

        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = std::to_string(squaredDistanceThreshold);
        ps.header.stamp = pose.header.stamp;
        rangePub.publish(ps);

        if (isSending) {
            // This is called whenever a pose comes in. The pose can be found in...well... "pose".
            martin_msg_lib::Segment segment;
            bool publish = false;
            segment.header.stamp = ros::Time::now();
            if (a < 0) {
                publish = true;
                segment.p1 = pose.pose;
                segment.p2.position = path[0];
                segment.p3.position = path[1];
                a = 0;
            } else {
                while (squaredEuclideanDistance(pose.pose.position, path[a])
                       < squaredDistanceThreshold && a <= path.size()) {
                    publish = true;
                    segment.p1.position = path[a % path.size()];
                    segment.p2.position = path[(a + 1) % path.size()];
                    segment.p3.position = path[(a + 2) % path.size()];
                    a++;
                    if (navMode == 0 && a == path.size() - 1) {
                        ROS_INFO("I'm DONE!");
                        std_msgs::Empty msg;
                        a++;
                    }
                }
            }
            if (publish) {
                pub.publish(segment);
                ROS_DEBUG("Segment send");
            }
        } else {
            ROS_DEBUG(
                    "Received pose and ignored it. I'm in my rebellious mode.");
        }
    }

///
/// \brief Upon receiving a path, save latest path to memory
///
    void pathCallback(const nav_msgs::Path &path) {
        if (isRecording) {
            latestPath = path;
        }
    }


    ///
/// \brief Upon receiving a path, save latest path to memory
///
    void staroverCallback(const std_msgs::Empty &empty) {
        ROS_INFO("STARTOVER CLICKED!");
        a = -1;
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Publisher togglePub;
    ros::Publisher rangePub;
    ros::Publisher recordPath1Pub;
    ros::Publisher recordPath2Pub;
    ros::Subscriber startoverSub;
    ros::Subscriber poseSub;
    ros::Subscriber pathSub;
    ros::Subscriber toggleSub;
    ros::Subscriber recordSub;
    ros::Subscriber savePathsSub;
    std::string pathToPath;
    nav_msgs::Path latestPath;
    std::vector<geometry_msgs::Point> path;
    int navMode;
    double squaredDistanceThreshold;
    int a;
    bool isSending;
    bool isRecording;
    bool loadPathFromFile;
};

int main(int argc, char **argv) {
    ROS_INFO("INITIALIZING PATH SEGMENTER NODE...");
    ros::init(argc, argv, "path_segmenter");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    pather_node_comm comm;
    ros::spin();
    return 0;
}
