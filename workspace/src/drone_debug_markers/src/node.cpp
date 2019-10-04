#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <martin_msg_lib/Segment.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64.h>

ros::Publisher marker_pub;
bool droneInitialized = false;
bool rangeInitialized = false;

void renderSegment(const martin_msg_lib::Segment msg) {
    ROS_INFO("Segment received.");
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "segment";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    std::vector<geometry_msgs::Point> points;
    points.push_back(msg.p1.position);
    points.push_back(msg.p2.position);
    marker.points = points;
    marker.scale.x = 0.01;
    marker.scale.y = 0.02;
    marker.lifetime = ros::Duration(0);
    std_msgs::ColorRGBA color;
    color.r = 255;
    color.g = 0;
    color.b = 0;
    color.a = 1;
    marker.color = color;
    marker_pub.publish(marker);
    ROS_INFO("Segment is rendered.");
}

void renderDroneMesh(const geometry_msgs::PoseStamped msg) {
    ROS_INFO_ONCE("Drone render msgs received!");
    if (!droneInitialized) {
        // Render mesh on TF
        visualization_msgs::Marker marker;
        // Set our initial shape type to be a cube
        uint32_t shape = visualization_msgs::Marker::MESH_RESOURCE;
        marker.header.frame_id = "bebop_body_scaled";
        marker.header.stamp = msg.header.stamp;

        marker.ns = "drone_mesh";
        marker.id = 0;

        marker.type = shape;

        marker.action = visualization_msgs::Marker::ADD;

        marker.frame_locked = true;

        geometry_msgs::Pose pose;
        pose.orientation.w = 0.707;
        pose.orientation.z = 0.707;
        marker.pose = pose;

        marker.scale.x = 0.035;
        marker.scale.y = 0.035;
        marker.scale.z = 0.035;

        std_msgs::ColorRGBA color;
        color.r = 200;
        color.g = 200;
        color.b = 200;
        color.a = 1;
        marker.color = color;

        marker.mesh_resource = "package://drone_debug_markers/meshes/Bebop.dae";

        marker_pub.publish(marker);
        ROS_INFO("Drone mesh placed.");
        droneInitialized = true;
    }
}

void renderDebugPoint(const geometry_msgs::PointStamped msg) {
    ROS_INFO_ONCE("debug Point received!");
    visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_link";
    marker.header.stamp = msg.header.stamp;
    marker.ns = "debug_point";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    std::vector<geometry_msgs::Point> points;
    geometry_msgs::Point start;
    points.push_back(start);
    points.push_back(msg.point);
    marker.points = points;
    marker.scale.x = 0.01;
    marker.scale.y = 0.02;
    marker.lifetime = ros::Duration(0);
    std_msgs::ColorRGBA color;
    color.r = 0;
    color.g = 0;
    color.b = 255;
    color.a = 1;
    marker.color = color;
    marker_pub.publish(marker);
}

void renderRangeSphere(const geometry_msgs::PoseStamped msg) {
    ROS_INFO_ONCE("Drone range msgs received!");
    if (!rangeInitialized) {
        // Render mesh on TF
        visualization_msgs::Marker marker;
        // Set our initial shape type to be a cube
        uint32_t shape = visualization_msgs::Marker::SPHERE;
        marker.header.frame_id = "camera_link";
        marker.header.stamp = msg.header.stamp;

        marker.ns = "drone_range";
        marker.id = 0;

        marker.type = shape;

        marker.action = visualization_msgs::Marker::ADD;

        marker.frame_locked = true;

        geometry_msgs::Pose pose;
        marker.pose = pose;
        std::string s = msg.header.frame_id;
        double scale;
        scale = std::stod(s, nullptr);
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;

        std_msgs::ColorRGBA color;
        color.r = 0;
        color.g = 200;
        color.b = 0;
        color.a = 0.2;
        marker.color = color;
        marker_pub.publish(marker);
        ROS_INFO("Drone range is placed.");
        rangeInitialized = true;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "drone_debug");
    ros::NodeHandle n;
    marker_pub = n.advertise<visualization_msgs::Marker>("/mesh_message", 1);
    ros::Subscriber pose_sub = n.subscribe("/orb_slam2/scaled/camera_pose", 10,
                                           renderDroneMesh);
    ros::Subscriber seg_sub = n.subscribe("/path_segmenter/path_segment", 1,
                                          renderSegment);
    ros::Subscriber debug_sub = n.subscribe("/bebop/debug", 1,
                                            renderDebugPoint);
    ros::Subscriber range_sub = n.subscribe("/bebop/range",1,renderRangeSphere);
    ros::spin();
}
