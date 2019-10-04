#include <path_navigator/path_nav_node.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_nav_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    path_nav::PathNavNode node(nh, nh_private);
    if (true) {
        ros::MultiThreadedSpinner spinner(2);
        node.settingLoopRate = 15; //Hz
        ROS_INFO("RUNNING\n");
        std::thread navigationThread(&path_nav::PathNavNode::runPathNavigator, &node);
        std::thread publisherThread(&path_nav::PathNavNode::publishMovement, &node, node.settingLoopRate);
        spinner.spin();

    } else {
        node.test();
    }

    return 0;
}

path_nav::PathNavNode::PathNavNode() = default;

path_nav::PathNavNode::PathNavNode(ros::NodeHandle &n, ros::NodeHandle &n_private)
        : nh(n), nh_private(n_private) {
    ROS_INFO("INIT\n");
    initParams();

    segment_sub = nh.subscribe("/path_segmenter/path_segment", 5, &path_nav::PathNavNode::updateSegmentCallback, this);
//    dronePose_sub = nh.subscribe("/orb_slam2/camera_pose", 5, &path_nav::PathNavNode::navigatorCallback, this);
    dronePose_sub = nh.subscribe("/orb_slam2/camera_pose", 5, &path_nav::PathNavNode::navigatorCallback, this);
    pather_toggled_sub = nh.subscribe("/pather_toggled", 1, &path_nav::PathNavNode::patherToggleCallback, this);
    bebop_pub = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1);
    bebop_pub_twistStamp = nh.advertise<geometry_msgs::TwistStamped>("/bebop/debugTwist", 1);
    bebop_pub_debug = nh.advertise<geometry_msgs::PointStamped>("/bebop/debug", 1);

    refTrack_pub= nh.advertise<nav_msgs::Path>("/path_nav/MPC/refTrack", 1);
    predictedTrack_pub = nh.advertise<nav_msgs::Path>("/path_nav/MPC/predictedTrack", 1);
}

// CallBack methods
void path_nav::PathNavNode::updateSegmentCallback(const martin_msg_lib::Segment &segment) {
    ROS_DEBUG("Segment received");
    // lock
    std::lock_guard<std::mutex> lock(setValue_mutex);
    start = segment.p1.position;
    goal = segment.p2.position;
    line.overrideValues(start, goal);
    bebop_pub.publish(STOP_command);
    // unlocks when out of scope.
    currentSegment = segment;
}

void path_nav::PathNavNode::navigatorCallback(const geometry_msgs::PoseStamped &poseStamped) {
    ROS_DEBUG("DronePose Received!");
    if (!line.initialized) return;
    if (!isNavigationActive) return;
    ROS_DEBUG("Processing DronePose.");

    geometry_msgs::Point camera_Pos = poseStamped.pose.position;

    // compute vector from drone to goal point:
    path_nav_math::Vector3d droneToGoal(goal.x - camera_Pos.x,
                                        goal.y - camera_Pos.y,
                                        goal.z - camera_Pos.z);

    tf::StampedTransform transform;
    geometry_msgs::PointStamped goalPoint_map;
    goalPoint_map.header.stamp = poseStamped.header.stamp;
    goalPoint_map.header.frame_id = poseStamped.header.frame_id;
    goalPoint_map.point = goal;
    geometry_msgs::PointStamped goalPoint_camera;

    path_nav_math::Vector3d driftCorrection_map = line.findShortestVectorFromPointToLine(camera_Pos);
//    path_nav_math::Vector3d driftCorrection = line.findShortestVectorFromPointToLine(camera_Pos);
    geometry_msgs::PointStamped driftCorrectionPoint_map;
    driftCorrectionPoint_map.header.stamp = poseStamped.header.stamp;
    driftCorrectionPoint_map.header.frame_id = poseStamped.header.frame_id;
    driftCorrectionPoint_map.point.x = driftCorrection_map.x;
    driftCorrectionPoint_map.point.y = driftCorrection_map.y;
    driftCorrectionPoint_map.point.z = driftCorrection_map.z;
    geometry_msgs::PointStamped driftCorrectionPoint_camera;

    try {

        tf_listener.waitForTransform("map", "bebop_body",
                                     poseStamped.header.stamp, ros::Duration(0.1));
        tf_listener.transformPoint("camera_link", goalPoint_map, goalPoint_camera);
        tf_listener.transformPoint("camera_link", driftCorrectionPoint_map, driftCorrectionPoint_camera);

        tf_listener.lookupTransform("map", "bebop_body", poseStamped.header.stamp, transform);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ROS_ERROR("Lf not computed!");
        ros::Duration(1.0).sleep();
    }

    bebop_pub_debug.publish(driftCorrectionPoint_camera);
//    std::string s;
//    s.append("\nTransform from map to bebop_body:\n");
//    s.append("Translate: (" + std::to_string(transform.getOrigin().x()) + "," +
//             std::to_string(transform.getOrigin().y()) +
//             "," + std::to_string(transform.getOrigin().z()) + ")\n");
//    s.append("Rotation: (" + std::to_string(transform.getRotation().x()) + "," +
//             std::to_string(transform.getRotation().y()) + "," + std::to_string(transform.getRotation().z()) + "," +
//             std::to_string(transform.getRotation().w()) + ")\n");
//    ROS_DEBUG(s.data());

    double pScale = line.p_scale;
    if (pScale > 0.6) {
        pScale = 1;
    } else if (pScale > 0.3) {
    } else if (pScale >= 0.0) {
        pScale = 0.1;
    }


    path_nav_math::Vector3d bodyToGoal = path_nav_math::Vector3d(goalPoint_camera.point.x * pScale,
                                                                 goalPoint_camera.point.y * pScale,
                                                                 goalPoint_camera.point.z * pScale);


    // find drift vector:

    // final move vector in world coordinates:
    path_nav_math::Vector3d driftCorrection_camera(driftCorrectionPoint_camera.point.x,
                                                   driftCorrectionPoint_camera.point.y,
                                                   driftCorrectionPoint_camera.point.z);

    bodyToGoal = bodyToGoal.add(driftCorrection_camera);


    // Find turn force of the drone
    // This is done by finding the angle between drone x-axis and the rotated vector.
    path_nav_math::Vector3d d_Xaxis = path_nav_math::Vector3d(1, 0, 0);

    Eigen::Vector2d v1 = Eigen::Vector2d(1, 0);
    Eigen::Vector2d v2 = Eigen::Vector2d(bodyToGoal.getX(), bodyToGoal.getY());

    v2.normalize();

    double radian = atan2(bodyToGoal.getY(), bodyToGoal.getX());
    double turnForce = 0.0;
    if (radian < 0) {
        // turn right instead of left.
        turnForce = (-1 > radian) ? -1 : radian;
    } else {
        turnForce = radian > 1 ? 1 : radian;
    }
    // Change the speed of the drone.
    if (bodyToGoal.getX() != 0 || bodyToGoal.getY() != 0 || bodyToGoal.getZ() != 0) {
        // normalizing a (0,0,0) vector is not happening.
//        bodyToGoal.normalize();
    }
    std::string command_string;
    command_string.append(
            "BEFORE ANYTHING \n Direction: (" + std::to_string(bodyToGoal.x) + "," + std::to_string(bodyToGoal.y) +
            "," +
            std::to_string(bodyToGoal.z) + ")\n");
    command_string.append("Rotation: (" + std::to_string(turnForce) + ")\n");
    ROS_DEBUG(command_string.data());


    // Scale the speed:
    double max = bodyToGoal.getMax();

    if (max > droneSpeed) {
        bodyToGoal.x = (bodyToGoal.x * droneSpeed) / max;
        bodyToGoal.y = (bodyToGoal.y * droneSpeed) / max;
        bodyToGoal.z = (bodyToGoal.z * droneSpeed) / max;
    }

    // Stop turning when we are very close to goal:
    if (max <= 0.003) {
        turnForce = 0;
    }

    std::string command_string2;
    command_string2.append(
            "AFTER SCALE \n Direction: (" + std::to_string(bodyToGoal.x) + "," + std::to_string(bodyToGoal.y) + "," +
            std::to_string(bodyToGoal.z) + ")\n");
    command_string2.append("Rotation: (" + std::to_string(turnForce) + ")\n");
    ROS_DEBUG(command_string2.data());

    geometry_msgs::Twist command = geometry_msgs::Twist();


    // The more we turn the less move we do
    command.linear.x =
            bodyToGoal.getX(); // * (1 - std::fabs(turnForce) * 0.8); // adding dronespeed to have a max speed.
    command.linear.y = bodyToGoal.getY(); //* (1 - std::fabs(turnForce) * 0.8);
    command.linear.z = bodyToGoal.getZ(); //* (1 - std::fabs(turnForce) * 0.3);
//    command.angular.z = turnForce * droneTurnSpeed; // Maybe change this to a droneTurnSpeed

    // Another layer of debugging.
    if (debug) {
        std::string s;

        s.append("\n############# INFO ############# \n");

        s.append("\ndrone POSE:\n");
        s.append("position: (" + std::to_string(dronePose.position.x) + "," + std::to_string(dronePose.position.y) +
                 "," + std::to_string(dronePose.position.z) + ")\n");
        s.append("orientation: (" + std::to_string(dronePose.orientation.x) + "," +
                 std::to_string(dronePose.orientation.y) + "," + std::to_string(dronePose.orientation.z) + "," +
                 std::to_string(dronePose.orientation.w) + ")\n");

        s.append("\nLine:\n");
        s.append(line.toString());

        s.append("\nVector from drone to goal (world)\n");
        s.append(droneToGoal.toString());

        s.append("\nVector from drone to goal (drone)\n");
        s.append(debugRotated.toString());

        s.append("\nradians between drone front (x-axis) and rotated vector \n");
        s.append(std::to_string(radian));


        s.append("\n############# SENDING COMMAND ############# \n");
        s.append("Direction: (" + std::to_string(command.linear.x) + "," + std::to_string(command.linear.y) + "," +
                 std::to_string(command.linear.z) + ")\n");
        s.append("Rotation: (" + std::to_string(command.angular.z) + ")\n");
        ROS_DEBUG(s.data());
    }

    // Timestamp is only for debugging and showing in rviz
    latestMoveCommand.setFrameTime(poseStamped.header.stamp);

    // Publish the command if sucessful
    if (std::isnan(command.linear.x) || std::isnan(command.linear.y) || std::isnan(command.linear.z) ||
        std::isnan(command.angular.z)) {
        ROS_WARN("NAN is detected in command!");
        latestMoveCommand.setCommand(STOP_command);
    } else {
        latestMoveCommand.setCommand(command);
        std::string command_string;
        command_string.append(
                "Direction: (" + std::to_string(command.linear.x) + "," + std::to_string(command.linear.y) + "," +
                std::to_string(command.linear.z) + ")\n");
        command_string.append("Rotation: (" + std::to_string(command.angular.z) + ")\n");
        ROS_INFO(command_string.data());
    }
}


void path_nav::PathNavNode::navigatorMPCCallback(const geometry_msgs::PoseStamped &poseStamped) {
    ROS_DEBUG("DronePose Received!");
    //**************************************************************
    //* GET THE CURRENT STATE
    //**************************************************************

    if (previous_stamp.isZero()) {
        try {

            tf_listener.waitForTransform("base_link", "camera_base_link",
                                         poseStamped.header.stamp, ros::Duration(0.1));
            geometry_msgs::PointStamped pointStamped_camera_base_link;
            pointStamped_camera_base_link.header.stamp = poseStamped.header.stamp;
            pointStamped_camera_base_link.header.frame_id = "camera_base_link";
            pointStamped_camera_base_link.point.x = 0;
            pointStamped_camera_base_link.point.y = 0;
            pointStamped_camera_base_link.point.z = 0;
            geometry_msgs::PointStamped pointStamped_base_link;

            tf_listener.transformPoint("base_link", pointStamped_camera_base_link, pointStamped_base_link);
            Lf = sqrt(pow(pointStamped_base_link.point.x, 2) + pow(pointStamped_base_link.point.y, 2));

        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        previous_stamp = poseStamped.header.stamp;
        previous_Pose = poseStamped.pose;

        return;
    }

    double deltaTime = (poseStamped.header.stamp - previous_stamp).toSec();
    const double px = poseStamped.pose.position.x;
    const double py = poseStamped.pose.position.x;
    const double pz = poseStamped.pose.position.z;

    // if we lost tracking we have no idea of delta.
    const double delta = deltaTime > 0.5 ? 0 : previous_delta;
    // compute the yaw
    auto q = poseStamped.pose.orientation;
    const double psi = atan2(2.0 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
    auto q2 = previous_Pose.orientation;
    const double previous_psi = atan2(2.0 * (q2.y * q2.z + q2.w * q2.x),
                                      q2.w * q2.w - q2.x * q2.x - q2.y * q2.y + q2.z * q2.z);

    // Compute velocity
//    const double v_x = (px - previous_Pose.position.x) / (deltaTime);
//    const double v_y = (py - previous_Pose.position.y) / (deltaTime);
//    const double v_z = (pz - previous_Pose.position.z) / (deltaTime);
    // https://en.wikipedia.org/wiki/Angular_velocity
    const double v_angular_rad = (psi - previous_psi) / (deltaTime);
    geometry_msgs::PoseStamped previous_pose_cam;
    try {

        tf_listener.waitForTransform("map", "camera_link",
                                     poseStamped.header.stamp, ros::Duration(0.1));
        tf_listener.transformPose("camera_link", poseStamped, previous_pose_cam);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    const double v_x_c = (- previous_Pose.position.x) / (deltaTime);
    const double v_y_c = (- previous_Pose.position.y) / (deltaTime);
    const double v_z_c = (- previous_Pose.position.z) / (deltaTime);

    if (previous_velocity.x == 99) {
        previous_velocity.setVector(v_x_c, v_y_c, v_z_c);
        return;
    }
    // compute the acceleration

    // TODO::: FIX ACCEL IF NEED LATER (make it cam)!
//    const double a_x = (v_x - previous_velocity.x) / deltaTime;
//    const double a_y = (v_y - previous_velocity.y) / deltaTime;
//    const double a_z = (v_z - previous_velocity.z) / deltaTime;

    if (!line.initialized) return;
    if (!isNavigationActive) return;
    ROS_DEBUG("Processing DronePose.");

    geometry_msgs::Point camera_Pos = poseStamped.pose.position;



    // This is the path we want to follow.
    // Todo:: Maybe just a few points so we don't have to fit a poly on the hole path.
    std::vector<double> points_xs(3);
    std::vector<double> points_ys(3);
    std::vector<double> points_zs(3);
    points_xs[0] = currentSegment.p1.position.x;
    points_ys[0] = currentSegment.p1.position.y;
    points_zs[0] = currentSegment.p1.position.z;

    points_xs[1] = currentSegment.p2.position.x;
    points_ys[1] = currentSegment.p2.position.y;
    points_zs[1] = currentSegment.p2.position.z;

    points_xs[2] = currentSegment.p3.position.x;
    points_ys[2] = currentSegment.p3.position.y;
    points_zs[2] = currentSegment.p3.position.z;



//**************************************************************
//* CONVERT WAYPOINTS TO DRONE SPACE as VectorXd from GLOBAL SPACE
//**************************************************************
    const int NUMBER_OF_WAYPOINTS = points_xs.size();
    Eigen::VectorXd waypoints_xs(NUMBER_OF_WAYPOINTS);
    Eigen::VectorXd waypoints_ys(NUMBER_OF_WAYPOINTS);
    Eigen::VectorXd waypoints_zs(NUMBER_OF_WAYPOINTS);
    Eigen::VectorXd waypoints_ts(NUMBER_OF_WAYPOINTS);
    martin_msg_lib::Segment segmentInCamCoords;
    segmentInCamCoords.header.frame_id = "camera_link";
    segmentInCamCoords.header.stamp = poseStamped.header.stamp;
    try {

        tf_listener.waitForTransform("map", "bebop_body",
                                     poseStamped.header.stamp, ros::Duration(0.1));

        geometry_msgs::PoseStamped currSegStamp;
        currSegStamp.pose = currentSegment.p1;
        currSegStamp.header.frame_id ="map";
        currSegStamp.header.stamp = poseStamped.header.stamp;
        currSegStamp.pose.orientation.x = 0;
        currSegStamp.pose.orientation.y = 0;
        currSegStamp.pose.orientation.z = 0;
        currSegStamp.pose.orientation.w = 1;
        geometry_msgs::PoseStamped p;
        tf_listener.transformPose("camera_link", currSegStamp, p);
        segmentInCamCoords.p1 = p.pose;
        currSegStamp.pose.position = currentSegment.p2.position;
        tf_listener.transformPose("camera_link", currSegStamp, p);
        segmentInCamCoords.p2 = p.pose;
        currSegStamp.pose.position = currentSegment.p3.position;
        tf_listener.transformPose("camera_link", currSegStamp, p);
        segmentInCamCoords.p3 = p.pose;

        for (int i = 0; i < NUMBER_OF_WAYPOINTS; ++i) {
            geometry_msgs::PointStamped pointStamped_map;
            pointStamped_map.header.stamp = poseStamped.header.stamp;
            pointStamped_map.header.frame_id = poseStamped.header.frame_id;
            pointStamped_map.point.x = points_xs[i];
            pointStamped_map.point.y = points_ys[i];
            pointStamped_map.point.z = points_zs[i];
            geometry_msgs::PointStamped pointStamped_camera;

            tf_listener.transformPoint("camera_link", pointStamped_map, pointStamped_camera);

//
//            const double dx = points_xs[i] - px;
//            const double dy = points_ys[i] - py;
//            const double dz = points_zs[i] - pz;

            waypoints_xs[i] = pointStamped_camera.point.x;
            waypoints_ys[i] = pointStamped_camera.point.y;
            waypoints_zs[i] = pointStamped_camera.point.z;
            waypoints_ts[i] = i;
        }
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }


//**************************************************************
//* FIT POLYNOMAL
//**************************************************************
    const int ORDER = 2;
//    auto K = polyfit(waypoints_xs, waypoints_ys, ORDER);
    Eigen::VectorXd K_x = polyfit(waypoints_ts, waypoints_xs, ORDER);
    Eigen::VectorXd K_y = polyfit(waypoints_ts, waypoints_ys, ORDER);
    Eigen::VectorXd K_z = polyfit(waypoints_ts, waypoints_zs, ORDER);

//**************************************************************
//* GET POINTS TO DISPLAY FROM OUR FITTED POLYNOMIAL (ROAD CURVE)
//**************************************************************

    // Find current t where 0<t<1 even tho we support t<2 do we want to hit the point t=1 no matter what:
    line.findShortestVectorFromPointToLine(camera_Pos);
    double t_cur = line.getTCurCapped();

    std::vector<double> next_xs(N);
    std::vector<double> next_ys(N);
    std::vector<double> next_zs(N);
    const double D = 0.05;
    nav_msgs::Path path;
    path.header.frame_id = "camera_link";
    for (int i = 0; i < N; ++i) {
        const double dt = D * i + t_cur;
        const double dx = K_x[2] * dt * dt + K_x[1] * dt + K_x[0];
        const double dy = K_y[2] * dt * dt + K_y[1] * dt + K_y[0];
        const double dz = K_z[2] * dt * dt + K_z[1] * dt + K_z[0];

        next_xs[i] = dx;
        next_ys[i] = dy;
        next_zs[i] = dz;

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = dx;
        pose.pose.position.y = dy;
        pose.pose.position.z = dz;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;
        pose.header.frame_id = "camera_link";
        path.poses.push_back(pose);

    }
    refTrack_pub.publish(path);
//**************************************************************
//* GENERATE CURRENT ERROR ESTIMATES (cte, epsi)
//**************************************************************

    // current CTE is fitted polynomial (road curve) evaluated at px = 0.0
    // f = K[3] * px0 * px0 + px0 + K[2] * px0 * px0 + K[1] * px0 + K[0];
    const double cte_track_x = K_x[2] * t_cur * t_cur + K_x[1] * t_cur + K_x[0];
    const double cte_track_y = K_y[2] * t_cur * t_cur + K_y[1] * t_cur + K_y[0];
    const double cte_track_z = K_z[2] * t_cur * t_cur + K_z[1] * t_cur + K_z[0];

    // goal is the position where t = 1
    const double cte_goal_x = K_x[2] + K_x[1] + K_x[0];
    const double cte_goal_y = K_y[2] + K_y[1] + K_y[0];
    const double cte_goal_z = K_z[2] + K_z[1] + K_z[0];

    // current heading error epsi is the tangent to the road curve at px = 0.0
    // epsi = arctan(f') where f' is the derivative of the fitted polynomial
    // f' = 3.0 * K[3] * px0 * px0 + 2.0 * K[2] * px0 + K[1]
    // todo:: not sure if rotation is correct. (TEST IT)
    const double epsi = -atan(2.0 * K_y[2] * t_cur + K_y[1]);
//**************************************************************
//* GET THE CURRENT DELAYED STATE
//**************************************************************

    const double dt = 0.1;

    // current state must be in vehicle coordinates with the delay factored in
    // kinematic model is at play here
    // note that at current state at vehicle coordinates:
    // px, py, psi = 0.0, 0.0, 0.0
    // note that in drones coordinates it is going in all x-axis.
    // which means position in drones's coordinate can change on all axis.

    // notice:: we are adding angular velocity to the velocity (MIGHT BE WRONG)

    // Compute the angular velocity in x and y axis
    // hints -- https://www.youtube.com/watch?v=Ta8cKqltPfU
    double angular_v_x = cos(v_angular_rad * dt) * Lf;
    double angular_v_y = sin(v_angular_rad * dt) * Lf;

    const double current_px = 0.0 + v_x_c * dt * CppAD::cos(epsi); // todo:: maybe multiply it on;
    const double current_py = 0.0 + v_y_c * dt * CppAD::sin(epsi);
    const double current_pz = 0.0 + v_z_c * dt;

    const double current_v_x = v_x_c + previous_a_x * dt;
    const double current_v_y = v_y_c + previous_a_y * dt;
    const double current_v_z = v_z_c + previous_a_z * dt;

    const double current_cte_track_x = cte_track_x + v_x_c * cos(epsi) * dt;
    const double current_cte_track_y = cte_track_y + v_y_c * sin(epsi) * dt;
    const double current_cte_track_z = cte_track_z + v_z_c * tan(epsi) * dt;

    const double current_cte_goal_x = cte_goal_x + v_x_c * cos(epsi) * dt;
    const double current_cte_goal_y = cte_goal_y + v_y_c * sin(epsi) * dt;
    const double current_cte_goal_z = cte_goal_z + v_z_c * tan(epsi) * dt;
    // the steering angle is negative the given value as we have
    // as recall that during transformation we rotated all waypoints by -psi

    //const double current_psi = 0.0 + v_x * (-delta) / Lf * dt; --old
    const double current_psi = psi + v_angular_rad * dt;
    const double current_epsi = epsi + v_angular_rad * dt;

    const int NUMBER_OF_STATES = 14;
    Eigen::VectorXd state(NUMBER_OF_STATES);
    state << current_px, current_py, current_pz,
            current_psi,
            current_v_x, current_v_y, current_v_z,
            current_cte_track_x, current_cte_track_y, current_cte_track_z,
            current_cte_goal_x, current_cte_goal_y, current_cte_goal_z,
            current_epsi;

//**************************************************************
//* DETERMINE NEXT COURSE OF ACTION AND PREDICTED STATES
//* USING MODEL PREDICTIVE CONTROL
//**************************************************************

    mpc.solve(state, K_x, K_y, K_z, segmentInCamCoords, Lf);

    geometry_msgs::Twist command = geometry_msgs::Twist();


    // The more we turn the less move we do
    command.linear.x = mpc.a_x;
    command.linear.y = mpc.a_y;
    command.linear.z = mpc.a_z;
    command.angular.z = mpc.yaw_rotation;

    previous_a_x = mpc.a_x;
    previous_a_y = mpc.a_y;
    previous_a_z = mpc.a_z;
    previous_delta= mpc.yaw_rotation;

    // Timestamp is only for debugging and showing in rviz
    latestMoveCommand.setFrameTime(poseStamped.header.stamp);

    // Publish the command if sucessful
    if (std::isnan(command.linear.x) || std::isnan(command.linear.y) || std::isnan(command.linear.z) ||
        std::isnan(command.angular.z)) {
        ROS_WARN("NAN is detected in command!");
        latestMoveCommand.setCommand(STOP_command);
    } else {
        latestMoveCommand.setCommand(command);
        std::string command_string;
        command_string.append(
                "Direction: (" + std::to_string(command.linear.x) + "," + std::to_string(command.linear.y) + "," +
                std::to_string(command.linear.z) + ")\n");
        command_string.append("Rotation: (" + std::to_string(command.angular.z) + ")\n");
        ROS_INFO(command_string.data());
    }

    auto fxs = mpc.future_xs;
    auto fys = mpc.future_ys;
    auto fzs = mpc.future_zs;

    nav_msgs::Path path_predict;
    path_predict.header.frame_id = "camera_link";
    for (int i = 0; i < N; ++i) {
        auto dx = fxs[i];
        auto dy = fys[i];
        auto dz = fzs[i];
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = dx;
        pose.pose.position.y = dy;
        pose.pose.position.z = dz;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;
        pose.header.frame_id = "camera_link";
        path_predict.poses.push_back(pose);
    }

    predictedTrack_pub.publish(path_predict);
}


void path_nav::PathNavNode::patherToggleCallback(const std_msgs::Empty &empty) {
    isNavigationActive = !isNavigationActive;
    if (isNavigationActive) {
        ROS_INFO("NAVIGATION MODULE ACTIVATED");
    } else {
        ROS_INFO("NAVIGATION MODULE DEACTIVATED");
        bebop_pub.publish(STOP_command);
    }
}


void path_nav::PathNavNode::runPathNavigator() {

}

void path_nav::PathNavNode::publishMovement(const float &publish_loop_rate) {
    ros::Rate loop_rate(publish_loop_rate);
    while (ros::ok()) {
//        loop_rate.sleep();
        if (latestMoveCommand.isNewCommand) {
            auto twist = latestMoveCommand.getCommand();
            bebop_pub.publish(twist);
            auto debug_msg = geometry_msgs::TwistStamped();
            debug_msg.header.stamp = latestMoveCommand.frame_time;
            debug_msg.header.frame_id = "camera_link";
            debug_msg.twist = twist;
            bebop_pub_twistStamp.publish(debug_msg);
        }
    }
}

void path_nav::PathNavNode::initParams() {
    // Setup all the parameters and the default values.
    nh_private.param<bool>("debug", debug, false);
    nh_private.param<float>("settingLoopRate", settingLoopRate, 15);
    // between 1 and 0.
    nh_private.param<double>("droneSpeed", droneSpeed, 0.03);

    if (droneSpeed <= 0) {
        droneSpeed = 0.01;
    } else if (droneSpeed > 1) {
        droneSpeed = 1;
    }

    nh_private.param<double>("droneTurnSpeed", droneTurnSpeed, droneSpeed);

    STOP_command = geometry_msgs::Twist();
    STOP_command.angular.x = 0;
    STOP_command.angular.y = 0;
    STOP_command.angular.z = 0;

    STOP_command.linear.x = 0;
    STOP_command.linear.y = 0;
    STOP_command.linear.z = 0;
    latestMoveCommand.setCommand(STOP_command);

    mpc = MPC();

}

// Fit a polynomial.
// Adapted from https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd path_nav::PathNavNode::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); ++i) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); ++j) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}


// Fit a polynomial.
// Adapted from https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
// https://stackoverflow.com/questions/45350891/fitting-a-polynomial-using-np-polyfit-in-3-dimensions
Eigen::VectorXd
path_nav::PathNavNode::polyfit3d(Eigen::VectorXd xvals, Eigen::VectorXd yvals, Eigen::VectorXd zvals, int degree) {
    assert(xvals.size() == yvals.size());
    assert(yvals.size() == zvals.size());
    assert(degree >= 1 && degree <= xvals.size() - 1);
    Eigen::MatrixXd Ax(xvals.size(), degree + 1);
    Eigen::MatrixXd Ay(xvals.size(), degree + 1);

    for (int i = 0; i < xvals.size(); ++i) {
        Ax(i, 0) = 1.0;
        Ay(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); ++j) {
        for (int i = 0; i < degree; i++) {
            Ax(j, i + 1) = Ax(j, i) * xvals(j);
            Ay(j, i + 1) = Ay(j, i) * yvals(j);
        }
    }

    Eigen::VectorXd correctvals(zvals.size() * 2);
    correctvals << zvals, zvals;
    std::cout << "zvals: \n" << correctvals << "\n";
    std::cout << "Ax: \n" << Ax << "\n";
    std::cout << "Ay: \n" << Ay << "\n";
    Eigen::MatrixXd A(xvals.size(), 2 * (degree + 1));
    A << Ax, Ay;
    std::cout << "A: \n" << A << "\n";
    auto Q = A.householderQr();
    std::cout << "Q: \n" << Q.matrixQR() << "\n";
    auto result = Q.solve(zvals);
    std::cout << "result: \n" << result << "\n";
    Eigen::MatrixXd Ax1(xvals.size(), degree + 1);
    Eigen::MatrixXd Ay1(xvals.size(), degree + 1);

    for (int i = 0; i < xvals.size(); ++i) {
        Ax1(i, degree) = 1.0;
        Ay1(i, degree) = 1.0;
    }
    std::cout << "test \n";
    for (int j = 0; j < xvals.size(); ++j) {
        for (int i = degree; i > 0; i--) {
            Ax1(j, i - 1) = Ax1(j, i) * xvals(j);
            Ay1(j, i - 1) = Ay1(j, i) * yvals(j);
        }
    }

    std::cout << "Ax1: \n" << Ax1 << "\n";
    std::cout << "Ay1: \n" << Ay1 << "\n";
    Eigen::MatrixXd A1(xvals.size(), 2 * (degree + 1));
    A1 << Ax1, Ay1;
    std::cout << "A1: \n" << A1 << "\n";
    auto Q1 = A1.householderQr();
    std::cout << "Q1: \n" << Q1.matrixQR() << "\n";

    auto result1 = Q1.solve(zvals);
    std::cout << "result1: \n" << result1 << "\n";


    return result;
}


void path_nav::PathNavNode::test() {
//    int degree = 2;
//
//
//    Eigen::VectorXd waypoints_xs(3);
//    Eigen::VectorXd waypoints_ys(3);
//    Eigen::VectorXd waypoints_zs(3);
//    Eigen::VectorXd waypoints_ts(3);
//    waypoints_ts[0] = 0;
//    waypoints_ts[1] = 1;
//    waypoints_ts[2] = 2;
//
//    waypoints_xs[0] = 0.0;
//    waypoints_ys[0] = 2;
//    waypoints_zs[0] = 0.0;
//
//    waypoints_xs[1] = 12;
//    waypoints_ys[1] = 1;
//    waypoints_zs[1] = 0.695;
//
//    waypoints_xs[2] = 24;
//    waypoints_ys[2] = 3;
//    waypoints_zs[2] = 1.345;
//
//    Eigen::VectorXd K = polyfit3d(waypoints_xs, waypoints_ys, waypoints_zs, degree);
//
//    Eigen::VectorXd K_x = polyfit(waypoints_ts, waypoints_xs, degree);
//    Eigen::VectorXd K_y = polyfit(waypoints_ts, waypoints_ys, degree);
//    Eigen::VectorXd K_z = polyfit(waypoints_ts, waypoints_zs, degree);
//    std::cout << "k_x: \n" << K_x << "\n";
//    std::cout << "k_y: \n" << K_y << "\n";
//    std::cout << "k_z: \n" << K_z << "\n";
//    auto x = K_x[2] * 2 * 2 + K_x[1] * 2 + K_x[0]; // K[3] pxo³ + K[2] pxo² +K[1] pxo + K[0]
//    auto y = K_y[2] * 2 * 2 + K_y[1] * 2 + K_y[0]; // K[3] pxo³ + K[2] pxo² +K[1] pxo + K[0]
//    auto z = K_z[2] * 2 * 2 + K_z[1] * 2 + K_z[0]; // K[3] pxo³ + K[2] pxo² +K[1] pxo + K[0]
//    std::cout << "result t=2: \n" << x << "," << y << "," << z << "\n";

    //            ros::init(argc, argv, "path_nav_node");
//        geometry_msgs::Point startPoint;
//        startPoint.x = 1.0;
//        startPoint.y = 1.0;
//        startPoint.z = 0.0;
//
//        geometry_msgs::Point goalPoint;
//        goalPoint.x = 3.0;
//        goalPoint.y = 3.0;
//        goalPoint.z = 0.0;
//
//        geometry_msgs::Point dronePosition;
//        dronePosition.x = 2.0;
//        dronePosition.y = 2.0;
//        dronePosition.z = 1.0;
//
//        path_nav_math::Line line(startPoint, goalPoint);
//
//        std::cout << "LINE: \n" << line.toString();
//        std::cout << "Drone position:\n" << dronePosition << "\n";
//
//        path_nav_math::Vector3d result = line.findShortestVectorFromPointToLine(dronePosition);
//
//        path_nav::PathNavNode node = path_nav::PathNavNode();
//
//        node.goal = goalPoint;
//        node.start = startPoint;
//        node.line = line;
//        node.line.initialized = true;
//
//        // Rotate 90 degrees around y-axis. (https://quaternions.online/)
//        geometry_msgs::Pose_<std::allocator<void>>::_orientation_type q;
//        q.w = 0.707;
//        q.x = 0.0;
//        q.y = 0.707;
//        q.z = 0.0;
//        std::cout << "Quaternion: \n" << q << "\n";
//        geometry_msgs::Pose dronePose;
//        dronePose.position = dronePosition;
//        dronePose.orientation = q;
//
//        node.navigatorCallback(dronePose);
//
//        // To convert and check result check (http://www.nh.cas.cz/people/lazar/celler/online_tools.php?start_vec=1,1,-2&rot_ax=0,90,0&rot_ang=90)
//        std::cout << "moveVectorWorld:\n" << node.debugMoveVectorWorld.toString();
//        std::cout << "rotated: \n" << node.debugRotated.toString();
//
//        std::cout << "command: \n" << node.latestMoveCommand;
}

