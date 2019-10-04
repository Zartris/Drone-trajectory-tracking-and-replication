//
// Created by zartris on 2/21/19.
//
#include <path_navigator/line.h>
//
//int main(int argc, char **argv) {
//    // TESTING:
//    std::cout << "#######TESTING Vector3d CLASS###### \n \n";
//    path_nav_math::Vector3d v1(3,3,3);
//    path_nav_math::Vector3d v2(1,1,1);
//    path_nav_math::Vector3d result = v1.add(v2);
//    std::cout << "add v1 v2: " << result.toString() << " \n \n";
//
//    result = v1.sub(v2);
//    std::cout << "sub v1 v2: " << result.toString() <<" \n \n";
//
//    v1.dot(v2);
//    std::cout << "dot v1 v2: (" << v1.dot(v2) << ") " << v1.toString() << ", " << v2.toString() <<"\n \n";
//
//    v1.cross(v2);
//    std::cout << "dot v1 v2: (" << v1.cross(v2) << ") " << v1.toString() << ", " << v2.toString() <<"\n \n";
//
//    std::cout << "#######TESTING line CLASS###### \n \n";
//    geometry_msgs::Point startPoint;
//    startPoint.x = 1.0;
//    startPoint.y = 1.0;
//    startPoint.z = 0.0;
//
//    geometry_msgs::Point goalPoint;
//    goalPoint.x = 3.0;
//    goalPoint.y = 3.0;
//    goalPoint.z = 0.0;
//
//    geometry_msgs::Point dronePos;
//    dronePos.x = 2.0;
//    dronePos.y = 2.0;
//    dronePos.z = 1.0;
//
//    path_nav_math::Line line(startPoint, goalPoint);
//
//    std::cout << "LINE: \n" << line.toString();
//    std::cout << "Drone position:\n" << dronePos << "\n";
//
//    result = line.findShortestVectorFromPointToLine(dronePos);
//    std::cout << "point to line vector: " << result.toString() << " \n \n";
//
//    std::cout << "point on line: " << line.debug_pol.toString() << " \n \n";
//
//    std::cout <<"Testing override\n";
//    std::cout << "LINE: \n" << line.toString();
//    goalPoint.x = 200.0;
//    goalPoint.y = 300.0;
//    goalPoint.z = 400.0;
//    line.overrideValues(startPoint, goalPoint);
//    startPoint.x = 100000000.0;
//    std::cout << "New line: \n" << line.toString();
//
//
//    return 0;
//}


/**
 * CONSTRUCTORS AND OPERATORS::::::::::
 */
path_nav_math::Line::Line() {
    geometry_msgs::Point startPoint;
    startPoint.x = 0.0;
    startPoint.y = 0.0;
    startPoint.z = 0.0;

    geometry_msgs::Point goalPoint;
    goalPoint.x = 0.0;
    goalPoint.y = 0.0;
    goalPoint.z = 0.0;

    // Lock
    std::lock_guard<std::mutex> lock(setValue_mutex);
    setValues(startPoint, goalPoint);
    initialized = false;
}

path_nav_math::Line::Line(geometry_msgs::Point startPoint, geometry_msgs::Point goalPoint) {
    std::lock_guard<std::mutex> lock(setValue_mutex);
    setValues(startPoint, goalPoint);
    initialized = true;
}

// copy constructor
path_nav_math::Line::Line(const path_nav_math::Line &other) {
    std::lock_guard<std::mutex> lock(other.setValue_mutex);
    start = other.start;
    goal = other.goal;
    denominator = other.denominator;
}


// move constructor
path_nav_math::Line::Line(path_nav_math::Line &&other) noexcept {
    std::lock_guard<std::mutex> lock(other.setValue_mutex);
    start = std::move(other.start);
    goal = std::move(other.goal);
    denominator = std::move(other.denominator);
    other.start = Vector3d(0, 0, 0);
    other.goal = Vector3d(0, 0, 0);
    other.denominator = 0;
}


/**
 * FUNCTIONS:::::::::
 */
path_nav_math::Vector3d path_nav_math::Line::findShortestVectorFromPointToLine(geometry_msgs::Point point) {
    // based on : http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
    path_nav_math::Vector3d p(point.x, point.y, point.z);
    double numerator = start.sub(p).dot(goal.sub(start));
    double t;
    if (denominator == 0) {
        t = -(numerator / 0.0000000001);
    } else {
        t = -(numerator / denominator);
    }

    t_cur = t;
    if(t<0) {
        t=0;
    } else if(t>1) {
        t=1;
    }
    p_scale = 1 - t;
    // find point on line:
    path_nav_math::Vector3d pointOnLine(start.getX() + (goal.getX() - start.getX()) * t,
                                        start.getY() + (goal.getY() - start.getY()) * t,
                                        start.getZ() + (goal.getZ() - start.getZ()) * t);

    return pointOnLine;
//    debug_pol = pointOnLine;
//    // Finding vector from point to pointOnLine = (x2-x1, y2-y1, z2-z1)
//    return path_nav_math::Vector3d(pointOnLine.getX() - point.x,
//                                   pointOnLine.getY() - point.y,
//                                   pointOnLine.getZ() - point.z);
}

void path_nav_math::Line::setStart(geometry_msgs::Point point) {
    start = Vector3d(point.x, point.y, point.z);
}

void path_nav_math::Line::setGoal(geometry_msgs::Point point) {
    goal = Vector3d(point.x, point.y, point.z);
}

path_nav_math::Vector3d path_nav_math::Line::getStart() {
    return start;
}

path_nav_math::Vector3d path_nav_math::Line::getGoal() {
    return goal;
}

void path_nav_math::Line::setDenominator() {
    denominator = pow(sqrt(pow(goal.getX() - start.getX(), 2) +
                           pow(goal.getY() - start.getY(), 2) +
                           pow(goal.getZ() - start.getZ(), 2)), 2);
}


void path_nav_math::Line::overrideValues(geometry_msgs::Point startPoint, geometry_msgs::Point goalPoint) {
    // Lock
//    std::lock_guard<std::mutex> lock(setValue_mutex);
    setValues(startPoint, goalPoint);
    initialized = true;
}

std::string path_nav_math::Line::toString() {
    std::string s;
    s.append("   goal: ");
    s.append(goal.toString());
    s.append("\n");

    s.append("   start: ");
    s.append(start.toString());
    s.append("\n");

    s.append("   denominator: ");
    s.append(std::to_string(denominator));
    s.append("\n");
    return s;
}

void path_nav_math::Line::setValues(geometry_msgs::Point startPoint, geometry_msgs::Point goalPoint) {
//    std::lock_guard<std::mutex> lock(setValue_mutex);
    setStart(startPoint);
    setGoal(goalPoint);
    setDenominator();
}

double path_nav_math::Line::getTCurCapped() {
    if(t_cur<0) {
        return 0;
    } else if(t_cur>1) {
        return 1;
    }
    return t_cur;
}

double path_nav_math::Line::getTCurUncapped() {
    return t_cur;
}





