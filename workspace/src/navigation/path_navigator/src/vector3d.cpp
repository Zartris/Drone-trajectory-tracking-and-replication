

//
// Created by zartris on 2/21/19.
//

#include <path_navigator/vector3d.h>

path_nav_math::Vector3d::Vector3d(const double x, const double y, const double z) {
    setVector(x, y, z);
}

path_nav_math::Vector3d::Vector3d(Eigen::Vector3d v) {
    setVector(v.coeff(0), v.coeff(1), v.coeff(2));
}

// Move initialization
path_nav_math::Vector3d::Vector3d(path_nav_math::Vector3d &&other) noexcept {
    std::lock_guard<std::mutex> lock(other.setValue_mutex);
    x = other.x;
    y = other.y;
    z = other.z;
    other.x = 0;
    other.y = 0;
    other.z = 0;
}

// Copy initialization
path_nav_math::Vector3d::Vector3d(const path_nav_math::Vector3d &other) {
    std::lock_guard<std::mutex> lock(other.setValue_mutex);
    x = other.x;
    y = other.y;
    z = other.z;
}


void path_nav_math::Vector3d::setVector(double x, double y, double z) {
    std::lock_guard<std::mutex> lock(setValue_mutex);
    setX(x);
    setY(y);
    setZ(z);
}

double path_nav_math::Vector3d::getMax(){
    if(std::fabs(x)>=std::fabs(y)&& std::fabs(x)>=std::fabs(z)) {
        return std::fabs(x);
    } else if(std::fabs(y)>=std::fabs(x)&& std::fabs(y)>=std::fabs(z)) {
        return std::fabs(y);
    }
    return std::fabs(z);
}

Eigen::Vector3d path_nav_math::Vector3d::getEigenVector() {
    return {x, y, z};
}

path_nav_math::Vector3d path_nav_math::Vector3d::sub(path_nav_math::Vector3d v) {
    double x_val = x - v.getX();
    double y_val = y - v.getY();
    double z_val = z - v.getZ();

    return path_nav_math::Vector3d(x_val, y_val, z_val);
}

path_nav_math::Vector3d path_nav_math::Vector3d::add(path_nav_math::Vector3d v) {
    double x_val = x + v.getX();
    double y_val = y + v.getY();
    double z_val = z + v.getZ();

    return path_nav_math::Vector3d(x_val, y_val, z_val);
}


double path_nav_math::Vector3d::dot(path_nav_math::Vector3d v) {
    return getEigenVector().dot(v.getEigenVector());
}


Eigen::Vector3d path_nav_math::Vector3d::cross(path_nav_math::Vector3d v) {
    return getEigenVector().cross(v.getEigenVector());
}

void path_nav_math::Vector3d::normalize() {
    Eigen::Vector3d v = getEigenVector();
    v.normalize();
    setVector(v.coeff(0), v.coeff(1), v.coeff(2));
}


void path_nav_math::Vector3d::setX(double d) {
    x = d;
}

void path_nav_math::Vector3d::setY(double d) {
    y = d;
}

void path_nav_math::Vector3d::setZ(double d) {
    z = d;
}

double path_nav_math::Vector3d::getX() {
    return x;
}

double path_nav_math::Vector3d::getY() {
    return y;
}

double path_nav_math::Vector3d::getZ() {
    return z;
}

std::string path_nav_math::Vector3d::toString() {
    std::string s;
    s.append("(");
    s.append(std::to_string(getX()));
    s.append(", ");
    s.append(std::to_string(getY()));
    s.append(", ");
    s.append(std::to_string(getZ()));
    s.append(")\n \n");
    return s;
}

















