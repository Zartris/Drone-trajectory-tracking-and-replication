//
// Created by zartris on 2/21/19.
//
#ifndef PATH_NAVIGATOR_VECTOR3D_H
#define PATH_NAVIGATOR_VECTOR3D_H

#include <geometry_msgs/Point.h>
#include <eigen3/Eigen/Dense>
#include <utility>
#include <mutex>

namespace path_nav_math {
    class Vector3d {
    public:
        Vector3d(double x, double y, double z);

        Vector3d(Eigen::Vector3d v);

        // Help for move and copy https://stackoverflow.com/questions/24272641/how-to-use-a-stdmutex-in-a-class-context
        Vector3d(const Vector3d &other);  // copy constructor

        Vector3d(Vector3d &&other) noexcept; // move constructor

        // Move assignment
        Vector3d &operator=(Vector3d &&other) noexcept {
            std::lock(setValue_mutex, other.setValue_mutex);
            std::lock_guard<std::mutex> self_lock(setValue_mutex, std::adopt_lock);
            std::lock_guard<std::mutex> other_lock(other.setValue_mutex, std::adopt_lock);
            if (this != &other) {           // If the object isn't being called on itself
                this->x = std::move(other.x);          // "Move" other's data into the current object
                this->y = std::move(other.y);          // "Move" other's data into the current object
                this->z = std::move(other.z);          // "Move" other's data into the current object
            }
            return *this;
        }

        // Copy assignment
        Vector3d &operator=(const Vector3d &other) {
            std::lock(setValue_mutex, other.setValue_mutex);
            std::lock_guard<std::mutex> self_lock(setValue_mutex, std::adopt_lock);
            std::lock_guard<std::mutex> other_lock(other.setValue_mutex, std::adopt_lock);
            if (this != &other) {           // If the object isn't being called on itself
                this->x = other.x;          // "Move" other's data into the current object
                this->y = other.y;          // "Move" other's data into the current object
                this->z = other.z;          // "Move" other's data into the current object
            }
            return *this;
        }


        virtual ~Vector3d() = default;

        // getter and setters:
        void setVector(double x, double y, double z);

        Eigen::Vector3d getEigenVector();

        path_nav_math::Vector3d sub(path_nav_math::Vector3d v);

        path_nav_math::Vector3d add(path_nav_math::Vector3d v);

        double dot(path_nav_math::Vector3d v);

        Eigen::Vector3d cross(path_nav_math::Vector3d v);

        double getMax();

        void normalize();

        void setX(double d);

        void setY(double d);

        void setZ(double d);

        double getX();

        double getY();

        double getZ();

        std::string toString();

        mutable std::mutex setValue_mutex;

//    private:
        double x = 0;
        double y = 0;
        double z = 0;


    };
}
#endif //PATH_NAVIGATOR_VECTOR3D_H
