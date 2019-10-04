//
// Created by zartris on 2/21/19.
//

#ifndef PATH_NAVIGATOR_LINE_H
#define PATH_NAVIGATOR_LINE_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <martin_msg_lib/Segment.h>
#include <path_navigator/vector3d.h>
#include <math.h>
#include <mutex>


namespace path_nav_math {
    class Line {
    public:
        Line();

        Line(geometry_msgs::Point start, geometry_msgs::Point goal);

        // Help for move and copy https://stackoverflow.com/questions/24272641/how-to-use-a-stdmutex-in-a-class-context
        Line(const Line &other);  // copy constructor

        Line(Line &&other) noexcept; // move constructor

        // Move assignment
        Line &operator=(Line &&other) noexcept {
            std::lock(setValue_mutex, other.setValue_mutex);
            std::lock_guard<std::mutex> self_lock(setValue_mutex, std::adopt_lock);
            std::lock_guard<std::mutex> other_lock(other.setValue_mutex, std::adopt_lock);
            if (this != &other) {           // If the object isn't being called on itself
                this->start = std::move(other.start);          // "Move" other's data into the current object
                this->goal = std::move(other.goal);          // "Move" other's data into the current object
                this->denominator = std::move(
                        other.denominator);          // "Move" other's data into the current object
                other.start = Vector3d(0, 0, 0);
                other.goal = Vector3d(0, 0, 0);
                other.denominator = 0;
            }
            return *this;
        }

        // Copy assignment
        Line &operator=(const Line &other) {
            std::lock(setValue_mutex, other.setValue_mutex);
            std::lock_guard<std::mutex> self_lock(setValue_mutex, std::adopt_lock);
            std::lock_guard<std::mutex> other_lock(other.setValue_mutex, std::adopt_lock);
            if (this != &other) {           // If the object isn't being called on itself
                this->start = other.start;          // "Move" other's data into the current object
                this->goal = other.goal;          // "Move" other's data into the current object
                this->denominator = other.denominator;          // "Move" other's data into the current object
            }
            return *this;
        }


        virtual ~Line() = default;


        // Find orthogonal vector to line.
        path_nav_math::Vector3d findShortestVectorFromPointToLine(geometry_msgs::Point point);

        // getter and setters:
        path_nav_math::Vector3d getStart();

        path_nav_math::Vector3d getGoal();

        void overrideValues(geometry_msgs::Point start, geometry_msgs::Point goal);


        void setValues(geometry_msgs::Point start, geometry_msgs::Point goal);

        std::string toString();

        bool initialized = false;
        double p_scale = 0.0;
        double t_cur = 0;

        double getTCurCapped();
        double getTCurUncapped();
        // debug:
        path_nav_math::Vector3d debug_pol = path_nav_math::Vector3d(0, 0, 0);

    private:
        // init
        path_nav_math::Vector3d start = path_nav_math::Vector3d(0, 0, 0);
        path_nav_math::Vector3d goal = path_nav_math::Vector3d(0, 0, 0);
        double denominator = 0.0;

        void setGoal(geometry_msgs::Point point);

        void setStart(geometry_msgs::Point point);

        void setDenominator();

        mutable std::mutex setValue_mutex;

    };
}
#endif //PATH_NAVIGATOR_LINE_H
