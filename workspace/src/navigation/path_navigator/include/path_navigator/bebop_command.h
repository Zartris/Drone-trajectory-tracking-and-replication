//
// Created by zartris on 2/26/19.
//

#ifndef PATH_NAVIGATOR_BEBOP_COMMAND_H
#define PATH_NAVIGATOR_BEBOP_COMMAND_H

#include <mutex>
#include <geometry_msgs/Twist.h>

namespace bebop_command {
    class BebopCommand {
    public:
        BebopCommand() = default;

        virtual ~BebopCommand() = default;

        void setCommand(geometry_msgs::Twist c);
        void setFrameTime(ros::Time time);



        geometry_msgs::Twist getCommand();

        bool isNewCommand;

        ros::Time frame_time;



    private:
        // Find orthogonal vector to line.
        geometry_msgs::Twist command;
        std::mutex setValue_mutex;


    };
}
#endif //PATH_NAVIGATOR_BEBOP_COMMAND_H
