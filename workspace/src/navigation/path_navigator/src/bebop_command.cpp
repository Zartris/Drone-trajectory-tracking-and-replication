//
// Created by zartris on 2/26/19.
//

#include <path_navigator/bebop_command.h>
void bebop_command::BebopCommand::setCommand(geometry_msgs::Twist c) {
    std::lock_guard<std::mutex> lock(setValue_mutex);
    isNewCommand = true;
    command = c;
}

geometry_msgs::Twist bebop_command::BebopCommand::getCommand() {
    std::lock_guard<std::mutex> lock(setValue_mutex);
    isNewCommand = false;
    return command;
}

void bebop_command::BebopCommand::setFrameTime(ros::Time time) {
    frame_time = time;
}
