#! /usr/bin/env python

import rospy
from Drone import Drone

# Import msg
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from martin_msg_lib.msg import Segment
from std_msgs.msg import Empty
import math
import time
import tf
import numpy as np
import csv


class pidcontroller:

    # Defining the P,I,D control parameters
    def __init__(self, kp=[0.3, 0.3, 0.3, 3], ki=[0.01, 0.01, 0, 0.000], kd=[0.4, 0.4, 0.4, 0.000], mode="no_turn"):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.mode = mode
        self.drone = Drone()
        self.breaker = True
        self.vel = [0.0, 0.0, 0.0, 0.0]
        self.prev_time = [0.0, 0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0, 0.0]
        self.e_i = [0.0, 0.0, 0.0, 0.0]
        self.speed = 2
        self.repeat_pilot = False
        self.single_point_error = 0
        self.multiple_point_error = 0.2  # 0.1
        self.ang_error = 0.05  # 0.02
        self.length = 1
        self.arrA = []
        self.arrB = []
        self.arrC = []
        self.autopilot = False
        self.goal = PoseStamped()
        self.tfl = tf.TransformListener()

    def segment(self, segment_msgs):
        self.goal = PoseStamped()
        self.goal.header.frame_id = "map"
        self.goal.pose.position.x = segment_msgs.p2.position.x
        self.goal.pose.position.y = segment_msgs.p2.position.y
        self.goal.pose.position.z = segment_msgs.p2.position.z
        self.goal.pose.orientation.x = 0
        self.goal.pose.orientation.y = 0
        self.goal.pose.orientation.z = 0
        self.goal.pose.orientation.w = 1
        self.prev_error = [0.0, 0.0, 0.0, 0.0]

    def pose_autopilot(self, poseStamped):
        if not self.autopilot:
            return

        self.tfl.waitForTransform("map", "camera_link_scaled", poseStamped.header.stamp, rospy.Duration(1.0))
        self.goal.header.stamp = poseStamped.header.stamp

        try:
            goal_pose_body = self.tfl.transformPose("/camera_link_scaled", self.goal)
            self.error_tol = self.single_point_error
            print "goal_body: ", (goal_pose_body.pose.position.x, goal_pose_body.pose.position.y,
                                  goal_pose_body.pose.position.z)  # , target_orient[2] * 180 / math.pi

            if self.mode == 'move+turn':
                debug = 0
                # self.move_n_turn(goal_pose_body.pose.position, target_orient, rot)
            elif self.mode == 'no_turn':
                self.no_turn(goal_pose_body.pose.position, poseStamped.header.stamp.to_sec())
            elif self.mode == 'reach&turn':
                debug = 0
                # self.reach_turn(goal_pose_body.pose.position, target_orient, rot)
            elif self.mode == 'turn&move':
                debug = 0
                # self.turn_move(goal_pose_body.pose.position, target_orient, rot)
            else:
                print 'Invalid or No Autopilot Mode Specified!'

        except (tf.LookupException, tf.ConnectivityException):
            return

    def start_autopilot(self, autopilot_data):
        if not self.autopilot:
            return

        self.breaker = autopilot_data.autopilot_breaker_state
        self.count_loop = 0
        self.reach_pose = False

        listener = tf.TransformListener()
        rate = rospy.Rate(50)
        listener.waitForTransform("map", "/bebop_pose", rospy.Time(), rospy.Duration(1.0))
        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = autopilot_data.newpos[0]
        goal_pose.pose.position.y = autopilot_data.newpos[1]
        goal_pose.pose.position.z = autopilot_data.newpos[2]
        target_orient = autopilot_data.neworientation[:3]
        if autopilot_data.no_of_points > 1:
            self.repeat_pilot = True
            self.error_tol = self.multiple_point_error
            self.new_autopilot_newpos = autopilot_data.newpos[3:]
            self.new_autopilot_neworient = autopilot_data.neworientation[3:]
            self.points_left = len(self.new_autopilot_newpos) / 3
            if autopilot_data.no_of_points > self.length:
                self.list_of_points = autopilot_data.newpos
                self.list_of_orients = autopilot_data.neworientation
                self.length = autopilot_data.no_of_points
        elif autopilot_data.no_of_points == 1:
            self.error_tol = self.single_point_error
            if self.repeat_pilot:
                self.new_autopilot_newpos = self.list_of_points
                self.new_autopilot_neworient = self.list_of_orients
                self.points_left = len(self.list_of_points) / 3
                self.error_tol = self.multiple_point_error
        else:
            print 'Invalid no. of points given!'
        print 'point:', self.length - autopilot_data.no_of_points + 1, '/', self.length
        print (goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z), target_orient[
                                                                                                     2] * 180 / math.pi
        while not self.breaker:
            try:
                # rospy.Subscriber('/autopilot_stop_cmd', AutoPilotCmd, self.checkbreak)
                listener.waitForTransform("odom", "base_link", rospy.Time(), rospy.Duration(4.0))
                (trans, rot) = listener.lookupTransform("odom", "/base_link", rospy.Time())
                self.curr_global_pose = trans
                self.arrA.append(trans[0])
                self.arrB.append(trans[1])
                self.arrC.append(trans[2])
                goal_pose_n = goal_pose
                goal_pose_n.header.stamp = listener.getLatestCommonTime("/base_link", goal_pose_n.header.frame_id)
                pose_in_loc_frame = listener.transformPose("/base_link", goal_pose_n)
                target_point = pose_in_loc_frame.pose.position
                if autopilot_data.autopilot_mode == 'move+turn':
                    self.move_n_turn(target_point, target_orient, rot)
                elif autopilot_data.autopilot_mode == 'no_turn':
                    self.no_turn(target_point)
                elif autopilot_data.autopilot_mode == 'reach&turn':
                    self.reach_turn(target_point, target_orient, rot)
                elif autopilot_data.autopilot_mode == 'turn&move':
                    self.turn_move(target_point, target_orient, rot)
                else:
                    print 'Invalid or No Autopilot Mode Specified!'
                if self.reach_pose:
                    break
            except (tf.LookupException, tf.ConnectivityException):
                continue
            rate.sleep()
        if self.repeat_pilot == True and self.reach_pose == True and self.breaker == False:
            self.nextpoint(self.new_autopilot_newpos, self.new_autopilot_neworient, self.points_left,
                           autopilot_data.autopilot_mode)
        else:
            self.breaker = True
            self.repeat_pilot = False
            self.reach_pose = False
            print 'Navigation Done!'
            # with open('expt_path_crcl_nt.csv','w') as f:
            #     writer = csv.writer(f, delimiter='\t')
            #     writer.writerows(zip(self.arrA,self.arrB,self.arrC))

    def checkbreak(self, val):
        self.breaker = val.autopilot_breaker_state

    def calc_error(self, error, i, pose_time):  # Calculates the error, its `derivative' and `integral'
        curr_time = pose_time
        dt = 0.0
        if self.prev_time[i] != 0.0:
            dt = curr_time - self.prev_time[i]
        de = error - self.prev_error[i]

        e_p = self.kp[i] * error
        self.e_i[i] += error * dt
        self.e_i[i] = np.clip(self.e_i[i], -0.05, 0.05)
        e_d = 0
        if dt > 0:
            e_d = de / dt
        self.prev_time[i] = curr_time
        self.prev_error[i] = error
        return e_p + (self.ki[i] * self.e_i[i]) + (self.kd[i] * e_d)

    def no_turn(self, targ_pos, pose_time):  # Corrects only position
        if self.autopilot:
            errors = (targ_pos.x, targ_pos.y, targ_pos.z)
            print ("errors", errors)
            for i in range(len(errors)):
                self.vel[i] = self.calc_error(errors[i], i, pose_time)
            if max(errors) > self.error_tol or min(errors) < -self.error_tol:
                self.drone.moveLinear(self.speed * self.vel[0], self.speed * self.vel[1], self.speed * self.vel[2])
            else:
                self.reach_pose = True
                print 'reached destination coordinates'
                print 'error:', np.linalg.norm(errors)

    def reach_turn(self, targ_pos, targ_orient, curr_orient):  # Reaches the destination, then corrects orientation
        if self.autopilot:
            yaw = math.atan2(2 * (curr_orient[0] * curr_orient[1] + curr_orient[3] * curr_orient[2]),
                             curr_orient[3] * curr_orient[3] + curr_orient[0] * curr_orient[0] - curr_orient[1] *
                             curr_orient[1] - curr_orient[2] * curr_orient[2])
            errors = (targ_pos.x, targ_pos.y, targ_pos.z, targ_orient[2] - yaw)
            for i in range(len(errors)):
                self.vel[i] = self.calc_error(errors[i], i)
            if max(errors[:3]) > self.error_tol or min(errors[:3]) < -self.error_tol:
                self.drone.moveLinear(self.speed * self.vel[0], self.speed * self.vel[1], self.speed * self.vel[2])
            else:
                if abs(errors[3]) > self.ang_error:
                    self.drone.moveAngular(0, 0, self.speed * self.vel[3])
                    print 'Reached position. Correcting orientation...'
                else:
                    self.reach_pose = True
                    print 'Reached destination pose'

    def turn_move(self, targ_pos, targ_orient, curr_orient):  # Corrects orientation, then position
        if self.autopilot:
            yaw = math.atan2(2 * (curr_orient[0] * curr_orient[1] + curr_orient[3] * curr_orient[2]),
                             curr_orient[3] * curr_orient[3] + curr_orient[0] * curr_orient[0] - curr_orient[1] *
                             curr_orient[1] - curr_orient[2] * curr_orient[2])
            errors = (targ_pos.x, targ_pos.y, targ_pos.z, targ_orient[2] - yaw)
            for i in range(len(errors)):
                self.vel[i] = self.calc_error(errors[i], i)
            if abs(errors[3]) > self.ang_error:
                self.drone.moveAngular(0, 0, self.speed * self.vel[3])
            else:
                if max(errors[:3]) > self.error_tol or min(errors[:3]) < -self.error_tol:
                    self.drone.moveLinear(self.speed * self.vel[0], self.speed * self.vel[1], self.speed * self.vel[2])
                    print 'Corrected Orientation. Moving to destination...'
                else:
                    self.reach_pose = True
                    print 'Reached Destination pose'

    def move_n_turn(self, targ_pos, targ_orient, curr_orient):  # Corrects both position and orientation simultaneously
        if self.autopilot:
            yaw = math.atan2(2 * (curr_orient[0] * curr_orient[1] + curr_orient[3] * curr_orient[2]),
                             curr_orient[3] * curr_orient[3] + curr_orient[0] * curr_orient[0] - curr_orient[1] *
                             curr_orient[1] - curr_orient[2] * curr_orient[2])
            errors = (targ_pos.x, targ_pos.y, targ_pos.z, targ_orient[2] - yaw)
            for i in range(len(errors)):
                self.vel[i] = self.calc_error(errors[i], i)
            if max(errors) > self.error_tol or min(errors) < -self.error_tol:
                if self.count_loop % 2 == 0 or abs(errors[3]) < self.ang_error:
                    self.drone.moveLinear(self.speed * self.vel[0], self.speed * self.vel[1], self.speed * self.vel[2])
                else:
                    self.drone.moveAngular(0, 0, self.speed * self.vel[3])
                self.count_loop += 1
            else:
                self.reach_pose = True
                print 'reached destination pose!'
                print 'errors:- Linear: %f; Angular: %f' % (
                    np.linalg.norm((errors[0], errors[1], errors[2])), errors[3])

    def nextpoint(self, newpos, neworient, points_left, mode):
        # loop_data = AutoPilotCmd(False, newpos, neworient, points_left, mode)
        # self.start_autopilot(loop_data)
        print 'going to next point'

    def toggleAutopilot(self, empty):
        if self.autopilot:
            self.autopilot = False
        else:
            self.autopilot = True


def listener():
    rospy.init_node('pid_navigator')
    pid = pidcontroller(mode="no_turn")
    rospy.Subscriber('/pather_toggled', Empty, pid.toggleAutopilot)
    rospy.Subscriber('/orb_slam2/camera_pose', PoseStamped, pid.pose_autopilot)
    rospy.Subscriber('/path_segmenter/path_segment', Segment, pid.segment)

    print '>>Started PID Controller for Spinny...\n\tSinglePose Error tol:', pid.single_point_error, '\n\tMultiPose Error tol:', pid.multiple_point_error, '\n\tAngular Error:', pid.ang_error
    print '\tAutoPilot Speed:', pid.speed
    print 'PID terms:', pid.kp, pid.kd, pid.ki
    rospy.spin()


if __name__ == '__main__':
    listener()
