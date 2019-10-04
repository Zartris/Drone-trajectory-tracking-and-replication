import rospy
from geometry_msgs.msg import Twist, TwistStamped, Vector3
from std_msgs.msg import Empty
import numpy as np


class Drone:

    def __init__(self):
        # publish commands (send to quadrotor)
        self.pub_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
        self.pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=10)
        self.pub_reset = rospy.Publisher('/bebop/reset', Empty, queue_size=10)
        self.pub_velocity = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
        self.pub_twist_debug = rospy.Publisher('/bebop/debugTwist', TwistStamped, queue_size=10)
        self.hasTakenOff = False

    def takeOff(self):
        self.pub_takeoff.publish(Empty())
        self.hasTakenOff = True

    def land(self):
        self.pub_land.publish(Empty())
        self.hasTakenOff = False

    def reset(self):
        self.pub_reset.publish(Empty())

    def moveLinear(self, vx, vy, vz):
        speed = 0.1
        if np.fabs(vx) > speed or np.fabs(vy) > speed or np.fabs(vz) > speed:
            maxvel = 0
            if np.fabs(vx) >= np.fabs(vy) and np.fabs(vx) >= np.fabs(vz):
                maxvel = np.fabs(vx)
            elif np.fabs(vy) >= np.fabs(vx) and np.fabs(vy) >= np.fabs(vz):
                maxvel = np.fabs(vy)
            else:
                maxvel = np.fabs(vz)
            vx = (vx * speed) / maxvel
            vy = (vy * speed) / maxvel
            vz = (vz * speed) / maxvel

        twist = Twist(Vector3(vx, vy, vz), Vector3(0, 0, 0))

        self.pub_velocity.publish(twist)
        twistStamped = TwistStamped()
        twistStamped.twist = twist
        twistStamped.header.frame_id = "camera_link_scaled"
        twistStamped.header.stamp = rospy.Time.now()
        self.pub_twist_debug.publish(twistStamped)

    def moveAngular(self, wx, wy, wz):
        self.pub_velocity.publish(Twist(Vector3(0, 0, 0), Vector3(wx, wy, wz)))

    def hover(self):
        self.moveLinear(0, 0, 0)
