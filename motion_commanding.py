import rospy
from gazebo_msgs.srv import GetModelState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf
from geometry_msgs.msg import Point
import sys, select, os
import tty, termios
import numpy as np


# much of the live terminal stuff was modified from the turtlebot3_teleop_key package
# https://github.com/ROBOTIS-GIT/turtlebot3/blob/master/turtlebot3_teleop/nodes/turtlebot3_teleop_key

class MotionCommander:
    def __init__(self, x, y):
        rospy.init_node('mover', anonymous=False)
        rospy.Subscriber("odom", Odometry, self._update_state)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.tolerance = 0.01
        self.angle_tolerance = 0.01
        self.desired_location = np.array([x, y])
        self.max_vel = 0.2
        self.max_ang_vel = 0.5
        self.k_p_angle = 0.1
        self.k_p_vel = 0.5

    def _update_state(self, response):
        rospy.wait_for_service("gazebo/get_model_state")
        gms = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
        self.state = gms('turtlebot3_waffle_pi', None)
        self.location = np.array([self.state.pose.position.x, self.state.pose.position.y])
        quaternion = self.state.pose.orientation
        explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        self.angle = tf.transformations.euler_from_quaternion(explicit_quat)[2]
        # roll = angle[0], pitch = angle[1], yaw = angle[2]
        self.command_movement()
        print("Location: x = " + str(self.location[0]) + ", y = " + str(self.location[1]))
        print("Angle = " + str(self.angle))

    def command_movement(self):
        twist = Twist()
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0

        if distance(self.location, self.desired_location) < self.tolerance:
            self.stop()
            return

        d_loc = self.desired_location - self.location
        d = np.linalg.norm(d_loc)
        desired_angle = np.arctan2(d_loc[1], d_loc[0])
        d_angle = (self.angle - desired_angle) % (2 * np.pi)
        if d_angle > np.pi:
            d_angle = -(2 * np.pi - d_angle)

        if d_angle < self.angle_tolerance:
            ang_vel = 0
            lin_vel = self.k_p_vel * d
        else:
            ang_vel = self.k_p_angle * d_angle
            lin_vel = 0
        twist.linear.x = constrain(lin_vel, 0, self.max_vel)
        twist.angular.z = constrain(ang_vel, -self.max_ang_vel, self.max_ang_vel)
        self.pub.publish(twist)

    def move_to_position(self, x, y):
        self.desired_location[0] = x
        self.desired_location[1] = y

    def stop(self):
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub.publish(twist)


def distance(p1, p2):
    return np.linalg.norm(p2 - p1)


def constrain(val, low, high):
    if val < low:
        return low

    if val > high:
        return high

    return val


if __name__ == '__main__':
    goals = np.array([[3.25, 1], [2.7, 2.4], [3.8, 2.4], [3.8, 3.45], [2.7, 3.45], [3.25, 5.1], [5, 3], [1.5, 3]])
    goal_matrix = np.load('goal_matrix.npy', allow_pickle=True)
    mc = MotionCommander(goals[0][0], goals[0][1])
    while True:
        continue
