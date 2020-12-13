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
        self.coordinate_offset = np.array([3.3, 3.05])
        self.tolerance = 0.01
        self.angle_tolerance = 0.2
        self.max_vel = 0.26
        self.max_ang_vel = 1.82
        self.k_p_angle = -2
        self.k_p_vel = 1
        self.goal_reached = False
        self.desired_location = np.array([x, y])
        self.move_to_position(x, y)

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

    def command_movement(self):
        if self.goal_reached:
            return
        twist = Twist()
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0

        if distance(self.location, self.desired_location) < self.tolerance:
            self.goal_reached = True
            print("Waypoint reached.")
            return

        d_loc = self.desired_location - self.location
        d = np.linalg.norm(d_loc)
        desired_angle = np.arctan2(d_loc[1], d_loc[0])
        d_angle = (self.angle - desired_angle) % (2 * np.pi)
        if d_angle > np.pi:
            d_angle = -(2 * np.pi - d_angle)

        if np.abs(d_angle) < np.abs(self.angle_tolerance):
            ang_vel = self.k_p_angle * d_angle
            if ang_vel > 0:
                ang_vel = ang_vel + 0.05
            else:
                ang_vel = ang_vel - 0.05
            lin_vel = self.k_p_vel * d
            lin_vel = lin_vel + 0.1
        else:
            ang_vel = self.k_p_angle * d_angle
            if ang_vel > 0:
                ang_vel = ang_vel + 0.1
            else:
                ang_vel = ang_vel - 0.1
            lin_vel = 0
        twist.linear.x = constrain(lin_vel, 0, self.max_vel)
        twist.angular.z = constrain(ang_vel, -self.max_ang_vel, self.max_ang_vel)
        self.pub.publish(twist)

    def move_to_position(self, x, y):
        self.goal_reached = False
        self.desired_location[0] = y - self.coordinate_offset[1]
        self.desired_location[1] = -(x - self.coordinate_offset[0])
        print("Moving to (" + str(self.desired_location[0]) + ", " + str(self.desired_location[1]) + ")")

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


def traverse_path(cur_goal, next_goal, goal_matrix):
    if cur_goal == next_goal:
        return True
    path = goal_matrix[next_goal, cur_goal]
    for point in path:
        mc.move_to_position(point[0], point[1])
        while not mc.goal_reached:
            # wait until we reach the waypoint
            pass
    mc.stop()
    return True


if __name__ == '__main__':
    goals = np.array([[3.25, 1], [2.7, 2.4], [3.8, 2.4], [3.8, 3.45], [2.7, 3.45], [3.25, 5.1], [5, 3], [1.5, 3]])
    goal_matrix = np.load('goal_matrix.npy', allow_pickle=True)
    mc = MotionCommander(goals[0][0], goals[0][1])
    cur_goal = 0
    while True:
        if mc.goal_reached:
            mc.stop()
            print
            goal_num = raw_input('Enter goal number or "end" to end: ')
            if goal_num == 'end':
                print("Returning to start, then exiting.")
                traverse_path(cur_goal, 0, goal_matrix)
                mc.stop()
                print("Goodbye!")
                break
            else:
                try:
                    goal_num = int(goal_num)
                except ValueError as e:
                    print("Goal must be numeric or 'end'!")
                    goal_num = cur_goal

            if 0 <= goal_num < len(goals):
                print("Traversing to goal " + str(goal_num))
                success = traverse_path(cur_goal, goal_num, goal_matrix)
                if success:
                    cur_goal = goal_num
            else:
                print("Enter a number 0-" + str(len(goals) - 1))
