#!/usr/bin/env python
import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf


class AD(enumerate):
    pX = 0  # position X
    pY = 1  # position Y
    qX = 2  # quaternion X
    qY = 3  # quaternion Y
    qZ = 4  # quaternion Z
    qW = 5  # quaternion W


class CalculateGoalParamHandler:
    def __init__(self):
        """
        Class for returning the corresponding metric class with the given parameter.
        """
        pass

    def parse_parameter(self, testblock_name, params):
        """
        Method that returns the metric method with the given parameter.
        :param params: Parameter
        """
        metrics = []
        if type(params) is not list:
            rospy.logerr("metric config not a list")
            return False

        for metric in params:
            # check for optional parameters
            try:
                groundtruth = metric["groundtruth"]
                groundtruth_epsilon = metric["groundtruth_epsilon"]
            except (TypeError, KeyError):
                rospy.logwarn(
                    "No groundtruth parameters given, skipping groundtruth evaluation for metric 'jerk' in testblock '%s'",
                    testblock_name)
                groundtruth = None
                groundtruth_epsilon = None
            metrics.append(CalculateGoal(metric["topic"], groundtruth, groundtruth_epsilon))
        return metrics


class CalculateGoal:
    def __init__(self, groundtruth, groundtruth_epsilon, topic='/base_pose_ground_truth'):
        self.AD = AD()
        self.active = False
        self.finished = False
        self.positiontopic = topic
        self.goaltopic = '/move_base_simple/goal'
        self.groundtruth = groundtruth
        self.groundtruth_epsilon = groundtruth_epsilon
        self.targetgoal = None
        self.softgoal = None
        # create array for further use
        self.A_listener_position = np.ones([0, 8], dtype=np.double)
        self.A_listener_goal = np.ones([0, 8], dtype=np.double)

        rospy.Subscriber(self.positiontopic, Odometry, self.callback, queue_size=1)
        rospy.Subscriber(self.goaltopic, PoseStamped, self.callback_goal, queue_size=1)

    def callback(self, msg):
        if self.active:
            data_list = [float(msg.pose.pose.position.x),
                         float(msg.pose.pose.position.y),
                         float(msg.pose.pose.orientation.x),
                         float(msg.pose.pose.orientation.y),
                         float(msg.pose.pose.orientation.z),
                         float(msg.pose.pose.orientation.w)]

            # append data to array
            self.A_listener_position = np.append(self.A_listener_position,
                                                 [[data_list[0], data_list[1], data_list[2], data_list[3], data_list[4],
                                                   data_list[5]]], axis=0)

    def callback_goal(self, msg):
        if self.active:
            data_list = [float(msg.pose.position.x),
                         float(msg.pose.position.y),
                         float(msg.pose.orientation.x),
                         float(msg.pose.orientation.y),
                         float(msg.pose.orientation.z),
                         float(msg.pose.orientation.w)]

            # append data to array
            self.A_listener_goal = np.append(self.A_listener_goal,
                                             [[data_list[0], data_list[1], data_list[2], data_list[3], data_list[4],
                                               data_list[5]]], axis=0)

    def start(self, timestamp):
        self.active = True
        self.start_time = timestamp
        rospy.loginfo('\033[91m' + '----goal.py----' + '\033[0m')

    def stop(self, timestamp):
        self.active = False
        self.stop_time = timestamp
        self.finished = True

    def pause(self, timestamp):
        # TODO: Implement pause time calculation
        pass

    def purge(self, timestamp):
        # TODO: Implement purge as soon as pause is implemented
        pass

    def quaternion2euler(self, quaternion):
        '''
        returns euler angles of given quaternions
        :param quaternion: list of quaternions [x, y, z, w]
        :return: euler angles
        '''
        euler = tf.transformations.euler_from_quaternions(quaternion)
        return euler

    def getDistance(self):
        delta_x = (self.A_listener_position[-1, self.AD.pX] - self.A_listener_goal[-1, self.AD.pX])
        delta_y = (self.A_listener_position[-1, self.AD.pY] - self.A_listener_goal[-1, self.AD.pY])
        distance = math.sqrt(delta_x ** 2 + delta_y ** 2)
        # return delta distance in [m]
        return distance

    def getAngle(self):
        quaternion_position = [self.A_listener_position[-1, self.AD.qX],
                               self.A_listener_position[-1, self.AD.qY],
                               self.A_listener_position[-1, self.AD.qZ],
                               self.A_listener_position[-1, self.AD.qW]]
        quaternion_goal = [self.A_listener_goal[-1, self.AD.qX],
                           self.A_listener_goal[-1, self.AD.qY],
                           self.A_listener_goal[-1, self.AD.qZ],
                           self.A_listener_goal[-1, self.AD.qW]]
        yaw_position = self.quaternion2euler(quaternion=quaternion_position)
        yaw_goal = self.quaternion2euler(quaternion=quaternion_goal)
        # return absolut value for delta yaw in [radian]
        return math.fabs(yaw_goal - yaw_position)

    def get_result(self):
        groundtruth_result = None
        details = {'topic': self.positiontopic}

        # TODO: implement goal
        # actual calculating
        if self.finished:
            if self.groundtruth != None and self.groundtruth_epsilon != None:
                if self.getDistance() <= self.groundtruth and self.getAngle() <= self.groundtruth_epsilon:
                    data = self.getDistance()
                    groundtruth_result = True
                else:
                    groundtruth_result = False
            return 'goal', data, groundtruth_result, self.groundtruth, self.groundtruth_epsilon, details
        else:
            return False
