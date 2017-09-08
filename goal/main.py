#!/usr/bin/python

"""
Created on Sep 7, 2017

@author: flg-ma
@attention: Jerk Metric
@contact: marcel.albus@ipa.fraunhofer.de (Marcel Albus)
@version: 1.1.0
"""
import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf


# colors in terminal prints
class TerminalColors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


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
        '''
        init
        :param topic: topic on which position is published '/base_pose_ground_truth'
        :param groundtruth: max allowed distance in [m]
        :param groundtruth_epsilon: max allowed delta angle in [radian]
        '''
        self.active = False
        self.finished = False
        self.positiontopic = topic
        self.goaltopic = '/move_base_simple/goal'
        self.groundtruth = groundtruth
        self.groundtruth_epsilon = groundtruth_epsilon
        self.targetgoal = None
        self.softgoal = None

        # create array for further use
        self.A_listener_position = np.ones([0, 6], dtype=np.double)
        self.A_listener_goal = np.ones([0, 6], dtype=np.double)

        # subscribe to topics in need
        rospy.Subscriber(self.positiontopic, Odometry, self.callback_position, queue_size=1)
        rospy.Subscriber(self.goaltopic, PoseStamped, self.callback_goal, queue_size=1)
        rospy.init_node('goal_metrics_listener', anonymous=True)

        # only for colourful terminal visualization
        self.tc = TerminalColors()

    def callback_position(self, msg):
        '''
        callback for position topic
        :param msg: msg from where the data is collected
        :return: --
        '''
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
        '''
        callback for goal topic
        :param msg: msg from where the data is collected
        :return: --
        '''
        if self.active:
            data_list = [float(msg.pose.position.x),
                         float(msg.pose.position.y),
                         float(msg.pose.orientation.x),
                         float(msg.pose.orientation.y),
                         float(msg.pose.orientation.z),
                         float(msg.pose.orientation.w)]

            # print goal to console once (goal message is only published once)
            print self.tc.BOLD + '=' * 17
            print 'Goal X: {:.3f} [m]'.format(data_list[0])
            print 'Goal Y: {:.3f} [m]'.format(data_list[1])
            print '=' * 17 + self.tc.ENDC

            # append data to array
            self.A_listener_goal = np.append(self.A_listener_goal,
                                             [[data_list[0], data_list[1], data_list[2], data_list[3], data_list[4],
                                               data_list[5]]], axis=0)

    def start(self, timestamp):
        print self.tc.OKBLUE + '=' * 20
        print 'Goal Metrics started'
        print '=' * 20 + self.tc.ENDC
        self.active = True
        self.start_time = timestamp

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
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler

    def getDistance(self):
        '''
        calculates distance between position and goal
        :return: distance in [m]
        '''
        delta_x = (self.A_listener_position[-1, AD.pX] - self.A_listener_goal[-1, AD.pX])
        delta_y = (self.A_listener_position[-1, AD.pY] - self.A_listener_goal[-1, AD.pY])
        distance = math.sqrt(delta_x ** 2 + delta_y ** 2)
        # return delta distance in [m]
        return distance

    def getAngle(self):
        '''
        calculates the angle around z-axis between position and goal
        :return: yaw-angle in degree
        '''
        quaternion_position = [self.A_listener_position[-1, AD.qX],
                               self.A_listener_position[-1, AD.qY],
                               self.A_listener_position[-1, AD.qZ],
                               self.A_listener_position[-1, AD.qW]]
        quaternion_goal = [self.A_listener_goal[-1, AD.qX],
                           self.A_listener_goal[-1, AD.qY],
                           self.A_listener_goal[-1, AD.qZ],
                           self.A_listener_goal[-1, AD.qW]]
        euler_position = self.quaternion2euler(quaternion=quaternion_position)
        euler_goal = self.quaternion2euler(quaternion=quaternion_goal)
        print 'Yaw Position: {:.3f} [rad]'.format(euler_position[2])
        print 'Yaw Goal: {:.3f} [rad]'.format(euler_goal[2])
        # return absolut value for delta yaw in [radian]
        return math.degrees(math.fabs(euler_goal[2] - euler_position[2]))

    def get_result(self):
        '''
        calculates difference between position goal and actual position
        calculates difference between angle around z-axis and max allowed angle difference
        :return: True - distance and angle <= max allowed values
        :return: False - distance and angle > max allowed values
        '''
        groundtruth_result = None
        details = {'topic': self.positiontopic}
        if self.finished:
            print 'Groundtruth: {:.3f} [m]'.format(self.groundtruth)
            print 'Groundtruth Epsilon: {:.3f} [degree]'.format(self.groundtruth_epsilon)
            if self.groundtruth != None and self.groundtruth_epsilon != None:
                if self.getDistance() <= self.groundtruth and self.getAngle() <= self.groundtruth_epsilon:
                    print self.tc.OKGREEN + '=' * 23
                    print 'Max distance: {:.4f} [m]'.format(self.getDistance())
                    print '=' * 23 + self.tc.ENDC
                    data = self.getDistance()
                    groundtruth_result = True
                else:
                    print self.tc.FAIL + '=' * 59
                    print 'Distance to far: {:.4f} [m], max allowed: {:.3f} [m]'.format(self.getDistance(),
                                                                                    self.groundtruth)
                    print 'Angle to big: {:.3f} [degree], max allowed: {:.3f} [degree]'.format(self.getAngle(),
                                                                                     self.groundtruth_epsilon)
                    print '=' * 59 + self.tc.FAIL
                    data = -1
                    groundtruth_result = False
            return 'goal', data, groundtruth_result, self.groundtruth, self.groundtruth_epsilon, details
        else:
            return False


if __name__ == '__main__':
    # CalculateGoal( distance [m], angle [degrees])
    cg = CalculateGoal(0.2, 20)
    cg.start(rospy.Time.now())
    rospy.spin()
    cg.stop(rospy.Time.now())
    cg.get_result()
pass
