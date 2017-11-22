#!/usr/bin/env python
import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
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
            # metrics.append(CalculateGoal(metric["topic"], metric["groundtruth_angle"], metric["groundtruth_angle_epsilon"],
            #                   groundtruth, groundtruth_epsilon))
        return metrics


class CalculateGoal:
    # def __init__(self, topic, groundtruth_angle, groundtruth_angle_epsilon, groundtruth, groundtruth_epsilon):
    def __init__(self, topic, groundtruth, groundtruth_epsilon):
        self.active = False
        self.finished = False
        self.positiontopic = '/base_pose_ground_truth'
        self.goaltopic = topic
        self.groundtruth = groundtruth
        self.groundtruth_epsilon = groundtruth_epsilon
        # self.groundtruth_angle = float(groundtruth_angle)
        # self.groundtruth_angle_epsilon = float(groundtruth_angle_epsilon)
        self.targetgoal = None
        self.softgoal = None
        # create array for further use
        self.A_listener_position = np.ones([0, 6], dtype=np.double)
        self.A_listener_goal = np.ones([0, 6], dtype=np.double)

        # subscribe to topics in need
        rospy.Subscriber(self.positiontopic, Odometry, self.callback_position, queue_size=None)
        # /move_base/goal has MoveBaseActionGoal msg type
        rospy.Subscriber(self.goaltopic, MoveBaseActionGoal, self.callback_goal, queue_size=None)

    def callback_position(self, msg):
        # if self.active:
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
        # msg bei /move_base_simple/goal topic
        # msg.pose.position.x
        # msg.pose.position.y
        # msg.pose.orientation.x
        # msg.pose.orientation.y
        # msg.pose.orientation.z
        # msg.pose.orientation.w
        if self.active:
            data_list = [float(msg.goal.target_pose.pose.position.x),
                         float(msg.goal.target_pose.pose.position.y),
                         float(msg.goal.target_pose.pose.orientation.x),
                         float(msg.goal.target_pose.pose.orientation.y),
                         float(msg.goal.target_pose.pose.orientation.z),
                         float(msg.goal.target_pose.pose.orientation.w)]

            rospy.loginfo('\033[91m' + '----callback goal----' + '\033[0m')
            rospy.loginfo('\033[1m' + 'Goal X: ' + str(data_list[0]) + '\t[m]' + '\033[0m')
            rospy.loginfo('\033[1m' + 'Goal Y: ' + str(data_list[1]) + '\t[m]' + '\033[0m')

            # append data to array
            self.A_listener_goal = np.append(self.A_listener_goal,
                                             [[data_list[0], data_list[1], data_list[2], data_list[3], data_list[4],
                                               data_list[5]]], axis=0)

    def start(self, timestamp):
        self.active = True
        self.start_time = timestamp
        # rospy.loginfo('\033[91m' + '----goal.py----' + '\033[0m')

    def stop(self, timestamp):
        self.active = False
        self.stop_time = timestamp
        self.finished = True

        # rospy.loginfo('\033[94m' + '=' * 82 + '\033[0m')
        # result = self.get_result()
        # rospy.loginfo('\033[94m' + '=' * 82 + '\033[0m')
        # rospy.loginfo('\033[94m' + 'Result: ' + str(result) + '\033[0m')
        # rospy.loginfo('\033[94m' + '=' * 82 + '\033[0m')

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
        delta_x = (self.A_listener_position[-1, AD.pX] - self.A_listener_goal[-1, AD.pX])
        delta_y = (self.A_listener_position[-1, AD.pY] - self.A_listener_goal[-1, AD.pY])
        distance = math.sqrt(delta_x ** 2 + delta_y ** 2)
        # return delta distance in [m]
        return distance

    def getAngle(self):
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
        # return absolut value for delta yaw in [degree]
        return math.degrees(math.fabs(euler_goal[2] - euler_position[2]))

    def terminal_output(self, bool):
        '''
        print information to terminal
        :param bool: True - distance < groundtruth_epsilon - output is green
        :param bool: False - distance > groundtruth_epsilon - output is red
        :return: --
        '''
        if bool:
            rospy.loginfo('\033[92m' + 'Distance:\t\t\t' + str(self.getDistance()) + ' [m]' + '\033[0m')
            rospy.loginfo('\033[92m' + 'Angle:\t\t\t\t' + str(self.getAngle()) + ' [degree]' + '\033[0m')
            rospy.loginfo('\033[92m' + 'Groundtruth Position:\t\t' + str(self.groundtruth) + ' [m]' + '\tType: ' + str(
                type(self.groundtruth)) + '\033[0m')
            rospy.loginfo(
                '\033[92m' + 'Groundtruth Position Epsilon:\t' + str(
                    self.groundtruth_epsilon) + ' [m]' + '\tType: ' + str(
                    type(self.groundtruth_epsilon)) + '\033[0m')
            # rospy.loginfo('\033[92m' + 'Groundtruth Angle:\t\t' + str(self.groundtruth_angle) + ' [degree]' + '\tType: ' + str(type(self.groundtruth_angle)) + '\033[0m')
            # rospy.loginfo('\033[92m' + 'Groundtruth Angle Epsilon:\t' + str(self.groundtruth_angle_epsilon) + ' [degree]' + '\tType: ' + str(type(self.groundtruth_angle_epsilon)) + '\033[0m')

        else:
            rospy.loginfo('\033[91m' + 'Distance:\t\t\t' + str(self.getDistance()) + ' [m]' + '\033[0m')
            rospy.loginfo('\033[91m' + 'Angle:\t\t\t\t' + str(self.getAngle()) + ' [degree]' + '\033[0m')
            rospy.loginfo('\033[91m' + 'Groundtruth Position:\t\t' + str(self.groundtruth) + ' [m]' + '\tType: ' + str(
                type(self.groundtruth)) + '\033[0m')
            rospy.loginfo(
                '\033[91m' + 'Groundtruth Position Epsilon:\t' + str(
                    self.groundtruth_epsilon) + ' [m]' + '\tType: ' + str(
                    type(self.groundtruth_epsilon)) + '\033[0m')
            # rospy.loginfo('\033[91m' + 'Groundtruth Angle:\t\t' + str(self.groundtruth_angle) + '\tType: ' + str(type(self.groundtruth_angle)) + '\033[0m')
            # rospy.loginfo('\033[91m' + 'Groundtruth Angle Epsilon:\t' + str(self.groundtruth_angle_epsilon) + ' [degree]' + '\tType: ' + str(type(self.groundtruth_angle_epsilon)) + '\033[0m')

    def get_result(self):
        groundtruth_result = None
        details = {'topic': self.goaltopic}
        if self.finished:
            # save file for debugging
            f = open('/home/flg-ma/PycharmProjects/goal_metrics/debug/calculate_goal.txt', 'w')
            f.write(str(self.A_listener_position))
            f.write('\n' + '=' * 20 + '\n')
            f.write(str(self.A_listener_goal))
            f.close()

            if self.groundtruth != None and self.groundtruth_epsilon != None:
                # if distance <= self.groundtruth and angle <= self.groundtruth_epsilon:
                if (self.getDistance() - self.groundtruth) <= self.groundtruth_epsilon:
                    self.terminal_output(True)
                    data = self.getDistance()
                    groundtruth_result = True
                else:
                    self.terminal_output(False)
                    data = self.getDistance()
                    groundtruth_result = False
            return 'goal', data, groundtruth_result, self.groundtruth, self.groundtruth_epsilon, details
        else:
            return False
