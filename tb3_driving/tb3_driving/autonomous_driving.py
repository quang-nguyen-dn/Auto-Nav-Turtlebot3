import os
import select
import sys
import numpy as np
import rclpy
import tb3_driving.opencv_graphing as graph

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, String
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tb3_driving.param import ROBOT_MARGIN, THRESHOLD, STOP_THRESHOLD, THRESHOLD_ANGLE,\
    MIN_THRESHOLD, THRESHOLD_STEP, ANGLE_TO_START_MOVING

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

PI = np.pi
BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

IS_AUTONOMOUS = False
TARGET = [0.0, 0.0]
#ROBOT_MARGIN = 180

MAP_RANGE = [4000, 6000, 10000]
MAP_CIRCLES = [4, 6, 10]


def rad(x: float):
    return x/180*PI


SETTINGS = None
if os.name != 'nt':
    SETTINGS = termios.tcgetattr(sys.stdin)


def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class AutonomousDriving(Node):
    """ROS2 Node for autonomous driving without localisation"""

    def __init__(self):
        super().__init__('autonomous_driving')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.scan_ranges = []
        self.init_scan_state = False  # To get the initial scan data at the beginning
        self.notified_scan_state = False

        self.linear_speed = 0
        self.angular_speed = 0
        self.control_linear_speed = 0
        self.control_angular_speed = 0

        self.pose_init = False
        self.initial_coord_reset = False
        self.xi = 0.0
        self.yi = 0.0
        self.theta_i = 0.0

        self.map_scale_level = 0
        self.my_map = graph.Graph(MAP_RANGE[0], MAP_RANGE[0])
        self.my_map.generate_range_circles(MAP_CIRCLES[0])
        self.pose = [0, 0, 0]
        self.original_pose = [0, 0, 0]

        self.is_autonomous = False
        self.target = TARGET
        self.target_array = np.full((360, 2), np.inf)
        self.target_polar = np.full(360, np.inf)

        self.sub_target = [np.inf, np.inf]
        self.sub_target_array = np.full((360, 2), np.inf)
        self.sub_target_polar = np.full(360, np.inf)
        self.Sub_targets_angles = np.full(len(MAP_RANGE), np.inf)

        self.completed_autonomous_driving = True
        self.init_autonomous = 0
        self.robot_status = Int16()
        self.robot_status.data = 0
        self.auto_mode_exit = False
        self.available_direction = np.zeros(360)
        self.threshold = 1.5
        self.max_threshold = THRESHOLD

        self.selected_direction = 0
        self.was_moving_left = False
        self.was_moving_right = False
        self.waypoint = [-1, 0]
        self.waypoint_cartesian = [np.inf, np.inf]
        self.waypoint_angle = np.inf

        self.gone_stupid = False
        self.gone_stupid_times = 0
        self.is_ok_times = 0
        self.is_really_stupid = False
        self.is_really_stupid_times = 0
        self.old_angular_speed = 0.0
        self.is_solving_stupidity = False

        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0

        self.robot_status_pub = self.create_publisher(
            Int16, 'robot_status', qos)

        self.robot_status_pub.publish(self.robot_status)
        self.robot_status_published = False

        # Initialise subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_update_callback,
            qos_profile=qos_profile_sensor_data)

        self.odometry_sub = self.create_subscription(
            Odometry,
            'odom',
            self.pose_callback,
            qos_profile=qos_profile_sensor_data)
        
        self.command_sub = self.create_subscription(
            String,
            'driving_command',
            self.command_monitor,
            10
        )

        """************************************************************
        ** Initialise timers
        ************************************************************"""
        self.update_timer = self.create_timer(
            0.01,  # unit: s
            self.operate_autonomous_driving)

        self.get_logger().info("Finished initialising modules")
        # self.scan_update()

    def command_monitor(self, msg):
        message = msg.data.split(', ')
        self.init_autonomous = int(message[0])
        self.target = [float(message[1]), float(message[2])]
        self.auto_mode_exit = int(message[3])
        if int(message[4]) == True:
            self.reset_coord()
            #self.get_logger().info("Coordinates have been reset to zeroes")
        # print("Received data from controller: {}, {}, {}, {}".format(self.init_autonomous,\
        #    self.target[0], self.target[1], self.auto_mode_exit))
    
    def reset_coord(self):
        if self.pose_init:
            [self.xi, self.yi, self.theta_i] = self.original_pose
            self.pose = [0.0, 0.0, 0.0]
            self.get_logger().info("Coordinates have been reset to zeroes")

    def pose_callback(self, msg):
        theta = 2*np.arccos(msg.pose.pose.orientation.w)
        if msg.pose.pose.orientation.z < 0:
            theta = -theta
        theta = self.standardise_angle(theta)
        self.original_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, theta)
        self.pose = [self.original_pose[0] - self.xi, self.original_pose[1] - self.yi, 
                        self.original_pose[2] - self.theta_i]
        self.linear_speed = msg.twist.twist.linear.x
        self.angular_speed = msg.twist.twist.angular.z
        self.pose_init = True

    def cartesian(self, points, start_p: int = 0, end_p: int = 360, stepp: int = 1, excp=np.inf):
        result = np.full((end_p-start_p, 2), excp)
        for i in range(start_p, end_p, stepp):
            if points[i] != excp:
                result[i][0] = (points[i] * np.cos(i/180*PI)) * 1000
                result[i][1] = (points[i] * np.sin(i/180*PI)) * 1000
            else:
                result[i][0] = excp
                result[i][1] = excp
        return result

    def scan_update_callback(self, msg):
        self.scan_ranges = msg.ranges
        for i in range(360):
            if self.scan_ranges[i] <= 0.05:
                self.scan_ranges[i] = np.inf
        self.init_scan_state = True
        self.update_map()

    def update_map(self):
        if not self.notified_scan_state:
            self.get_logger().info("Connection to LIDAR established")
            self.notified_scan_state = True
        #pos = self.cartesian(self.scan_ranges)

        self.available_direction = self.get_available_directions()
        #pos2 = self.cartesian(self.available_direction * self.threshold)

        current_point = [self.pose[0], self.pose[1]]
        dist = self.distance(current_point, self.target)

        #r = self.waypoint[0]
        #theta = self.waypoint[1] / 180*PI
        #phi = self.pose[2]

        if self.waypoint[0] != -1:
            #waypoint = [
            #    current_point[0] + r * np.cos(theta + phi),
            #    current_point[1] + r * np.sin(theta + phi)
            #]
            waypoint = self.cartesian_absolute(self.waypoint)
        else:
            waypoint = [np.inf, np.inf]

        if not self.is_autonomous:
            #self.target_array = np.full((360, 2), np.inf)
            #self.sub_target_array = np.full((360, 2), np.inf)
            #target_to_plot = [np.inf, np.inf]
            #sub_target_to_plot = [np.inf, np.inf]
            if self.map_scale_level != 0:
                self.map_scale_level = 0
                self.my_map = graph.Graph(MAP_RANGE[0], MAP_RANGE[0])
                self.my_map.generate_range_circles(MAP_CIRCLES[0])

        else:
            #self.target_polar = np.full(360, np.inf)
            #self.sub_target_polar = np.full(360, np.inf)
            #target_to_plot = self.target
            #sub_target_to_plot = self.sub_target

            
            #self.target_polar[
            #    int(self.control_theta(current_point, self.target) / PI*180)]\
            #    = self.distance(current_point, self.target)
            #self.sub_target_polar[
            #    int(self.control_theta(current_point, self.sub_target) / PI*180)]\
            #    = self.distance(current_point, self.sub_target)

            #self.target_array = self.cartesian(self.target_polar)
            #self.sub_target_array = self.cartesian(self.sub_target_polar)

            # Update map scale based on distance to target
            
            map_scale_level = 0
            for i in range(len(MAP_RANGE)-2, -1, -1):
                if dist*1000 > MAP_RANGE[i]:
                    map_scale_level = i+1
                    break
            if self.map_scale_level != map_scale_level:
                self.map_scale_level = map_scale_level
                self.my_map = graph.Graph(
                    MAP_RANGE[map_scale_level], MAP_RANGE[map_scale_level])
                self.my_map.generate_range_circles(
                    MAP_CIRCLES[map_scale_level])

        self.my_map.update_map(self.scan_ranges, self.available_direction*self.threshold,
                                self.target, self.sub_target, self.waypoint_cartesian,
                                self.pose, self.linear_speed,
                                self.angular_speed, self.is_autonomous)

    def move_linear_with_speed(self, lin_speed):
        if np.abs(lin_speed) > BURGER_MAX_LIN_VEL:
            lin_speed = BURGER_MAX_LIN_VEL * np.sign(lin_speed)
        lin_speed = float(lin_speed)
        self.control_linear_speed = lin_speed
        self.twist.linear.x = lin_speed
        self.cmd_vel_pub.publish(self.twist)

    def rotate_with_speed(self, ang_speed):
        if np.abs(ang_speed) > BURGER_MAX_ANG_VEL:
            ang_speed = BURGER_MAX_ANG_VEL * np.sign(ang_speed)
        ang_speed = float(ang_speed)
        self.control_angular_speed = ang_speed
        self.twist.angular.z = ang_speed
        self.cmd_vel_pub.publish(self.twist)

    def stop_moving(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

    def min_distance_ahead(self, angle_range: int = 90, central_angle: int = 0):
        ahead_distance = []
        """for i in range(central_angle, central_angle + int(angle_range/2)):
            while i >= 360: i -= 360
            while i < 0: i += 360
            ahead_distance.append(self.scan_ranges[i])
        for i in range(central_angle + 359, central_angle + 359 - int(angle_range/2) - 1, -1):
            while i >= 360: i -= 360
            while i < 0: i += 360
            ahead_distance.append(self.scan_ranges[i])"""
        for i in range(central_angle - int(angle_range/2), central_angle + int(angle_range/2)):
            while i >= 360:
                i -= 360
            while i < 0:
                i += 360
            ahead_distance.append(self.scan_ranges[i])
        return min(ahead_distance)

    def ahead_is_available(self, angle_range: int = 20, central_angle: int = 0):
        for i in range(central_angle - int(angle_range/2), central_angle + int(angle_range/2)):
            while i >= 360:
                i -= 360
            while i < 0:
                i += 360
            if not self.available_direction[i]:
                return False
        return True

    def robot_gone_stupid_check(self):
        if np.abs(self.linear_speed) < 0.005:
            if self.angular_speed*self.old_angular_speed < 0:
                self.gone_stupid = True
                self.get_logger().info("Robot has gone stupid")
                self.gone_stupid_times += 1
                self.is_ok_times = 0
                if self.gone_stupid_times == 5:
                    self.get_logger().info("Robot HAS REALLY GONE STUPID")
                    self.is_really_stupid_times += 1
                    self.gone_stupid_times = 0
                    self.is_really_stupid = True
            else:
                self.is_ok_times += 1
                if self.is_ok_times >= 100:
                    self.gone_stupid = False
                    self.is_really_stupid = False
                    self.is_really_stupid_times = 0
                    self.gone_stupid_times = 0
                    self.get_logger().info("Robot is OK")
                    self.is_ok_times = 0
        self.old_angular_speed = self.angular_speed

    def cartesian_absolute(self, polar_point):
        xc = self.pose[0]
        yc = self.pose[1]
        phi = self.pose[2]
        r = polar_point[0]
        theta = polar_point[1] / 180*PI
        return [xc + r*np.cos(theta + phi), yc + r*np.sin(theta + phi)]

    def weighted_linear_speed(self):
        min_distance_ahead = self.min_distance_ahead(40)
        if min_distance_ahead >= 2.0:
            return BURGER_MAX_LIN_VEL
        elif min_distance_ahead >= 1.0:
            return 0.8*BURGER_MAX_LIN_VEL
        elif min_distance_ahead >= 0.5:
            return 0.6*BURGER_MAX_LIN_VEL
        else:
            return 0.4*BURGER_MAX_LIN_VEL
    
    def standardise_angle_deg(self, angle_in_deg, output_less_than_180=False):
        while angle_in_deg >= 360: angle_in_deg -= 360
        while angle_in_deg < 0: angle_in_deg += 360
        if angle_in_deg <= 180 or not output_less_than_180:
            return angle_in_deg
        else:
            return 360 - angle_in_deg

    def standardise_angle(self, angle):
        while angle < 0:
            angle += 2*PI
        while angle >= 2*PI:
            angle -= 2*PI
        return angle

    def distance(self, point1, point2):
        return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    def control_theta(self, origin, target):
        theta = self.standardise_angle(self.pose[2])
        theta_t = np.arctan2(target[1]-origin[1], target[0]-origin[0])
        theta_t = self.standardise_angle(theta_t)
        return theta_t - theta

    def move_towards(self, target):
        current_location = [self.pose[0], self.pose[1]]
        distance = self.distance(target, current_location)

        if distance <= STOP_THRESHOLD:
            self.stop_moving()
            #self.get_logger().info("Reached point ({:.3f}, {:.3f})".format(target[0], target[1]))
            return 1

        Theta = self.control_theta(current_location, target)
        #print("Current, target, control: {}, {}, {}".format(theta, theta_t, Theta))

        if np.abs(Theta) >= PI:
            sign_Theta = -np.sign(Theta)
        else:
            sign_Theta = np.sign(Theta)
        #print(theta, theta_t, Theta, sign_Theta)
        weighted_linear_speed = self.weighted_linear_speed()

        deflection_angle = self.standardise_angle(Theta)
        if deflection_angle > PI:
            deflection_angle = 2*PI - deflection_angle
        #print("Deflection angle: {:.3f}".format(deflection_angle/PI*180))

        if deflection_angle > 3*PI/2:
            self.rotate_with_speed(1.0*sign_Theta)
            self.move_linear_with_speed(0)
        elif deflection_angle > PI/2:
            self.rotate_with_speed(0.7*sign_Theta)
            self.move_linear_with_speed(0)
        elif deflection_angle > PI/4:
            self.rotate_with_speed(0.5*sign_Theta)
            self.move_linear_with_speed(0)
        elif deflection_angle > PI/6:
            self.rotate_with_speed(0.3*sign_Theta)
            self.move_linear_with_speed(0)
        elif deflection_angle > ANGLE_TO_START_MOVING:
            self.rotate_with_speed(0.2*sign_Theta)
            self.move_linear_with_speed(0)
        else:
            if deflection_angle <= PI/60:
                self.rotate_with_speed(0.0)
            elif deflection_angle <= PI/45:
                self.rotate_with_speed(0.07*sign_Theta)
            elif deflection_angle <= PI/36:
                self.rotate_with_speed(0.1*sign_Theta)
            else:
                self.rotate_with_speed(0.15*sign_Theta)
            # elif np.abs(Theta) <= PI/4:
            #    self.rotate_with_speed(0.35*sign_Theta)

            if distance >= 2.0:
                self.move_linear_with_speed(weighted_linear_speed)
            elif distance >= 1.0:
                self.move_linear_with_speed(0.9*weighted_linear_speed)
            elif distance >= 0.5:
                self.move_linear_with_speed(0.8*weighted_linear_speed)
            else:
                self.move_linear_with_speed(0.7*weighted_linear_speed)
        return 0

    def get_available_directions(self):
        #surrounding_range = -1.0
        # for i in range(360):
        #    if self.scan_ranges[i] < np.inf and self.scan_ranges[i] > surrounding_range:
        #        surrounding_range = self.scan_ranges[i]
        #if surrounding_range >= THRESHOLD: self.threshold = THRESHOLD
        # else: self.threshold = surrounding_range
        r = np.inf
        dead_angle = -np.inf
        result = np.zeros(360, int)
        threshold = self.threshold
        robot_margin = ROBOT_MARGIN * 1.0 / 1000
        for i in range(540):
            index = self.standardise_angle_deg(i)
            ri = self.scan_ranges[index]
            #if ri == 0 or np.abs(robot_margin / (2*ri)) > 1:
            #    result[index] = 0
            #    continue
            if ri < threshold:
                r = ri
                dead_angle = i + round(2 * np.arcsin(robot_margin / (2*r)) / PI*180)
                result[index] = 0
            else:
                if i > dead_angle:
                    r = np.inf
                    dead_angle = -np.inf
                    result[index] = 1
                else:
                    result[index] = 0

        r = np.inf
        dead_angle = np.inf
        for i in range(359, -180, -1):
            index = self.standardise_angle_deg(i)
            ri = self.scan_ranges[index]
            #if ri == 0 or np.abs(robot_margin / (2*ri)) > 1: 
            #    result[index] = 0
            #    continue
            if ri < threshold:
                r = ri
                dead_angle = i - round(2 * np.arcsin(robot_margin / (2*r)) / PI*180)
                result[index] = 0
            else:
                if i < dead_angle:
                    r = np.inf
                    dead_angle = np.inf
                    #result[i] = 1
                else:
                    result[index] = 0

        return result

    def autonomous_driving(self, target):
        current_location = [self.pose[0], self.pose[1]]
        distance = self.distance(current_location, target)

        if distance <= STOP_THRESHOLD:
            self.stop_moving()
            self.get_logger().info("Reached destination ({:.3f}, {:.3f})"
                                   .format(target[0], target[1]))
            return 1

        self.max_threshold = THRESHOLD
        while self.max_threshold > distance + THRESHOLD_STEP:
            self.max_threshold -= THRESHOLD_STEP
        
        #if self.waypoint[0] != -1:
        #    self.waypoint_cartesian = self.cartesian_absolute(self.waypoint)
        #else:
        #    self.waypoint_cartesian = [np.inf, np.inf]
        if self.waypoint[0] != -1:
            dist = self.distance(current_location, self.waypoint_cartesian)
        else:
            dist = np.inf

        self.threshold = MIN_THRESHOLD
        self.available_direction = self.get_available_directions()
        self.update_map()

        Theta = self.control_theta(current_location, target)
        target_angle_index = self.standardise_angle_deg(int(Theta/PI*180))
        #current_theta = self.pose[2]

        Theta_2 = self.control_theta(current_location, self.waypoint_cartesian)
        waypoint_index = self.standardise_angle_deg(int(Theta_2/PI*180))
        #self.get_logger().info("Target is at angle index: {}".format(target_angle_index))
        selected_direction = target_angle_index
        right_angle = 0
        left_angle = 359

        
        is_moving_towards_target = False
        is_moving_towards_waypoint = False

        if self.ahead_is_available(2, target_angle_index):
            while (\
                self.ahead_is_available(2, target_angle_index) and\
                self.threshold < self.max_threshold\
            ):
                self.threshold += THRESHOLD_STEP
                self.available_direction = self.get_available_directions()
                self.update_map()
            #self.get_logger().info("Target detected at threshold: {}".format(self.threshold))

            if self.threshold >= distance or self.threshold == self.max_threshold:
                #self.move_towards(target)
                self.sub_target = target
                is_moving_towards_target = True
                self.waypoint[0] = -1
                self.waypoint_angle = np.inf
                self.waypoint_cartesian = [np.inf, np.inf]
                self.update_map()
                #self.get_logger().info("Moving towards    TARGET")

        elif self.waypoint[0] != -1 and self.ahead_is_available(2, waypoint_index):
            while (\
                self.ahead_is_available(2, waypoint_index) and\
                self.threshold < self.max_threshold\
            ):
                self.threshold += THRESHOLD_STEP
                self.available_direction = self.get_available_directions()
                self.update_map()
            
            if self.threshold >= dist or self.threshold == self.max_threshold:
                #self.move_towards(self.waypoint_cartesian)
                self.sub_target = self.waypoint_cartesian
                is_moving_towards_waypoint = True
                self.update_map()
                #self.get_logger().info("Moving towards:   Waypoint")

        #if self.sub_target != target and self.sub_target != self.waypoint_cartesian:
            #sub_dist = self.distance(current_location, self.sub_target)

            # self.get_logger().info("Sub_target is at threshold: {} and distance: {} m".\
            #    format(self.threshold, sub_dist))

        self.move_towards(self.sub_target)

            #self.get_logger().info("Moving towards sub_target")

        if not is_moving_towards_target and not is_moving_towards_waypoint:

            sub_target_selected = False
            self.threshold = self.max_threshold + THRESHOLD_STEP
            while not sub_target_selected and self.threshold > MIN_THRESHOLD:
                if self.threshold > MIN_THRESHOLD:
                    self.threshold -= THRESHOLD_STEP
                #self.get_logger().info("Considering threshold: {}".format(self.threshold))
                self.available_direction = self.get_available_directions()
                #self.update_map()

                if self.threshold >= dist or self.waypoint[0] == -1:
                    prioritised_direction = target_angle_index
                    #self.get_logger().info("Prioritising:   Target")
                else:
                    prioritised_direction = waypoint_index
                    #self.get_logger().info("Prioritising:   Waypoint")

                right_index = np.inf
                left_index = np.inf

                for i in range(prioritised_direction, prioritised_direction+180, 1):
                    index = self.standardise_angle_deg(i)
                    if self.available_direction[index]:
                        left_index = index
                        break
                for i in range(prioritised_direction + 359, prioritised_direction + 179, -1):
                    index = self.standardise_angle_deg(i)
                    if self.available_direction[index]:
                        right_index = index
                        break
                #selected_direction = min(right_angle, 359-left_angle)
                if right_index != np.inf:
                    right_angle = self.standardise_angle_deg(
                                    right_index - prioritised_direction, True)
                else:
                    right_angle = np.inf

                if left_index != np.inf:
                    left_angle = self.standardise_angle_deg(
                                    left_index - prioritised_direction, True)
                else:
                    left_angle = np.inf

                if right_index == np.inf and left_index == np.inf:
                    continue

                if right_angle < left_angle:
                    selected_direction = right_index
                else:
                    selected_direction = left_index
                #print("Left: {:3.0f}, right: {:3.0f}, selected left: {}".
                #    format(left_angle, right_angle, selected_direction == left_index))
                
                new_angle_to_old = self.standardise_angle_deg(
                                    selected_direction - self.selected_direction, True)
                #print("Compared to previously selected angle: {}".format(
                #    new_angle_to_old))

                if new_angle_to_old < 150:
                    self.selected_direction = selected_direction

            #if self.threshold >= self.waypoint[0]:
                deflection_angle = self.standardise_angle_deg(
                                    self.selected_direction - prioritised_direction, True)
                #self.get_logger().info("Deflection angle: {} deg".format(deflection_angle))

                if self.threshold < dist and self.waypoint[0] != -1:

                    if deflection_angle <= THRESHOLD_ANGLE or self.threshold == MIN_THRESHOLD:
                        self.sub_target = self.cartesian_absolute(
                            [self.threshold, self.selected_direction]
                        )
                        sub_target_selected = True
                        #self.move_towards(self.sub_target)
                        # self.get_logger().info("Selected new sub_target: ({:.3f}, {:.3f})".\
                        #    format(self.sub_target[0], self.sub_target[1]))
                        #self.get_logger().info("Updated:   Sub-target")

                    if deflection_angle > THRESHOLD_ANGLE:
                        #if self.threshold > MIN_THRESHOLD: self.threshold -= THRESHOLD_STEP
                        continue
                        # if self.threshold == MIN_THRESHOLD:
                        #    break
                else:
                    if (self.waypoint[0] == self.threshold 
                            and deflection_angle < self.waypoint_angle
                            and deflection_angle <= 90) \
                        or (self.waypoint[0] > self.threshold
                            and deflection_angle <= 90) \
                        or (self.waypoint[0] == -1
                            and deflection_angle <= 90):

                        self.waypoint = [self.threshold, self.selected_direction]
                        self.waypoint_cartesian = self.cartesian_absolute(self.waypoint)
                        self.waypoint_angle = deflection_angle
                        self.sub_target = self.waypoint_cartesian
                        sub_target_selected = True
                        #self.get_logger().info("Updated WAYPOINT")

            if not sub_target_selected and self.threshold == MIN_THRESHOLD:
                return -1
            #self.get_logger().info("Checking if target or sub_target is available")

        return 0

    def operate_autonomous_driving(self):
        if get_key(SETTINGS) in ['\x03', 'q']:
            self.update_timer.cancel()
            self.stop_moving()
            self.robot_status.data = 0
            self.robot_status_pub.publish(self.robot_status)
            self.get_logger().info("Programme closed cleanly.")
            sys.exit()

        #if not self.initial_coord_reset:
        #    self.reset_coord()
        #    if self.pose_init:
        #        self.initial_coord_reset = True

        if self.init_autonomous and not self.is_autonomous:
            self.get_logger().info("Initialising autonomous driving to target: ({:.3f}, {:.3f})".
                                   format(self.target[0], self.target[1]))
            self.is_autonomous = True
            self.init_autonomous = False
            self.sub_target = self.target
            self.completed_autonomous_driving = False
            Theta = self.control_theta(
                [self.pose[0], self.pose[1]], self.target)
            self.selected_direction = int(Theta/PI*180)

        if self.is_autonomous and not self.completed_autonomous_driving and self.init_scan_state:
            if not self.robot_status_published:
                self.robot_status.data = 1
                self.robot_status_pub.publish(self.robot_status)
                self.robot_status_published = True
            autonomous_driving_status = self.autonomous_driving(self.target)
            if autonomous_driving_status == 1 or autonomous_driving_status == -1:
                # if self.move_towards(self.target) == 1:
                self.completed_autonomous_driving = True
                self.is_autonomous = False
                self.robot_status.data = 0
                self.robot_status_pub.publish(self.robot_status)
                self.robot_status_published = False
                self.threshold = 1.5
                self.waypoint = [-1, 0]
                self.waypoint_cartesian = [np.inf, np.inf]
                self.waypoint_angle = np.inf
                if autonomous_driving_status == -1:
                    self.get_logger().info("There is no available direction for the robot \
                        to move to. Please consider moving it to a clear area.\n\
                            Autonomous driving mode disrupted.")

        if self.auto_mode_exit:
            self.auto_mode_exit = False
            if self.is_autonomous and not self.completed_autonomous_driving:
                self.completed_autonomous_driving = True
                self.robot_status.data = 0
                self.robot_status_pub.publish(self.robot_status)
                self.robot_status_published = False
                self.is_autonomous = False
                self.completed_autonomous_driving = True
                self.stop_moving()
                self.threshold = 1.5
                self.waypoint = [-1, 0]
                self.waypoint_cartesian = [np.inf, np.inf]
                self.waypoint_angle = np.inf
                self.get_logger().info("Autonomous driving mode interrupted. Robot stopped.")


def main(args=None):
    print("Hello, beginning operation\n")

    rclpy.init(args=args)
    auto_drive = AutonomousDriving()
    rclpy.spin(auto_drive)

    auto_drive.destroy_node()
    print("Program exitting")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
