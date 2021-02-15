import os
import select
import sys
if os.name == 'nt':
    import msvcrt
else:
    import termios, tty

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16, String
from geometry_msgs.msg import Twist

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84
WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

settings = None
if os.name != 'nt':
    settings = termios.tcgetattr(sys.stdin)

def print_vels(target_linear_velocity, target_angular_velocity):
    print('currently:\tlinear velocity {0}\t angular velocity {1} '.format(
        target_linear_velocity,
        target_angular_velocity))

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

def make_simple_profile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output


def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


def check_linear_limit_velocity(velocity):
    if TURTLEBOT3_MODEL == 'burger':
        return constrain(velocity, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    else:
        return constrain(velocity, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)

def check_angular_limit_velocity(velocity):
    if TURTLEBOT3_MODEL == 'burger':
        return constrain(velocity, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    else:
        return constrain(velocity, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)


class ControllerPublisher(Node):

    def __init__(self):
        super().__init__('controller')
        self.auto_driving_instruction = self.create_publisher(
            String,
            'driving_command',
            10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_info = self.create_subscription(
            Int16,
            'robot_status',
            self.listener_callback,
            10
        )
        self.robot_is_busy = False

        self.target_linear_velocity   = 0.0
        self.target_angular_velocity  = 0.0
        self.control_linear_velocity  = 0.0
        self.control_angular_velocity = 0.0

        self.timer = self.create_timer(0.01, self.keyboard_control)
        self.target = [0.0, 0.0]
        self.twist = Twist()

    def listener_callback(self, msg):
        self.robot_is_busy = msg.data
        #print("Robot status updated: {}".format(self.robot_is_busy))
        if self.robot_is_busy:
            self.get_logger().info("Robot status updated: BUSY")
        else:
            self.get_logger().info("Robot status updated: AVAILABLE")


    def keyboard_control(self):
        key = get_key(settings)
        cmd = String()

        if key == '0':
            cmd.data = '0, {}, {}, 1, 0'.format(self.target[0], self.target[1])
            self.auto_driving_instruction.publish(cmd)
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            self.robot_is_busy = False
            self.get_logger().info("Autonomous mode terminated")

        elif key == '\x03' or key == 'q':
            self.timer.cancel()
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            self.get_logger().info("Programme exitting")
            sys.exit()
        
        elif key == '*':
            cmd.data = '0, {}, {}, 0, 1'.format(self.target[0], self.target[1])
            self.auto_driving_instruction.publish(cmd)
            self.get_logger().info("Sent request for resetting coordinates")

        elif not self.robot_is_busy:
            if key == 'w':
                self.target_linear_velocity =\
                    check_linear_limit_velocity(self.target_linear_velocity + LIN_VEL_STEP_SIZE)
                #print_vels(self.target_linear_velocity, self.target_angular_velocity)
            elif key == 's':
                self.target_linear_velocity =\
                    check_linear_limit_velocity(self.target_linear_velocity - LIN_VEL_STEP_SIZE)
                #print_vels(self.target_linear_velocity, self.target_angular_velocity)
            elif key == 'a':
                self.target_angular_velocity =\
                    check_angular_limit_velocity(self.target_angular_velocity + ANG_VEL_STEP_SIZE)
                #print_vels(self.target_linear_velocity, self.target_angular_velocity)
            elif key == 'd':
                self.target_angular_velocity =\
                    check_angular_limit_velocity(self.target_angular_velocity - ANG_VEL_STEP_SIZE)
                #print_vels(self.target_linear_velocity, self.target_angular_velocity)
            elif key == ' ':
                self.target_linear_velocity   = 0.0
                self.control_linear_velocity  = 0.0
                self.target_angular_velocity  = 0.0
                self.control_angular_velocity = 0.0
                #print_vels(self.target_linear_velocity, self.target_angular_velocity)

            elif key == '+':
                #self.get_logger().info("Switching to autonomous mode. Type 'cancel' to exit.")
                inp = input("Enter x-coordinate of target: ")
                if inp == 'cancel':
                    self.get_logger().info("Cancelled switch to autonomous mode.")
                    return
                self.target[0] = float(inp)
                self.target[1] = float(input("Enter y-coordinate of target: "))
                cmd.data = '1, {}, {}, 0, 0'.format(self.target[0], self.target[1])
                self.auto_driving_instruction.publish(cmd)
                self.get_logger().info("Autonomous driving mode. Target: ({:.3f}, {:.3f})."\
                    .format(self.target[0], self.target[1]))
                self.get_logger().info("Press '0' to terminate autonomous driving mode.")
                return

            #twist = Twist()

            self.control_linear_velocity = make_simple_profile(
                self.control_linear_velocity,
                self.target_linear_velocity,
                (LIN_VEL_STEP_SIZE/2.0))

            self.twist.linear.x = self.control_linear_velocity
            self.twist.linear.y = 0.0
            self.twist.linear.z = 0.0

            self.control_angular_velocity = make_simple_profile(
                self.control_angular_velocity,
                self.target_angular_velocity,
                (ANG_VEL_STEP_SIZE/2.0))

            self.twist.angular.x = 0.0
            self.twist.angular.y = 0.0
            self.twist.angular.z = self.control_angular_velocity

            self.cmd_vel_pub.publish(self.twist)

def main(args=None):
    print("Hello, this is controller")
    print("Use W,A,S,D to move the robot")
    print("Press key '+' to enter autonomous driving mode\n")

    rclpy.init(args=args)
    controller_publisher = ControllerPublisher()
    rclpy.spin(controller_publisher)

    controller_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()