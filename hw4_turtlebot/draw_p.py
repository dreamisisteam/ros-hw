#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

LINEAR_VELOCITY = 0.14
ANGULAR_VELOCITY = 0.52

VERTICAL_TIMEOUT = 10
HORIZONTAL_TIMEOUT = 7
ROTATE_TIMEOUT = 3
INIT_TIMEOUT = 1


def move_robot(
    linear_speed: float,
    angular_speed: float,
    *,
    sec: int = 0,
):
    """Перемещение робота по заданной траектории."""
    twist = Twist()

    twist.linear.x = linear_speed
    twist.angular.z = angular_speed

    rospy.loginfo(f'Движение: linear_speed = {linear_speed} | angular_speed = {angular_speed}')

    cmd_vel_pub.publish(twist)

    rospy.sleep(sec)


if __name__ == '__main__':
    rospy.init_node('draw_P', anonymous=True)

    global cmd_vel_pub
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    move_robot(0, 0, sec=INIT_TIMEOUT)

    move_robot(LINEAR_VELOCITY, 0, sec=VERTICAL_TIMEOUT)
    move_robot(0, ANGULAR_VELOCITY, sec=ROTATE_TIMEOUT)
    move_robot(LINEAR_VELOCITY, 0, sec=HORIZONTAL_TIMEOUT)
    move_robot(0, ANGULAR_VELOCITY, sec=ROTATE_TIMEOUT)
    move_robot(LINEAR_VELOCITY, 0, sec=VERTICAL_TIMEOUT)

    move_robot(0, 0, sec=INIT_TIMEOUT)
