import rospy
import math

from robot_position import RobotPosition
from geometry_msgs.msg import Twist, Vector3


class LinearVector():

    def __init__(self):

        self.max_xy_speed = 0.5
        self.max_rot_speed = 0.5
        self.min_turn_speed = 0.00  # Turn is the speed of x according to z

        # The lesser, the rotate is much faster but never exceed the rot speed
        self.rot_rate = 0.3
        self.speed_rate = 0.5
        self.turn_rate = 2
        self.xy_delay = 0.5
        self.rot_delay = 2
        self.turn_delay = 3

        self.distance_torelance = 0.5
        self.yaw_torelance = math.pi / 16

        self.twist_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    def get_vector(self, robot_pos, goal):

        diff_x = goal["x"] - robot_pos.robot_x
        diff_y = goal["y"] - robot_pos.robot_y

        magnitude = math.sqrt(math.pow(diff_x, 2) + math.pow(diff_y, 2))

        target_yaw = math.atan2(diff_y, diff_x)

        target_degree = target_yaw * 180 / math.pi

        if target_degree < 0:

            target_degree += 360

        target_yaw = target_degree * math.pi / 180

        return magnitude, target_yaw

    def order_robot(self, obs_magnitude, goal_magnitude, final_yaw):

        if goal_magnitude > self.distance_torelance:

            goal_sigmoid = 1 / \
                (1 + math.exp(-(goal_magnitude / self.speed_rate - self.xy_delay)))
            rot_sigmoid = 1 / \
                (1 + math.exp(-(abs(final_yaw) / self.rot_rate - self.rot_delay)))
            turn_sigmoid = 1 / (1 + math.exp(self.turn_rate * abs(obs_magnitude) - self.turn_delay))

            yaw_sign = final_yaw / abs(final_yaw)  # return 1 or -1

            vel_x = self.max_xy_speed * goal_sigmoid

            if abs(final_yaw) > self.yaw_torelance:  # If yaw is greater than yaw torelance

                vel_x = (vel_x - self.min_turn_speed) *  turn_sigmoid + self.min_turn_speed

                rot_z = yaw_sign * self.max_rot_speed * rot_sigmoid

            else:   # If yaw is less than yaw torelance, but not reached the goal yet.
                vel_x = vel_x
                rot_z = 0
        else:
            vel_x = 0
            rot_z = 0

            twist = Twist(Vector3(vel_x, 0, 0), Vector3(0, 0, rot_z))
            self.twist_pub.publish(twist)

            return True

        # print(vel_x)

        twist = Twist(Vector3(vel_x, 0, 0), Vector3(0, 0, rot_z))
        self.twist_pub.publish(twist)

        return False

    def get_move_radian(self, target_yaw, robot_pos):

        new_target_yaw = target_yaw - robot_pos.robot_yaw

        if new_target_yaw < 0:

            new_target_yaw += 2 * math.pi

        left_yaw = new_target_yaw
        right_yaw = (2 * math.pi) - new_target_yaw

        if left_yaw <= right_yaw:
            final_yaw = left_yaw
        else:
            final_yaw = -1 * right_yaw

        return final_yaw


if __name__ == "__main__":

    rospy.init_node("linear_vector")

    program = LinearVector()

    robot_pos = RobotPosition()

    goal = {"x": 0, "y": 0, "yaw": math.pi}

    while True:

        magnitude, target_yaw = program.get_vector(robot_pos, goal)

        final_yaw = program.get_move_radian(target_yaw, robot_pos)

        finished = program.order_robot(magnitude, final_yaw)

        rospy.sleep(0.1)
