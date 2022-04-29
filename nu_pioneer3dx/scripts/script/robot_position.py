import math
import rospy

from tf import transformations
from geometry_msgs.msg import PoseWithCovarianceStamped
# from nav_msgs.msg import Odometry


class RobotPosition():

    def __init__(self):

        self.robot_x = 0
        self.robot_y = 0
        self.robot_yaw = 0

        rospy.Subscriber(
            "/amcl_pose", PoseWithCovarianceStamped, self.get_robot_position)

        # rospy.Subscriber("/odom", Odometry, self.get_robot_position)

    def get_robot_position(self, amcl):

        self.robot_x = amcl.pose.pose.position.x
        self.robot_y = amcl.pose.pose.position.y

        qx = amcl.pose.pose.orientation.x
        qy = amcl.pose.pose.orientation.y
        qz = amcl.pose.pose.orientation.z
        qw = amcl.pose.pose.orientation.w

        robot_euler = transformations.euler_from_quaternion([qx, qy, qz, qw])
        robot_radian = robot_euler[2]

        if robot_radian < 0:

            robot_radian += 2 * math.pi

        self.robot_yaw = robot_radian


if __name__ == '__main__':

    rospy.init_node('robot_position')

    program = RobotPosition()

    while True:

        print("x: {}, y: {}, yaw: {}".format(
            program.robot_x, program.robot_y, program.robot_yaw))

        rospy.sleep(1)
