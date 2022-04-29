import rospy
import math

from sensor_msgs.msg import PointCloud


class SonarValue():

    def __init__(self):

        self.laser_value = {}
        rospy.Subscriber("/rosaria/sonar", PointCloud, self.get_sonar_value)

    def get_sonar_value(self, sonar):

        index = 0

        for i in [90, 50, 30, 10, -10, -30, -50, -90]:

            sonar_x = sonar.points[index].x
            sonar_y = sonar.points[index].y

            sonar_magnitude = math.sqrt(math.pow(sonar_x, 2) + math.pow(sonar_y, 2))

            # if sonar_magnitude >= 5:
            #     sonar_magnitude = 0.1

            self.laser_value[i] = sonar_magnitude

            index += 1

        # print(sonar.points[3].x, sonar.points[3].y)


if __name__ == "__main__":

    rospy.init_node("sonar_value")

    program = SonarValue()

    while True:

        print(program.laser_value)

        rospy.sleep(0.1)
