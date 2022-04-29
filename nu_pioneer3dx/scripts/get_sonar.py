import rospy
import math

from sensor_msgs.msg import PointCloud


class SensorValue():

    def __init__(self):

        self.sensor_value = {}
        rospy.Subscriber("/rosaria/sonar", PointCloud, self.get_sensor_value)

    def get_sensor_value(self, sonar):

        index = 0

        for i in [90, 50, 30, 10, -10, -30, -50, -90]:

            sonar_x = sonar.points[index].x
            sonar_y = sonar.points[index].y

            sonar_magnitude = math.sqrt(
                math.pow(sonar_x, 2) + math.pow(sonar_y, 2))

            self.sensor_value[i] = sonar_magnitude

            index += 1


if __name__ == "__main__":

    rospy.init_node("sonar_value")

    program = SensorValue()

    while True:

        print(program.sensor_value)

        rospy.sleep(0.1)
