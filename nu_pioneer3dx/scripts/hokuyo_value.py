import rospy
from sensor_msgs.msg import LaserScan


class LaserValue():

    def __init__(self):

        self.laser_value = {}
        rospy.Subscriber("/scan", LaserScan, self.get_laser_value)

    def get_laser_value(self, laser):

        global laser_list

        # hokuyo laser scale = 512 / 180 * (x+90); x = degree

        self.laser_value[90] = laser.ranges[511]
        self.laser_value[50] = laser.ranges[398]
        self.laser_value[30] = laser.ranges[341]
        self.laser_value[10] = laser.ranges[284]
        self.laser_value[-10] = laser.ranges[228]
        self.laser_value[-30] = laser.ranges[171]
        self.laser_value[-50] = laser.ranges[114]
        self.laser_value[-90] = laser.ranges[0]


if __name__ == "__main__":

    rospy.init_node("laser_value")

    program = LaserValue()

    laser_list = []

    while True:

        rospy.sleep(0.1)
