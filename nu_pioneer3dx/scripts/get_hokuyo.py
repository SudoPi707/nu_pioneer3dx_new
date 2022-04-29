import rospy
from sensor_msgs.msg import LaserScan


class SensorValue():

    def __init__(self):

        self.sensor_value = {}
        rospy.Subscriber("/scan", LaserScan, self.get_sensor_value)

    def get_sensor_value(self, laser):

        # laser.ranges[x]; x = 512 / 180 * (c+90) & c = desired degree

        self.sensor_value[90] = laser.ranges[511]
        self.sensor_value[50] = laser.ranges[398]
        self.sensor_value[30] = laser.ranges[341]
        self.sensor_value[10] = laser.ranges[284]
        self.sensor_value[-10] = laser.ranges[228]
        self.sensor_value[-30] = laser.ranges[171]
        self.sensor_value[-50] = laser.ranges[114]
        self.sensor_value[-90] = laser.ranges[0]


if __name__ == "__main__":

    rospy.init_node("laser_value")

    program = SensorValue()

    while True:

        rospy.sleep(0.1)
