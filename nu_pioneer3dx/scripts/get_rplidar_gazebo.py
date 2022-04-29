import rospy

from sensor_msgs.msg import LaserScan


class SensorValue():

    def __init__(self):

        self.sensor_value = {}
        rospy.Subscriber("/scan", LaserScan, self.get_sensor_value)

    def get_sensor_value(self, laser):

        # For Gazebo
        self.sensor_value[90] = laser.ranges[90]
        self.sensor_value[50] = laser.ranges[50]
        self.sensor_value[30] = laser.ranges[30]
        self.sensor_value[10] = laser.ranges[10]
        self.sensor_value[-10] = laser.ranges[-10]
        self.sensor_value[-30] = laser.ranges[-30]
        self.sensor_value[-50] = laser.ranges[-50]
        self.sensor_value[-90] = laser.ranges[-90]


if __name__ == "__main__":

    rospy.init_node("sensor_value")

    program = LaserValue()

    while True:

        rospy.sleep(0.1)
