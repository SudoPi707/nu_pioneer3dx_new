import rospy

from sensor_msgs.msg import LaserScan


class SensorValue():

    def __init__(self):

        self.sensor_value = {}
        rospy.Subscriber("/scan", LaserScan, self.get_sensor_value)

    def get_sensor_value(self, laser):

        # For Real Situation
        self.sensor_value[90] = laser.ranges[540]
        self.sensor_value[50] = laser.ranges[460]
        self.sensor_value[30] = laser.ranges[420]
        self.sensor_value[10] = laser.ranges[380]
        self.sensor_value[-10] = laser.ranges[340]
        self.sensor_value[-30] = laser.ranges[300]
        self.sensor_value[-50] = laser.ranges[260]
        self.sensor_value[-90] = laser.ranges[180]


if __name__ == "__main__":

    rospy.init_node("sensor_value")

    program = SensorValue()

    while True:

        rospy.sleep(0.1)
