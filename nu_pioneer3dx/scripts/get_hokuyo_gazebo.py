import rospy
from sensor_msgs.msg import LaserScan


class SensorValue():

    def __init__(self):

        self.sensor_value = {}
        rospy.Subscriber("/scan", LaserScan, self.get_sensor_value)

    def get_sensor_value(self, laser):

	global x

	# Angle min is -120 and Angle max is 120
	# Length of array = 727

	# laser.ranges[x]; x = 727 / 240 * (c+120) & c = desired degree

        self.sensor_value[90] = laser.ranges[636]
        self.sensor_value[50] = laser.ranges[515]
        self.sensor_value[30] = laser.ranges[454]
        self.sensor_value[10] = laser.ranges[394]
        self.sensor_value[-10] = laser.ranges[333]
        self.sensor_value[-30] = laser.ranges[273]
        self.sensor_value[-50] = laser.ranges[212]
        self.sensor_value[-90] = laser.ranges[91]

	x = laser.ranges[394]


if __name__ == "__main__":

    rospy.init_node("laser_value")

    program = SensorValue()

    x = 0

    while True:

	print(x)

        rospy.sleep(0.1)
