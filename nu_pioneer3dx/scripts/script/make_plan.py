import rospy

from nav_msgs.srv import GetPlan
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PoseWithCovarianceStamped

class MakePlan():

    def __init__(self):
        self.robot_x = 0
        self.robot_y = 0

    def set_robot_pose(self, amcl_pose):
        self.robot_x = amcl_pose.pose.pose.position.x
        self.robot_y = amcl_pose.pose.pose.position.y

    def get_plan(self, target_x, target_y):
        rospy.wait_for_service("/move_base/NavfnROS/make_plan")

        try:
            make_plan = rospy.ServiceProxy("/move_base/NavfnROS/make_plan", GetPlan)

            amcl_sub = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.set_robot_pose)
            # rospy.sleep(0.1)
            amcl_sub.unregister()

            ctime = rospy.get_rostime()

            start = PoseStamped(Header(0, ctime, "map"),
                                Pose(Point(self.robot_x, self.robot_y, 0), 
                                    Quaternion(0,0,0,1)))

            goal = PoseStamped(Header(0, ctime, "map"),
                                Pose(Point(target_x, target_y, 0), 
                                    Quaternion(0,0,0,1)))

            tolerance = 0

            resp = make_plan(start, goal, tolerance)

            return resp.plan.poses

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

if __name__ == "__main__":

    rospy.init_node("make_plan")

    # program = MakePlan()
    # print(len(program.get_plan()))

    pass