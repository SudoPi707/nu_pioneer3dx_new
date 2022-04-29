import math
import rospy
import sys

from robot_position import RobotPosition
from vector_calculation_waypoint import LinearVector
from make_plan import MakePlan


class ObstacleAvoidant:

    def __init__(self):

        self.obs_force_scale = 0.4  # The larger, the more distance from robot to obstacle
        self.global_percent = 0.1
        self.local_percent = 1 - self.global_percent
        self.obs_max_distance = 3

    def get_obstacle_vector(self, laser):

        obs_force_x = 0
        obs_force_y = 0

        for i in laser.sensor_value:

            force_x = force_y = 0

            if laser.sensor_value[i] <= self.obs_max_distance:

                force_yaw = (i * math.pi / 180) + robot.robot_yaw + math.pi

                obs_magnitude = self.obs_force_scale / laser.sensor_value[i]

                force_x = obs_magnitude * math.cos(force_yaw)
                force_y = obs_magnitude * math.sin(force_yaw)

            obs_force_x += force_x
            obs_force_y += force_y

        return obs_force_x, obs_force_y

    def compute(self, vector, laser, robot, goal):

        # goal_yaw is map_yaw
        goal_magnitude, goal_yaw = vector.get_vector(robot, goal)

        goal_magnitude_x = goal_magnitude * math.cos(goal_yaw)
        goal_magnitude_y = goal_magnitude * math.sin(goal_yaw)

        obs_magnitude_x, obs_magnitude_y = self.get_obstacle_vector(laser)

        obs_magnitude = math.sqrt(
            math.pow(obs_magnitude_x, 2) + math.pow(obs_magnitude_y, 2)) * self.local_percent

        summed_magnitude_x = (
            goal_magnitude_x * self.global_percent) + (obs_magnitude_x * self.local_percent)
        summed_magnitude_y = (
            goal_magnitude_y * self.global_percent) + (obs_magnitude_y * self.local_percent)

        summed_yaw = math.atan2(summed_magnitude_y, summed_magnitude_x)

        if summed_yaw < 0:
            summed_yaw += 2 * math.pi

        return obs_magnitude, goal_magnitude, summed_yaw


if __name__ == "__main__":

    if len(sys.argv) < 3:
        print("Please specify the laser type (rplidar / hokuyo) and world type (gazebo / real).")
    else:

        rospy.init_node("obstacle_avoidant")
        program = ObstacleAvoidant()
        robot = RobotPosition()
        vector = LinearVector()
        make_plan = MakePlan()

        laser_type = sys.argv[1]
	
	    # For gazebo world
        if sys.argv[2] == "gazebo":

            if sys.argv[1] == "rplidar":
                from get_rplidar_gazebo import SensorValue

            elif sys.argv[1] == "hokuyo":
                from get_hokuyo_gazebo import SensorValue

            else:
                print("No type of this laser!!")
                exit()

            # Common configuration for gazebo simulation
            program.obs_force_scale = 0.3
            program.global_percent = 0.5
            program.local_percent = 0.5
            program.obs_max_distance = 3
            vector.max_xy_speed = 0.5
            vector.max_rot_speed = 0.5
            vector.min_turn_speed = 0.00
            vector.turn_rate = 2
            vector.distance_torelance = 0.5

	    # For real world
        elif sys.argv[2] == "real":

	        # Common configuration for gazebo simulation
            program.obs_force_scale = 0.3
            program.global_percent = 0.1
            program.local_percent = 0.9
            program.obs_max_distance = 3
            vector.max_xy_speed = 0.4
            vector.max_rot_speed = 0.4
            vector.min_turn_speed = 0.00
            vector.turn_rate = 3
            vector.distance_torelance = 0.5

            if sys.argv[1] == "rplidar":
                from get_rplidar import SensorValue

            elif sys.argv[1] == "hokuyo":
                from get_hokuyo import SensorValue

            elif sys.argv[1] == "sonar":
                from get_sonar import SensorValue
                vector.max_xy_speed = 0.3
                vector.max_rot_speed = 0.4

            else:
                print("No type of this laser!!")
                exit()

        else:
            print("Invalid World Type")
            exit()

        laser = SensorValue()

        while True:

            print("Enter target coordinate: ")
            goal_x = float(input("x: "))
            goal_y = float(input("y: "))

            plan_array = make_plan.get_plan(goal_x, goal_y)

            step = 25
            waypoint = [plan_array[i] for i in range(step, len(plan_array), step)]
            waypoint.append(plan_array[len(plan_array)-1])

            is_final_point = False

            for i in range(len(waypoint)):

                if i == len(waypoint) - 1:
                    is_final_point = True

                pos_x = waypoint[i].pose.position.x
                pos_y = waypoint[i].pose.position.y

                goal = {"x": pos_x, "y": pos_y, "yaw": math.pi}

                print("going to waypoint{} at x: {:.2f}, y: {:.2f}".format(i+1, pos_x, pos_y))

                finished = False

                while not finished:
                
                    obs_magnitude, goal_magnitude, target_yaw = program.compute(
                        vector, laser, robot, goal)

                    final_yaw = vector.get_move_radian(target_yaw, robot)

                    finished = vector.order_robot(goal_magnitude, final_yaw, is_final_point)

                    rospy.sleep(0.1)

            print("Robot reach the goal!" + "\n")
            
