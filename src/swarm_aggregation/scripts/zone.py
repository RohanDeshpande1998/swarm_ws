#!/usr/bin/env python3
import rospy 
import statistics
from geometry_msgs.msg import Point, Twist, Pose2D
from sensor_msgs.msg import LaserScan
import numpy as np
from collections import deque
from math import * 
from swarm_aggregation.msg import botPose
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import random

# Global Definitions
X = 0
Y = 1
MINIMUM_NEIGHBOURS = 1
GOAL_RESET_TIME = 40 ##40sec
class Workspace:
    def __init__(self, no_of_bots):
        #Danger Zone square
        self.sqr_1_x_min = -7
        self.sqr_1_x_max = -0.5
        self.sqr_1_y_min = -3
        self.sqr_1_y_max = 3
        
        #Safe Zone square
        self.sqr_2_x_min = 1.5
        self.sqr_2_x_max = 7.5
        self.sqr_2_y_min = -5.5
        self.sqr_2_y_max = 0
        self.goal = [None, None] 
        self.lattice_data = [] # Array containing list of (lattice_centroid, radius)
        self.obstacle_data = [] # Array containing list of (obstacle_coordinates, diameter)
        self.obstacle_coordinates = []
        self.agents_location = [None]*no_of_bots
        
    def update_agent_location(self, id, bot_position):
        self.agents_location[id] = bot_position
    
    def update_lattice_data(self, lattice_centroids, radii): ##Very inefficient
        self.lattice_data = []
        for centroid in lattice_centroids:
            self.lattice_data.append([centroid, radii])
    
    def update_obstacle_data(self, obstacle_coordinates_data,diameter):
        self.obstacle_data.append([obstacle_coordinates_data, diameter])
        self.obstacle_coordinates.append(obstacle_coordinates_data)

    def get_robot_id_at(self, coordinate):
        for robot_id, location in enumerate(self.agents_location):
            if self.is_within_tolerance(coordinate, location, 0.5):  # Use the same tolerance as detection
                return robot_id
        return None
    
    def is_within_tolerance(self, new_point, obstacle, tolerance):
        distance = dist(new_point, obstacle)  # Use Euclidean distance
        if distance < tolerance:
            return True  # The point is too close, treat it as a duplicate
        return False  # The point is far enough to be considered unique

    def generate_goal(self, goal_coordinates = [None,None]):
        """
        Generate a random point within a 2D rectangle defined by
        (x_min, x_max) for x-coordinates and (y_min, y_max) for y-coordinates.
        If goal point not provided
        """
        if random.randint(0,1):
            x_min = self.sqr_1_x_min
            x_max = self.sqr_1_x_max
            y_min = self.sqr_1_y_min
            y_max = self.sqr_1_y_max
        else:
            x_min = self.sqr_2_x_min
            x_max = self.sqr_2_x_max
            y_min = self.sqr_2_y_min
            y_max = self.sqr_2_y_max
        while True:
            if all(coord is not None for coord in goal_coordinates):
                # Both goal_coordinates are provided
                x = goal_coordinates[X]
                y = goal_coordinates[Y]
                return x,y
            else:
                x = random.uniform(x_min, x_max)
                y = random.uniform(y_min, y_max)
                if self.validity_of_goal([x,y]):
                    return x,y
                else:
                    print("invalid goal: ", x,y)
                    goal_coordinates = [None, None]
        
    def validity_of_goal(self, goal):
        if len(self.lattice_data) != 0:    
            for lattice_no in range(len(self.lattice_data)):
                if dist([goal[X], goal[Y]], self.lattice_data[lattice_no][0]) < self.lattice_data[lattice_no][1]:
                    return False
        for obstacle in self.obstacle_data:
            if dist([goal[X], goal[Y]], obstacle[0]) < obstacle[1]:
                return False
        return True
        
# class Lattice:
#     def __init__(self):
#         self.points = []  # List of previously received points
#         self.lattice_length = 1.5
#         self.distance_tolerance = 0.2
#         self.pseudo_lattice = []
#         self.lattice_centroid = []
             
#     def process_new_data_point(self, obstacle_2d_data):
#         if (len(self.pseudo_lattice)>0):
#             self.check_pseudo_lattice_match(obstacle_2d_data, 0.2)
#         self.cluster_points(obstacle_2d_data)
    
#     def check_pseudo_lattice_match(self, obstacle_2d_data, error):
#         for cluster in self.pseudo_lattice:
#             for point in cluster[2:]:
#                 if (abs(dist(point, obstacle_2d_data)) <= error):
#                     print("Badhai ho lattice mil gya!")
#                     lattice_cluster = [False, cluster[0], cluster[1], point]
#                     print("Lattice Cluster formed: " ,lattice_cluster)
#                     self.form_triangle_lattice(lattice_cluster)
#                     # print (lattice_cluster)
#                     return True
#         return False
             
#     def cluster_points(self, obstacle_2d_data):
#         for existing_point in self.points:
#             if abs(dist(existing_point, obstacle_2d_data) - self.lattice_length) <= self.distance_tolerance:
#                 self.form_pseudo_triangle_lattice([existing_point, obstacle_2d_data])
        
#         self.points.append(obstacle_2d_data)

    
#     def form_triangle_lattice(self, cluster):
#         centroid_of_lattice = np.mean(cluster[1:], axis=0)
#         self.lattice_centroid.append(centroid_of_lattice.tolist())
    
#     def form_pseudo_triangle_lattice(self, cluster):
#         X_1, Y_1 = cluster[0][X], cluster[0][Y]
#         X_2, Y_2 = cluster[1][X], cluster[1][Y]
#         third_point_of_lattice_ccw = (
#             X_1 + cos(radians(60))*(X_2 - X_1) - sin(radians(60))*(Y_2 - Y_1), 
#             Y_1 + sin(radians(60))*(X_2 - X_1) + cos(radians(60))*(Y_2 - Y_1)
#         )
#         third_point_of_lattice_cw = (
#             X_1 + cos(radians(-60))*(X_2 - X_1) - sin(radians(-60))*(Y_2 - Y_1), 
#             Y_1 + sin(radians(-60))*(X_2 - X_1) + cos(radians(-60))*(Y_2 - Y_1)
#             )
#         self.pseudo_lattice.append([cluster[0], cluster[1], third_point_of_lattice_ccw, third_point_of_lattice_cw])

class Lattice:
    def __init__(self):
        self.points = np.empty((0, 2))
        self.lattice_length = 1.5
        self.distance_tolerance = 0.2
        self.pseudo_lattice = np.empty((0, 4, 2))
        self.lattice_centroid = np.empty((0, 2))
             
    def process_new_data_point(self, obstacle_2d_data):
        if len(self.pseudo_lattice) > 0:
            self.check_pseudo_lattice_match(obstacle_2d_data, 0.2)
        self.cluster_points(obstacle_2d_data)
        self.trim_data()
    
    def check_pseudo_lattice_match(self, obstacle_2d_data, error):
        for cluster in self.pseudo_lattice:
            points_array = np.array(cluster[2:])
            distances = np.linalg.norm(points_array - obstacle_2d_data, axis=1)
            if np.any(distances <= error):
                print("Badhai ho lattice mil gya!")
                lattice_cluster = [False, cluster[0], cluster[1], points_array[np.argmin(distances)]]
                print("Lattice Cluster formed: ", lattice_cluster)
                self.form_triangle_lattice(lattice_cluster)
                return True
        return False
             
    def cluster_points(self, obstacle_2d_data):
        if len(self.points) > 0:
            distances = np.linalg.norm(self.points - obstacle_2d_data, axis=1)
            close_points = np.where(np.abs(distances - self.lattice_length) <= self.distance_tolerance)[0]
            for idx in close_points:
                self.form_pseudo_triangle_lattice([self.points[idx], obstacle_2d_data])
        self.points = np.vstack((self.points, obstacle_2d_data))
    
    def form_triangle_lattice(self, cluster):
        centroid_of_lattice = np.mean(np.array(cluster[1:]), axis=0)
        self.lattice_centroid = np.vstack((self.lattice_centroid, centroid_of_lattice))
    
    def form_pseudo_triangle_lattice(self, cluster):
        vec = np.array(cluster[1]) - np.array(cluster[0])
        rotation_matrix_ccw = np.array([[np.cos(np.radians(60)), -np.sin(np.radians(60))],
                                         [np.sin(np.radians(60)), np.cos(np.radians(60))]])
        rotation_matrix_cw = np.array([[np.cos(np.radians(-60)), -np.sin(np.radians(-60))],
                                        [np.sin(np.radians(-60)), np.cos(np.radians(-60))]])
        
        third_point_ccw = np.array(cluster[0]) + np.dot(rotation_matrix_ccw, vec)
        third_point_cw = np.array(cluster[0]) + np.dot(rotation_matrix_cw, vec)
        new_data = np.expand_dims([cluster[0], cluster[1], third_point_ccw, third_point_cw], axis=0)
        self.pseudo_lattice = np.vstack((self.pseudo_lattice, new_data))
    
    def trim_data(self):
        max_points = 10
        max_pseudo_lattices = 10

        if len(self.points) > max_points:
            self.points = self.points[-max_points:]
        
        if len(self.pseudo_lattice) > max_pseudo_lattices:
            self.pseudo_lattice = self.pseudo_lattice[-max_pseudo_lattices:]

    def print_data(self):
        print("points: ",self.points)
        print("lattice centroids: ", self.lattice_centroid)
        print("pseudo lattice coordinates: ", self.pseudo_lattice)
                
class Robot:
    def __init__(self,no_of_bots):
        self.observed_lattice_obj = Lattice()
        self.robot_workspace = Workspace(no_of_bots)
        self.total_bots = no_of_bots
        self.neighbour_array = []
        self.x = 0
        self.y = 0        
        self.incident_time = []           
        self.yaw = 0
        self.range = [0]
        self.goal_start_time = 0
        self.current_time = 0
        self.angle = 0
        self.ang_max = 0
        self.ang_inc = 0
        self.disij = []
        self.delij = []             
        self.bearing = [0]
        self.neigh = 0
        self.robot = []          
        self.goal = [4, 0] #Initial goal
        self.goal_set = False #Set this to true if you want the agent(s) to go to initial self.goal
        self.namespace = rospy.get_namespace()
        scan_topic = self.namespace + "scan"
        self.speed = Twist()
        self.hist = deque(maxlen=20)
        self.odom_sub = rospy.Subscriber("/odom",Odometry,self.update_Odom)
        self.bot_data = rospy.Subscriber("/obs_data", botPose, self.get_other_bot_position) 
        self.object_detector = rospy.Subscriber(scan_topic,LaserScan, self.scanner)

        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)    
        self.pubg = rospy.Publisher('/goal', Point, queue_size=1)
        self.rad_pub = rospy.Publisher('/radius', Point, queue_size=1)
        # self.obs_pub = rospy.Publisher('/obs',obs, queue_size=1)    
    
    """LOCALIZATION"""
    def update_Odom(self,odom):
        """ Odometry of current bot"""       
        self.x = odom.pose.pose.position.x 
        self.y = odom.pose.pose.position.y
        self.rot_q = odom.pose.pose.orientation
        euler = euler_from_quaternion([self.rot_q.x , self.rot_q.y , self.rot_q.z , self.rot_q.w ])
        self.yaw = atan2(sin(euler[2]),cos(euler[2])) 
        self.odom = Pose2D(self.x,self.y,self.yaw)
        self.vel = Pose2D(odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.angular.z)

    
    """MAPPING""" 
    def get_other_bot_position(self, msg):
        bot_info = msg.botpose
        for i in range(len(msg.bot_id)):
            position = bot_info[i].pose.pose.position
            # print(i, position)
            self.robot_workspace.update_agent_location(i, [position.x, position.y])
                
    def scanner(self,msg):
        ranges_list = list(msg.ranges)
        dq = deque(ranges_list[0:180] + [inf]*179)
        dq.rotate(-90)  # code written for fov ranging from [0, 359] current range is [-90 90]
        msg.ranges = list(dq)
        self.range = msg.ranges
        min_distance = min(self.range)
        self.ang_max = msg.angle_max
        self.ang_inc = msg.angle_increment
        self.process_scanner_data(msg)
        # rospy.loginfo(f"Minimum distance to object: {min_distance} meters")
    
    def distance(self, point1, point2):
    # Convert numbers to single-element tuples
        point1_tuple = (point1,) if not isinstance(point1, (tuple, list)) else point1
        point2_tuple = (point2,) if not isinstance(point2, (tuple, list)) else point2
        return dist(point1_tuple, point2_tuple)

    def get_medians_and_edge_with_angle(self, data_points, epsilon, epsilon_angle):
        """
        Clusters data points based on value and angle thresholds.
        
        Parameters:
        - data_points: List of tuples (value, angle).
        - epsilon: Threshold for clustering values.
        - epsilon_angle: Threshold for clustering angles.
        """
        clusters = []
        current_cluster = []
        
        # Step 1: Form clusters based on the distance threshold for both value and angle
        for current_data_point in data_points:
            value, angle = current_data_point
            
            if not current_cluster:
                current_cluster.append(current_data_point)
            else:
                prev_value, prev_angle = current_cluster[-1]
                
                # Check if both value and angle differences are within the threshold
                if (self.distance(value, prev_value) <= epsilon and 
                    abs(angle - prev_angle) <= epsilon_angle):
                    current_cluster.append(current_data_point)
                else:
                    # Save the completed cluster and start a new one
                    clusters.append(current_cluster)
                    current_cluster = [current_data_point]

        # Add the last cluster if any
        if current_cluster:
            clusters.append(current_cluster)
        
        # Remove clusters with fewer than 2 elements to eliminate false positives
        clusters = [cluster for cluster in clusters if len(cluster) >= 2]

        # Step 3: Calculate medians and edge points of each cluster
        median_array = []
        edge_points_array = []
        
        for cluster in clusters:
            # Extract values and angles separately
            values = [point[0] for point in cluster]
            # angles = [point[1] for point in cluster]
            
            # Calculate medians for both value and angle
            median_value = statistics.median_low(values)
            # median_angle = statistics.median_low(angles)
            median_array.append(median_value)
            
            # Determine the edge point in terms of value distance from the median
            edge_point = max(cluster, key=lambda point: abs(point[0] - median_value))
            edge_points_array.append(edge_point)
        
        return median_array, edge_points_array
        
    def is_within_tolerance(self, new_point, obstacles, tolerance):
        if len(obstacles) == 0:
            return False
        # Check if any point in obstacles is within tolerance of new_point
        else:
            for existing_point in obstacles:
                distance = dist(new_point, existing_point)  # Use Euclidean distance
                if distance < tolerance:
                    return True  # The point is too close, treat it as a duplicate
            return False  # The point is far enough to be considered unique
    
    def wrap_around(self, value, lower_bound, upper_bound):
        """
        Wraps a value around if it goes out of the specified bounds.

        Args:
            value (int): The value to wrap around.
            lower_bound (int): The inclusive lower bound of the range.
            upper_bound (int): The exclusive upper bound of the range.

        Returns:
            int: The wrapped value within the specified bounds.
        """
        range_size = upper_bound - lower_bound
        return (value - lower_bound) % range_size + lower_bound

    def process_scanner_data(self,msg):
        # Threshold limit to consider as an obstacle (in meters)
        R_max = 2
        obs_size_limit = 0.15
        distances = []
        angles = []
        distance_angle_pairs = []
        obstacles_found = False
        self.neighbour_array = []
        # Iterate through laser scan ranges and print angles where distance < limit
        for angle, distance in enumerate(msg.ranges):
            # adjusted_angle = self.wrap_around(angle - 180, 0, 360)
            if distance < R_max:
                obstacles_found = True
                distances.append(distance)
                angles.append(angle)
                distance_angle_pairs.append((round(distance, 4), angle))
          
        ##If obstacles are found then clean the data, check it with previous data of obstacles and compare it with other agent positions to prevent false alarms 
        # if (self.namespace == "/tb3_1/"):
        # print("Length of message:",len(msg.ranges));
        # non_inf_count = sum(1 for x in msg.ranges if not isinf(x))
        # print("Length of messages without inf:", non_inf_count)
        # print(msg.ranges)

        if (obstacles_found):
            rounded_distances = np.round(distances, 4).tolist()
            median_obstacle_distance_list, edge_points_list = self.get_medians_and_edge_with_angle(distance_angle_pairs, 0.1,2)
            median_obstacle_distance_indices_list = [rounded_distances.index(val) for val in median_obstacle_distance_list]
            edge_points_indices_list = [rounded_distances.index(val[0]) for val in edge_points_list]
            for enum, index in enumerate(median_obstacle_distance_indices_list):
                if ((rounded_distances[index]*tan(radians(abs(angles[index] - angles[edge_points_indices_list[enum]])))) < obs_size_limit):
                    obstacle_x = self.odom.x + rounded_distances[index]*cos(radians(angles[index]) + self.yaw)
                    obstacle_y = self.odom.y + rounded_distances[index]*sin(radians(angles[index]) + self.yaw)
                    obstacle_coordinate = (round(obstacle_x,3), round(obstacle_y,3)) 
                    
                    if not self.is_within_tolerance(obstacle_coordinate, self.robot_workspace.agents_location, 0.3):
                        if not self.is_within_tolerance(obstacle_coordinate, self.robot_workspace.obstacle_coordinates, 0.3): 
                                ##Is a new obstacle observed?
                                # print(obstacle_coordinate)
                                # print(self.robot_workspace.agents_location)
                                self.observed_lattice_obj.process_new_data_point(obstacle_coordinate)
                                self.robot_workspace.update_obstacle_data(obstacle_coordinate, obs_size_limit)
                                default_lattice_radius = self.observed_lattice_obj.lattice_length/sqrt(3)
                                self.robot_workspace.update_lattice_data(self.observed_lattice_obj.lattice_centroid, default_lattice_radius)
                                if all(coord is not None for coord in self.goal):
                                    if self.robot_workspace.validity_of_goal(self.goal):
                                        self.robot_workspace.generate_goal()
                                # print(distance_angle_pairs)
                                # print("\n")
                                # print(obstacle_coordinate)
                                # print(self.robot_workspace.agents_location)
                    else:
                        # Check and print the robot ID
                        robot_id = self.robot_workspace.get_robot_id_at(obstacle_coordinate)
                        if robot_id is not None:
                            # print(f"Ayyyyyyy robot detected! Robot ID: {robot_id} and I am: {self.namespace}")
                            # print("Neighbour at:",self.robot_workspace.agents_location[robot_id])
                            if not self.is_within_tolerance(self.robot_workspace.agents_location[robot_id], self.neighbour_array, 0.3):
                                self.neighbour_array.append(self.robot_workspace.agents_location[robot_id])
                        else:
                            print("Ayyyyyyy robot detected, but ID not found!")
                    # print("Obstacle_coordinates:", self.obstacle_coordinates)


    """NAVIGATION"""
    def compute_avoidance_velocity(self, robot_position, robot_yaw, avoidance_radius, linear_velocity, angular_velocity, max_angular_velocity=1.0):
        """
        Adjusts the robot's velocity to avoid clusters based on proximity.
        """
        repulsion_vector = np.array([0.0, 0.0])
        for centroid,radius in self.robot_workspace.lattice_data:
            # Calculate distance from the robot to the cluster centroid
            distance_to_centroid = dist(robot_position, centroid)
            
            if distance_to_centroid < avoidance_radius:
                # Calculate the repulsive force (inverse distance weighting)
                direction_away = np.subtract(robot_position, centroid)
                normalized_direction = direction_away / np.linalg.norm(direction_away)
                repulsion_vector += normalized_direction * (avoidance_radius - distance_to_centroid)
        # If there is a repulsion vector, adjust velocities
        if np.linalg.norm(repulsion_vector) > 0:
            # Calculate desired angle to move away from cluster
            target_angle = atan2(repulsion_vector[1], repulsion_vector[0])
            angle_diff = target_angle - robot_yaw

            # Normalize angle difference to the range [-pi, pi]
            angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi

            # Adjust angular velocity to steer away
            adjusted_angular_velocity = np.clip(angle_diff * 2.0, -max_angular_velocity, max_angular_velocity)

            # Adjust linear velocity based on proximity
            if isinstance(linear_velocity, (list, tuple, np.ndarray)):
                # Handle vector linear velocity (e.g., [vx, vy])
                adjusted_linear_velocity = [
                    v * max(0, 1 - (np.linalg.norm(repulsion_vector) / avoidance_radius))
                    for v in linear_velocity
                ]
            else:
                # Handle scalar linear velocity
                adjusted_linear_velocity = linear_velocity * max(0, 1 - (np.linalg.norm(repulsion_vector) / avoidance_radius))
            return adjusted_linear_velocity, adjusted_angular_velocity
        # No obstacles nearby, proceed with given velocities
        return linear_velocity, angular_velocity
        
    def navigate_near_obstacle(self, robot_position):
        """funct to follow wall boundary
        Turn Right by default or rotate on CCW fashion"""
        # print("Wall following")
        deg = 30
        dst = 1.5
        avoidance_radius = 1
        
        closest_distance = float('inf')
        
        for centroid,radius in self.robot_workspace.lattice_data:
            distance = dist(centroid, robot_position)
            if distance < closest_distance:
                closest_distance = distance
        
        if (closest_distance > avoidance_radius):
            if min(self.range[0:deg]) <= dst or min(self.range[(360-deg):]) <= dst: # front wall
                self.speed.angular.z = -0.2
                self.speed.linear.x = 0.0
                # print("Front wall observed by", self.namespace)
            elif min(self.range[deg:89]) < dst: # left wall 
                self.speed.angular.z = 0.0
                self.speed.linear.x = 0.2
                # print(self.range)
                # print("Left wall observed by", self.namespace)
            # elif min(self.range[(359-120):(359-deg)]) < dst: # right wall
            #     self.speed.angular.z = 0.0
            #     self.speed.linear.x = -0.2
            else:
                robot_speed, robot_angular_velocity = self.compute_avoidance_velocity(
                    (self.odom.x, self.odom.y),
                    (self.yaw),
                    avoidance_radius,
                    (self.speed.linear.x, self.speed.linear.y),
                    (self.speed.angular.z)
                )
                self.speed.linear.x = robot_speed[0]
                self.speed.linear.y = robot_speed[1]
                self.speed.angular.z = robot_angular_velocity
        else:
            robot_speed, robot_angular_velocity = self.compute_avoidance_velocity(
                (self.odom.x, self.odom.y),
                (self.yaw),
                avoidance_radius,
                (self.speed.linear.x, self.speed.linear.y),
                (self.speed.angular.z)
            )
            self.speed.linear.x = robot_speed[0]
            self.speed.linear.y = robot_speed[1]
            self.speed.angular.z = robot_angular_velocity
    
    def set_goal(self, goal_coordinates = [None,None]):
        """write code for identification based goal update of robot"""
        """If robot -> update goal
        If obstacle -> wall following"""
        self.goal_start_time = self.current_time
        self.goal = self.robot_workspace.generate_goal(goal_coordinates)
        self.goal_set = True

        
    def controller(self,k):
        # print(self.robot_workspace.obstacle_coordinates, "for robot:", self.namespace, "for timestamp: ", k)

        # print("My name:",self.namespace)
        # print("Things around me are at:", self.neighbour_array)
        # print("My location:", self.x,self.y)
        # print("My goal:", self.goal, "\n")
        
        # print(self.neighbour_array)
        # if self.namespace == "/tb3_3/":
            # print("For agent: ", self.namespace)
            # print("Lattice data captured:",self.observed_lattice_obj.lattice_centroid)
            # print("Pseudo Lattice data captured:",self.observed_lattice_obj.pseudo_lattice)
            # print(self.goal)
            # print("\n")
        obstacle_nearby = False
        robot_position = [self.odom.x, self.odom.y]
        obstacle_avoidance_distance = 2 #2meters
        for obstacle_coordinate in self.robot_workspace.obstacle_coordinates:
            if dist(obstacle_coordinate, robot_position) < obstacle_avoidance_distance:
                obstacle_nearby = True

        
        if len(self.neighbour_array) == 0 or obstacle_nearby:    
            if not self.goal_set:
                self.set_goal()
                # print("Random Goal set:", self.goal, "for robot: ", self.namespace)
        else:
            x = self.x
            y = self.y
            for coords in self.neighbour_array:
                x += coords[X]
                y += coords[Y]
            
            # print(x,y)
            cluster_goal = [x/(len(self.neighbour_array)+1), y/(len(self.neighbour_array) + 1)]
            # print(cluster_goal)
            self.set_goal(cluster_goal)
            self.goal_set = False ##Since this is a tentative goal
            
            # print("Neighbour array:", self.neighbour_array)
            # print("Cluster goal set:", self.goal, "\n")
            # self.goal_set = True
            

        self.incx = (self.goal[X] - self.x)
        self.incy = (self.goal[Y] - self.y)
        # Bearing of bot
        self.bearing.append(atan2(self.incy,self.incx))

        # Distance Error
        self.dis_err = (sqrt(self.incx**2+self.incy**2))
        
        # Gradient of Bearing
        self.dtheta = (self.bearing[k] - self.bearing[k-1])/h
        
        if (self.dis_err) <= 0.90:
            self.goal_set = False
            if len(self.neighbour_array)>=MINIMUM_NEIGHBOURS:
                self.goal = [None, None]
            else:
                self.set_goal()
        # print("Agent Name:", self.namespace)
        # print("Goal:", self.goal)
        if all(coord is not None for coord in self.goal):
            # print("Moving towards goal")
            self.speed.linear.x = 0.18
            self.speed.angular.z = K*np.sign(self.dtheta)
            
            for obstacle_coordinate in self.robot_workspace.obstacle_coordinates:
                if dist(obstacle_coordinate, robot_position) < obstacle_avoidance_distance:
                    self.navigate_near_obstacle(robot_position)    
        else:
            self.speed.linear.x = 0
            self.speed.linear.y = 0
            self.speed.angular.z = 0
        
        self.cmd_vel.publish(self.speed)
        point = Point()
        point.x = self.goal[X]
        point.y = self.goal[Y]
        point.z = 0        
        self.pubg.publish(point)

## If the robot has not formed a cluster and is unable to reach the goal position in the GOAL RESET TIME then resets the Goal
    def goal_reset(self, time):
        self.current_time = time
        if all(coord is not None for coord in self.goal):
            if (self.current_time - self.goal_start_time > GOAL_RESET_TIME):
                print("Goal before:", self.goal)
                self.set_goal()
                print("Goal reset for robot:",self.namespace)
                print("Goal after:",self.goal)


if __name__ == '__main__':
    rospy.init_node("obstacle_controller")
    rospy.loginfo("Chal Gye badde")
    k = 0
    l = [] #l is time
    rate = rospy.Rate(4)
    bot = Robot(6)  
    rospy.sleep(6)
    # bot.set_goal()
    while not rospy.is_shutdown() and k < 500000:
        k = k+1
        h = 0.25
        K = 0.3
        l.append((k+1)/10) # Time
        bot.controller(k)
        bot.goal_reset(l[-1])
        rate.sleep()
    # observed_lattice_obj = Lattice()
    # # First test point, will start forming a cluster
    # obstacle_2d_data = (1, 2)  
    # observed_lattice_obj.process_new_data_point(obstacle_2d_data)
    # print("Clusters after 1st point:", observed_lattice_obj.clusters)

    # # Second test point, will form a pseudo lattice as there's only one point in pseudo_lattice
    # obstacle_2d_data = (4, 2)  
    # observed_lattice_obj.process_new_data_point(obstacle_2d_data)
    # print("Clusters after 2nd point:", observed_lattice_obj.clusters)
    
    # # Third test point, will check if it matches with the pseudo lattice
    # obstacle_2d_data = (2.5, 4.6)  # This should be close to the second point, and should trigger the lattice formation
    # observed_lattice_obj.process_new_data_point(obstacle_2d_data)
    # print("Clusters after 3rd point:", observed_lattice_obj.clusters)
    
    # # Fourth test point, which should not match the existing pseudo lattice, so a new cluster will form
    # obstacle_2d_data = (5, 5)  
    # observed_lattice_obj.process_new_data_point(obstacle_2d_data)
    # print("Clusters after 4th point:", observed_lattice_obj.clusters)
    
    # observed_lattice_obj.print_data("acjks")