#!/usr/bin/env python3
import rospy 
import statistics
from geometry_msgs.msg import Point, Twist, Pose2D
from sensor_msgs.msg import LaserScan
import numpy as np
from sklearn import cluster
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

class Workspace:
    def __init__(self, no_of_bots, x_min, x_max, y_min, y_max):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.goal = [None, None] 
        self.lattice_data = [] # Array containing list of (lattice_centroid, radius)
        self.obstacle_data = [] # Array contaonong list of (obstacle_coordinates, diameter)
        self.agents_location = [None]*no_of_bots
        
    def update_agent_location(self, id, bot_position):
        self.agents_location[id] = bot_position
    
    def update_lattice_data(self, lattice_centroids, radii): ##Very inefficient
        self.lattice_data = []
        for centroid in lattice_centroids:
            self.lattice_data.append([centroid, radii])
    
    def update_obstacle_data(self, obstacle_coordinates,diameter):
        self.obstacle_data.append([obstacle_coordinates, diameter])

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
        while True:
            if all(coord is not None for coord in goal_coordinates):
                # Both goal_coordinates are provided
                x = goal_coordinates[X]
                y = goal_coordinates[Y]
                return x,y
            else:
                x = random.uniform(self.x_min, self.x_max)
                y = random.uniform(self.y_min, self.y_max)
            if self.validity_of_goal([x,y]):
                return x,y
            # else:
                # print("invalid goal")
        
    def validity_of_goal(self, goal):
        for lattice_no in range(len(self.lattice_data)):
            if dist([goal[X], goal[Y]], self.lattice_data[lattice_no][0]) < self.lattice_data[lattice_no][1]:
                return False
        for obstacle in self.obstacle_data:
            if dist([goal[X], goal[Y]], obstacle[0]) < obstacle[1]:
                return False
        return True
        
class Lattice:
    def __init__(self):
        self.clusters = []
        self.pseudo_lattice = []
        self.lattice_centroid = []
        self.lattice_length = 1.3211
        self.length_error = 0.4
        
        
    def process_new_data_point(self, obstacle_2d_data):
        if (len(self.pseudo_lattice) < 1):
            self.cluster_points(obstacle_2d_data)
        else:
            if not self.check_pseudo_lattice_match(obstacle_2d_data, 0.5):
                self.cluster_points(obstacle_2d_data)
    
    def check_pseudo_lattice_match(self, obstacle_2d_data, error):
        for cluster in self.pseudo_lattice:
            for point in cluster[2:]:
                if (abs(dist(point, obstacle_2d_data)) <= error):
                    print("Badhai ho lattice mil gya!")
                    lattice_cluster = [False, cluster[0], cluster[1], obstacle_2d_data]
                    self.form_triangle_lattice(lattice_cluster)
                    # print (lattice_cluster)
                    
                    return True
        return False
             
    def cluster_points(self, obstacle_2d_data):
        if not self.clusters:  # If no clusters exist
            self.clusters.append([False, obstacle_2d_data])        
        else:
            clustered = False
            for cluster_number, cluster in enumerate(self.clusters):
                if not cluster[0]:
                    first_point_in_cluster = cluster[1]
                    if ((dist(first_point_in_cluster, obstacle_2d_data) <= self.lattice_length + self.length_error) and (dist(first_point_in_cluster,obstacle_2d_data) >= self.lattice_length - self.length_error)):
                        self.clusters[cluster_number].append(obstacle_2d_data)
                        # if (len(self.clusters[cluster_number]) == 4):
                        #     self.form_triangle_lattice(self.clusters[cluster_number])
                        #     self.clusters[cluster_number][0] = True
                        if (len(self.clusters[cluster_number]) == 3):
                            self.form_pseudo_triangle_lattice(self.clusters[cluster_number])
                            self.clusters[cluster_number][0] = True 
                        clustered = True
                        break
            if not clustered:    
                self.clusters.append([False, obstacle_2d_data])
    
    def form_triangle_lattice(self, cluster):
        if not cluster[0]:
            centroid_of_lattice = np.mean(cluster[1:], axis=0)
            self.lattice_centroid.append(centroid_of_lattice.tolist())
    
    def form_pseudo_triangle_lattice(self, cluster):
        if not cluster[0]:
            X_1, Y_1 = cluster[1][0], cluster[1][1]
            X_2, Y_2 = cluster[2][0], cluster[2][1]
            third_point_of_lattice_ccw = (
                X_1 + cos(radians(60))*(X_2 - X_1) - sin(radians(60))*(Y_2 - Y_1), 
                Y_1 + sin(radians(60))*(X_2 - X_1) + cos(radians(60))*(Y_2 - Y_1)
            )
            third_point_of_lattice_cw = (
                X_1 + cos(radians(-60))*(X_2 - X_1) - sin(radians(-60))*(Y_2 - Y_1), 
                Y_1 + sin(radians(-60))*(X_2 - X_1) + cos(radians(-60))*(Y_2 - Y_1)
                )
            self.pseudo_lattice.append([cluster[1], cluster[2], third_point_of_lattice_ccw, third_point_of_lattice_cw])
        
    def print_data(self):
        print("lattice centroids: ", self.lattice_centroid)
        print("pseudo lattice coordinates: ", self.pseudo_lattice)
        print("clusters: ", self.clusters)
                
class Robot:
    def __init__(self,no_of_bots):
        self.agent_detected = False 
        self.moving_towards_cluster = False
        self.detected_agent_coordinates = [None, None]
        self.observed_lattice_obj = Lattice()
        self.robot_workspace = Workspace(no_of_bots, -1, 8, -5, 6)
        self.total_bots = no_of_bots
        self.goal_set = False 
        self.x = 0
        self.y = 0
        self.am_I_in_danger = False        
        self.incident_time = []           
        self.yaw = 0
        self.range = [0]
        self.time = 0
        self.angle = 0
        self.ang_max = 0
        self.ang_inc = 0
        self.disij = []
        self.delij = []             
        self.bearing = [0]
        self.neigh = 0
        self.robot = []
        self.safe_zone = [0,0,0] #May cause issues TODO: Change the initialization params
        self.initial_no = -1                      
        self.goal = [7, 1] #Giving fixed goal for now
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
        
        self.obstacle_coordinates = []
    
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
    
    def process_scanner_data(self,msg):
        # Threshold limit to consider as an obstacle (in meters)
        R_max = 2
        obs_size_limit = 2
        distances = []
        angles = []
        distance_angle_pairs = []
        self.detected_agent_coordinates = [None, None]
        obstacles_found = False
        self.agent_detected = False

        # Iterate through laser scan ranges and print angles where distance < limit
        for angle, distance in enumerate(msg.ranges):
            if distance < R_max:
                obstacles_found = True
                distances.append(distance)
                angles.append(angle)
                distance_angle_pairs.append((round(distance, 4), angle))
          
        # print("ROBOT NUMBER:",self.robot_id)
        ##If obstacles are found then clean the data, check it with previous data of obstacles and compare it with other agent positions to prevent false alarms 
        if (obstacles_found):
            # self.am_I_in_danger = True
            rounded_distances = np.round(distances, 4).tolist()
            median_obstacle_distance_list, edge_points_list = self.get_medians_and_edge_with_angle(distance_angle_pairs, 0.1,2)
            median_obstacle_distance_indices_list = [rounded_distances.index(val) for val in median_obstacle_distance_list]
            edge_points_indices_list = [rounded_distances.index(val[0]) for val in edge_points_list]
            for enum, index in enumerate(median_obstacle_distance_indices_list):
                if ((rounded_distances[index]*tan(radians(abs(angles[index] - angles[edge_points_indices_list[enum]])))) < obs_size_limit):
                    obstacle_x = self.odom.x + rounded_distances[index]*cos(radians(angles[index]) + self.yaw)
                    obstacle_y = self.odom.y + rounded_distances[index]*sin(radians(angles[index]) + self.yaw)
                    obstacle_coordinate = (round(obstacle_x,3), round(obstacle_y,3)) 
                    ##Is a new obstacle observed?
                    if not self.is_within_tolerance(obstacle_coordinate, self.obstacle_coordinates, 1): 
                        if not self.is_within_tolerance(obstacle_coordinate, self.robot_workspace.agents_location, 0.3):
                            self.observed_lattice_obj.process_new_data_point(obstacle_coordinate)
                            self.robot_workspace.update_obstacle_data(obstacle_coordinate, obs_size_limit)
                            default_lattice_radius = self.observed_lattice_obj.lattice_length/sqrt(3)
                            self.robot_workspace.update_lattice_data(self.observed_lattice_obj.lattice_centroid, default_lattice_radius)
                            if not self.robot_workspace.validity_of_goal(self.goal):
                                self.robot_workspace.generate_goal()
                            self.obstacle_coordinates.append(obstacle_coordinate)
                            # print(obstacle_coordinate)
                            # print(self.robot_workspace.agents_location)
                        else:
                            # Check and print the robot ID
                            # self.am_I_in_danger = False
                            robot_id = self.robot_workspace.get_robot_id_at(obstacle_coordinate)
                            if robot_id is not None:
                                # print(f"Ayyyyyyy robot detected! Robot ID: {robot_id}")
                                self.agent_detected = True
                                self.detected_agent_coordinates = self.robot_workspace.agents_location[robot_id]
                            else:
                                print("Ayyyyyyy robot detected, but ID not found!")
                        # print("Obstacle_coordinates:", self.obstacle_coordinates)


    """NAVIGATION"""
    def compute_avoidance_velocity(self, robot_position, robot_yaw, avoidance_radius, linear_velocity, angular_velocity, max_angular_velocity=1.0):
        """
        Adjusts the robot's velocity to avoid clusters based on proximity.
        """
        repulsion_vector = np.array([0.0, 0.0])
        # print("\n")
        # print(self.robot_workspace.lattice_data)
        # print(robot_position)
        # print(robot_yaw)
        # print(linear_velocity)
        # print(angular_velocity)
        for centroid,radius in self.robot_workspace.lattice_data:
            # Calculate distance from the robot to the cluster centroid
            distance_to_centroid = dist(robot_position, centroid)
            
            if distance_to_centroid < avoidance_radius:
                # Calculate the repulsive force (inverse distance weighting)
                direction_away = np.subtract(robot_position, centroid)
                normalized_direction = direction_away / np.linalg.norm(direction_away)
                repulsion_vector += normalized_direction * (avoidance_radius - distance_to_centroid)
                # print(repulsion_vector)
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
        dst = 1
        avoidance_radius = 1
        
        closest_distance = float('inf')
        
        for centroid,radius in self.robot_workspace.lattice_data:
            distance = dist(centroid, robot_position)
            if distance < closest_distance:
                closest_distance = distance
        
        if (closest_distance > avoidance_radius):
            if min(self.range[0:deg]) <= dst or min(self.range[(359-deg):]) <= dst: # front wall
                self.speed.angular.z = -0.2
                self.speed.linear.x = 0.0
            elif min(self.range[deg:120]) < dst: # left wall 
                self.speed.angular.z = 0.0
                self.speed.linear.x = 0.2
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
    
    def set_goal(self):
        """write code for identification based goal update of robot"""
        """If robot -> update goal
        If obstacle -> wall following"""
        self.goal = self.robot_workspace.generate_goal()
        self.goal_set = True
        
    # def cluster_behavior(self):
        #Find neighbor
        #Neighbor found
        #Generate goal
            #Is goal valid
            #nope
        #Remove neighbor
        
    def is_cluster_possible(self, clustering_goal):
        #Todo: Is the agent also seeing me?
        
        #Is the goal in valid workspace?
        if not self.robot_workspace.validity_of_goal(clustering_goal):
            return False
        return True
        
    def controller(self,k):
        # if not self.am_I_in_danger:
        #     print("Not in Danger")
        #     ##IF friend assigned
        #     print("Will go towards friends")
        #     ##Else
        #     print("Setting goal")
        # else:
        if self.agent_detected and not self.moving_towards_cluster:
            clustering_goal = [(self.x + self.detected_agent_coordinates[X])/2, (self.y + self.detected_agent_coordinates[Y])/2] 
            # if not self.is_cluster_possible(clustering_goal):
            #     print("Not moving towards cluster")
            #     self.set_goal()
            #     self.moving_towards_cluster = False
            # else:
            # print("moving towards cluster")
            self.goal = self.robot_workspace.generate_goal(clustering_goal)
            self.goal_set = True
            print("Agent name",self.namespace, " and its Defined goal", self.goal)
            self.moving_towards_cluster = True
        if not self.goal_set:
            self.set_goal()
        self.incx = (self.goal[X] - self.x)
        self.incy = (self.goal[Y] - self.y)

        

        # Bearing of bot
        self.bearing.append(atan2(self.incy,self.incx))

        # Distance Error
        self.dis_err = (sqrt(self.incx**2+self.incy**2))
        
        # Gradient of Bearing
        self.dtheta = (self.bearing[k] - self.bearing[k-1])/h
        if (self.dis_err) >= 0.20:
            """write code to control movement of robots based on conditions satisfied"""
            #No obstacle detected -> 
            self.speed.linear.x = 0.18
            self.speed.angular.z = K*np.sign(self.dtheta)
            
            robot_position = [self.odom.x, self.odom.y]
            obstacle_avoidance_distance = 2 #2meters
            
            for obstacle_coordinate in self.obstacle_coordinates:
                if dist(obstacle_coordinate, robot_position) < obstacle_avoidance_distance:
                    self.navigate_near_obstacle(robot_position)
        else:
            self.speed.linear.x = 0
            self.speed.linear.y = 0
            self.speed.angular.z = 0
            self.goal_set = False
            self.moving_towards_cluster = False
            print("Goal Reached")
            print("My position:", self.x, self.y)  
            # #Robot Near -> 
            # t = rospy.get_time()
            # self.speed.linear.x = max((0.18 -(5000-t)*0.0001),0)                    
            # self.speed.angular.z = K*np.sign(self.dtheta)- 0.866*np.sign(self.delij[i])

        self.cmd_vel.publish(self.speed)
        point = Point()
        point.x = self.goal[X]
        point.y = self.goal[Y]
        point.z = 0        
        self.pubg.publish(point)

if __name__ == '__main__':
    rospy.init_node("obstacle_controller")
    rospy.loginfo("Chal Gye badde")
    k = 0
    l = [] #l is time
    rate = rospy.Rate(4)
    bot_1 = Robot(12)
    # bot_2 = Robot(2, 2)     
    rospy.sleep(6)
    # bot.set_goal()
    while not rospy.is_shutdown() and k < 500000:
        k = k+1
        h = 0.25
        K = 0.3
        l.append((k+1)/10) # Time
        bot_1.controller(k)
        # bot_2.controller(k)            
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