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

# Global Definitions
X = 0
Y = 1

class Lattice():
    def __init__(self):
        self.clusters = []
        self.pseudo_lattice = []
        self.lattice_centroid = []
        self.lattice_length = 1.6211
        self.length_error = 0.4
        
        
    def process_new_data_point(self, obstacle_2d_data):
        if (len(self.pseudo_lattice) < 1):
            self.cluster_points(obstacle_2d_data)
        else:
            if not self.check_pseudo_lattice_match(obstacle_2d_data, 0.1):
                self.cluster_points(obstacle_2d_data)
    
    def check_pseudo_lattice_match(self, obstacle_2d_data, error):
        for cluster in self.pseudo_lattice:
            for point in cluster[2:]:
                if (abs(dist(point, obstacle_2d_data)) <= error):
                    print("Badhai ho lattice mil gya!")
                    lattice_cluster = [False, cluster[0], cluster[1], obstacle_2d_data]
                    self.form_triangle_lattice(lattice_cluster)
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
            self.lattice_centroid.append(centroid_of_lattice)
    
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
        
    def print_data(self, arguments):
        # if arguments == "all":
        print("lattice centroids: ", self.lattice_centroid)
        print("pseudo lattice coordinates: ", self.pseudo_lattice)
        print("clusters: ", self.clusters)
                
class obstacle(Pose2D):
    def __init__(self,x,y,theta,dist,min_dis):
        super().__init__(x,y,theta)
        self.static = False
        self.detected_time = rospy.get_time()
        self.dist = dist
        self.min_dis = min_dis
    
    def __eq__(self,other):
        if abs(self.x - other.x)< 0.1 :
            if abs(self.y - other.y)< 0.1 :
                return True
            else:
                return False
        else:
            return False

class robot:
    def __init__(self,no_of_bots): 
        self.total_bots = no_of_bots 
        self.x = 0
        self.y = 0        
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
        self.bot_position = []
        self.safe_zone = [0,0,0] #May cause issues TODO: Change the initialization params
        self.initial_no = -1                      
        self.goal = Point(8.0, 0.0, 0.0) #Giving fixed goal for now
        self.namespace = rospy.get_namespace()
        self.speed = Twist()
        self.hist = deque(maxlen=20)
        self.odom_sub = rospy.Subscriber("/odom",Odometry,self.update_Odom)
        self.bot_data = rospy.Subscriber("/obs_data", botPose, self.get_other_bot_position) 
        self.object_detector = rospy.Subscriber('/scan',LaserScan, self.scanner)

        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)    
        self.pubg = rospy.Publisher('/goal', Point, queue_size=1)
        self.rad_pub = rospy.Publisher('/radius', Point, queue_size=1)
        # self.obs_pub = rospy.Publisher('/obs',obs, queue_size=1)
        
        self.obstacle_coordinates = []
        self.lattice_centroids = []
        
        self.lattice_obj = Lattice()
        
    def get_other_bot_position(self, msg):
        self.bot_position = []
        bot_info = msg.botpose
        for i in range(len(msg.bot_id)):
            position = bot_info[i].pose.pose.position
            self.bot_position.append([position.x, position.y])
            
        
    def update_Odom(self,odom):
        """ Odometry of current bot"""        
        self.x = odom.pose.pose.position.x 
        self.y = odom.pose.pose.position.y
        self.rot_q = odom.pose.pose.orientation
        euler = euler_from_quaternion([self.rot_q.x , self.rot_q.y , self.rot_q.z , self.rot_q.w ])
        self.yaw = atan2(sin(euler[2]),cos(euler[2])) 
        self.odom = Pose2D(self.x,self.y,self.yaw)
        self.vel = Pose2D(odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.angular.z)

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
    
    # def form_lattice_structure(self):
    #     obstacle_coordinates_np = np.array(self.obstacle_coordinates)
    #     # print(obstacle_coordinates_np)
    #     if (obstacle_coordinates_np.shape[0]>=2): 
    #     # Initialize Agglomerative Clustering
    #     # The metric can be 'euclidean', 'manhattan', or a custom distance function
    #         agg_clustering = cluster.AgglomerativeClustering(
    #             n_clusters=None,    # Set to None to decide the number of clusters based on distance threshold
    #             distance_threshold=3,  # Distance threshold to merge clusters, you can adjust this value
    #             metric='euclidean',  # You can also change to other distance metrics, e.g., 'manhattan'
    #             linkage='ward'  # The linkage criterion defines how the distance between clusters is calculated
    #         )
    #         # Fit the model
    #         agg_clustering.fit(obstacle_coordinates_np)

    #         # Print the clusters
    #         labels = agg_clustering.labels_
    #         unique_labels = set(labels)
    #         self.lattice_centroids = []
    #         # Group points by cluster label
    #         for lattice_id in unique_labels:    
    #             cluster_points = obstacle_coordinates_np[labels == lattice_id]
    #             lattice_centroid = np.mean(cluster_points, axis=0)  # Mean of points in the cluster
    #             self.lattice_centroids.append(lattice_centroid)
    #             print(f"Lattice {lattice_id}:")
    #             for i, label in enumerate(labels):
    #                 if label == lattice_id:
    #                     print(f"    {obstacle_coordinates_np[i]}")
    
    
    
    def is_within_tolerance(self, new_point, obstacles, tolerance):
        # Check if any point in obstacles is within tolerance of new_point
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
        # Iterate through laser scan ranges and print angles where distance < limit
        for angle, distance in enumerate(msg.ranges):
            if distance < R_max:
                distances.append(distance)
                angles.append(angle)
                distance_angle_pairs.append((round(distance, 4), angle))
        rounded_distances = np.round(distances, 4).tolist()
        median_obstacle_distance_list, edge_points_list = self.get_medians_and_edge_with_angle(distance_angle_pairs, 0.1,2)
        median_obstacle_distance_indices_list = [rounded_distances.index(val) for val in median_obstacle_distance_list]
        edge_points_indices_list = [rounded_distances.index(val[0]) for val in edge_points_list]
        for enum, index in enumerate(median_obstacle_distance_indices_list):
            if ((rounded_distances[index]*tan(radians(abs(angles[index] - angles[edge_points_indices_list[enum]])))) < obs_size_limit):
                obstacle_x = self.odom.x + rounded_distances[index]*cos(radians(angles[index]) + self.yaw)
                obstacle_y = self.odom.y + rounded_distances[index]*sin(radians(angles[index]) + self.yaw)
                obstacle_coordinate = (round(obstacle_x,3), round(obstacle_y,3))
                if not self.is_within_tolerance(obstacle_coordinate, self.obstacle_coordinates, 1) and not self.is_within_tolerance(obstacle_coordinate, self.bot_position, 0.5):
                    self.lattice_obj.process_new_data_point(obstacle_coordinate)
                    self.obstacle_coordinates.append(obstacle_coordinate)
                    # print("Obstacle_coordinates:", self.obstacle_coordinates)
                # print("Obstacle mila")
            # else:
                
                # print("WALL DETECTED AAAAA")
                
            


    def compute_avoidance_velocity(self, robot_position, robot_yaw, avoidance_radius, linear_velocity, angular_velocity, max_angular_velocity=1.0):
        """
        Adjusts the robot's velocity to avoid clusters based on proximity.
        """
        repulsion_vector = np.array([0.0, 0.0])
        # print("\n")
        # print(self.lattice_centroids)
        # print(robot_position)
        # print(robot_yaw)
        # print(linear_velocity)
        # print(angular_velocity)
        for centroid in self.lattice_centroids:
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
            # print(adjusted_linear_velocity, adjusted_angular_velocity)
            return adjusted_linear_velocity, adjusted_angular_velocity
        # print(linear_velocity, angular_velocity)
        # No obstacles nearby, proceed with given velocities
        return linear_velocity, angular_velocity
        

    # def wall_following(self):
    #     """funct to follow wall boundary
    #     Turn Right by default or rotate on CCW fashion"""
    #     # print("Wall following")
    #     deg = 30
    #     dst = 0.5
    #     # while True:
    #     if min(self.ranges[0:deg]) <= dst or min(self.ranges[(359-deg):]) <= dst: # front wall
    #         self.speed.angular.z = -0.2
    #         self.speed.linear.x = 0.0
    #     elif min(self.ranges[deg:120]) < dst: # left wall 
    #         self.speed.angular.z = 0.0
    #         self.speed.linear.x = 0.2
    #         # print("Left wall")
    #     else:
    #         self.speed.angular.z = 0.1
    #         self.speed.linear.x = 0.2
    
    def set_goal(self):
        """write code for identification based goal update of robot"""
        """If robot -> update goal
        If obstacle -> wall following"""

    def controller(self,k):
        self.set_goal()
        self.incx = (self.goal.x - self.x)
        self.incy = (self.goal.y - self.y)

        # Bearing of bot
        self.bearing.append(atan2(self.incy,self.incx))

        # Distance Error
        self.dis_err = (sqrt(self.incx**2+self.incy**2))
  
        #print(self.dis_err)        

        # Gradient of Bearing
        self.dtheta = (self.bearing[k] - self.bearing[k-1])/h
        self.lattice_obj.print_data("AAAAAAAA")
        # for obs_element in self.obs:
        #     x_diff = obs_element.x -self.x
        #     y_diff = obs_element.y -self.y
            
        #     dist = sqrt(x_diff**2 + y_diff**2)
        #     ang = atan2(y_diff, x_diff)
            
        #     self.disij.append(dist)
        #     self.delij.append(ang)  
        avoidance_radius = 1.5 
        if (self.dis_err) >= 0.850:
            """write code to control movement of robots based on conditions satisfied"""
            #No obstacle detected -> 
            self.speed.linear.x = 0.18
            self.speed.angular.z = K*np.sign(self.dtheta)
            self.lattice_centroids = self.lattice_obj.lattice_centroid
            # self.form_lattice_structure() 
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
            self.speed.linear.x = 0
            self.speed.linear.y = 0
            self.speed.angular.z = 0
            print("GOAAAAAAAAAAAAAAAAAALLLLLLLLLLLLLLLL REEEEEEEEEEEEEAAAAAAAAAAAAAAAACCCCCCCCCCCCCCHHHHHHHHHHHHHHHHHEEEEEEEEEEEEEEEEEDDDDDDDDDDDDDDD")  
            # #Robot Near -> 
            # t = rospy.get_time()
            # self.speed.linear.x = max((0.18 -(5000-t)*0.0001),0)                    
            # self.speed.angular.z = K*np.sign(self.dtheta)- 0.866*np.sign(self.delij[i])

        self.cmd_vel.publish(self.speed)
        self.pubg.publish(self.goal)

if __name__ == '__main__':
    rospy.init_node("obstacle_controller")
    rospy.loginfo("Chal Gye badde")
    k = 0
    l = [] #l is time
    rate = rospy.Rate(4)
    bot_1 = robot(1)
    # bot_2 = robot(1)     
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
    # lattice_obj = Lattice()
    # # First test point, will start forming a cluster
    # obstacle_2d_data = (1, 2)  
    # lattice_obj.process_new_data_point(obstacle_2d_data)
    # print("Clusters after 1st point:", lattice_obj.clusters)

    # # Second test point, will form a pseudo lattice as there's only one point in pseudo_lattice
    # obstacle_2d_data = (4, 2)  
    # lattice_obj.process_new_data_point(obstacle_2d_data)
    # print("Clusters after 2nd point:", lattice_obj.clusters)
    
    # # Third test point, will check if it matches with the pseudo lattice
    # obstacle_2d_data = (2.5, 4.6)  # This should be close to the second point, and should trigger the lattice formation
    # lattice_obj.process_new_data_point(obstacle_2d_data)
    # print("Clusters after 3rd point:", lattice_obj.clusters)
    
    # # Fourth test point, which should not match the existing pseudo lattice, so a new cluster will form
    # obstacle_2d_data = (5, 5)  
    # lattice_obj.process_new_data_point(obstacle_2d_data)
    # print("Clusters after 4th point:", lattice_obj.clusters)
    
    # lattice_obj.print_data("acjks")