#!/usr/bin/env python3
import rospy 
import statistics
from geometry_msgs.msg import Point, Twist, Pose2D
from sensor_msgs.msg import LaserScan
import numpy as np
from sklearn import cluster
from collections import deque
from math import *
# from swarm_aggregation.msg import obs
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

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
        self.safe_zone = [0,0,0] #May cause issues TODO: Change the initialization params
        self.initial_no = -1                      
        self.goal = Point(2.0, 4, 0.0) #Giving fixed goal for now
        self.odom = Odometry()
        self.namespace = rospy.get_namespace()
        self.speed = Twist()
        self.hist = deque(maxlen=20)

        self.object_detector = rospy.Subscriber('/scan',LaserScan, self.scanner)
        self.odom_sub = rospy.Subscriber("/odom",Odometry,self.update_Odom) 

        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)    
        self.pubg = rospy.Publisher('/goal', Point, queue_size=1)
        self.rad_pub = rospy.Publisher('/radius', Point, queue_size=1)
        # self.obs_pub = rospy.Publisher('/obs',obs, queue_size=1)      
        
        self.obstacle_coordinates = []
        
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

    def get_medians(self, data_points, epsilon):
        sorted_data_points = sorted(data_points)
        clusters = []
        current_cluster = []
        
        # Step 1: Form clusters based on the distance threshold
        for current_data_point in sorted_data_points:
            # Step 2: Form clusters based on the distance threshold
            if not current_cluster:
                current_cluster.append(current_data_point)
            else:
                # Check if the current distance is close to the last distance in the cluster
                prev_data_point = current_cluster[-1]
                if self.distance(current_data_point, prev_data_point) <= epsilon:
                    current_cluster.append(current_data_point)
                else:
                    # Save the completed cluster and start a new one
                    clusters.append(current_cluster)
                    current_cluster = [current_data_point]

        # Add the last cluster if any
        if current_cluster:
            clusters.append(current_cluster)

        # Step 3: Calculate and print the medians of each cluster
        median_array = []
        for cluster in clusters:
            median_point = statistics.median_low(cluster)
            median_array.append(median_point)
        return median_array
    
    def form_lattice_structure(self):
        obstacle_coordinates_np = np.array(self.obstacle_coordinates)
        
        # Initialize Agglomerative Clustering
        # The metric can be 'euclidean', 'manhattan', or a custom distance function
        agg_clustering = cluster.AgglomerativeClustering(
            n_clusters=None,    # Set to None to decide the number of clusters based on distance threshold
            distance_threshold=3,  # Distance threshold to merge clusters, you can adjust this value
            metric='euclidean',  # You can also change to other distance metrics, e.g., 'manhattan'
            linkage='ward'  # The linkage criterion defines how the distance between clusters is calculated
        )

        # Fit the model
        agg_clustering.fit(obstacle_coordinates_np)

        # Print the clusters
        labels = agg_clustering.labels_
        unique_labels = set(labels)

        # Group points by cluster label
        for cluster_id in unique_labels:
            print(f"Cluster {cluster_id}:")
            for i, label in enumerate(labels):
                if label == cluster_id:
                    print(f"    {obstacle_coordinates_np[i]}")
        
        
            
    
    def is_within_tolerance(self, new_point, obstacles, tolerance):
        # Check if any point in obstacles is within tolerance of new_point
        for existing_point in obstacles:
            distance = dist(new_point, existing_point)  # Use Euclidean distance
            if distance < tolerance:
                return True  # The point is too close, treat it as a duplicate
        return False  # The point is far enough to be considered unique

    
    def process_scanner_data(self,msg):
        # Threshold limit to consider as an obstacle (in meters)
        R_max = 10.0
        obstacles = []
        distances = []
        angles = []
        # Iterate through laser scan ranges and print angles where distance < limit
        for angle, distance in enumerate(msg.ranges):
            if distance < R_max:
                distances.append(distance)
                angles.append(angle)
        rounded_distances = np.round(distances, 4).tolist()
        obstacle_distances = self.get_medians(rounded_distances, 0.1)
        indices = [i for i, val in enumerate(rounded_distances) if val in obstacle_distances]
        for index in indices:
            obstacle_x = self.odom.x + rounded_distances[index]*cos(radians(angles[index]) + self.yaw)
            obstacle_y = self.odom.y + rounded_distances[index]*sin(radians(angles[index]) + self.yaw)
            obstacle_coordinate = (round(obstacle_x,3), round(obstacle_y,3))
            if not self.is_within_tolerance(obstacle_coordinate, self.obstacle_coordinates, 1):
                self.obstacle_coordinates.append(obstacle_coordinate)
        # print("Obstacles are at:", self.obstacle_coordinates)


    def wall_following(self):
        """funct to follow wall boundary
        Turn Right by default or rotate on CCW fashion"""
        # print("Wall following")
        deg = 30
        dst = 0.5
        # while True:
        if min(self.ranges[0:deg]) <= dst or min(self.ranges[(359-deg):]) <= dst: # front wall
            self.speed.angular.z = -0.2
            self.speed.linear.x = 0.0
        elif min(self.ranges[deg:120]) < dst: # left wall 
            self.speed.angular.z = 0.0
            self.speed.linear.x = 0.2
            # print("Left wall")
        else:
            self.speed.angular.z = 0.1
            self.speed.linear.x = 0.2
    
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
        if self.dis_err < 1:
            self.form_lattice_structure()
        #print(self.dis_err)        

        # Gradient of Bearing
        self.dtheta = (self.bearing[k] - self.bearing[k-1])/h

        # for obs_element in self.obs:
        #     x_diff = obs_element.x -self.x
        #     y_diff = obs_element.y -self.y
            
        #     dist = sqrt(x_diff**2 + y_diff**2)
        #     ang = atan2(y_diff, x_diff)
            
        #     self.disij.append(dist)
        #     self.delij.append(ang)  
        if (self.dis_err) >= 0.850:
            """write code to control movement of robots based on conditions satisfied"""
            #No obstacle detected -> 
            self.speed.linear.x = 0.18
            self.speed.angular.z = K*np.sign(self.dtheta)
                
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
    bot = robot(2)     
    rospy.sleep(6)
    # bot.set_goal()
    while not rospy.is_shutdown() and k < 500000:
        k = k+1
        h = 0.25
        K = 0.3
        l.append((k+1)/10) # Time
        bot.controller(k)            
        rate.sleep()