import rospy 

class Robot:
    def __init__(self):
        self.x = None
        self.y = None
        self.yaw = None
        self.safe_zone = [None, None, None]
        




if __name__ == '__main__':
    rospy.init_node("robot_node")
    k = 0
    l = [] #l is time
    rate = rospy.Rate(4)
    bot_1 = Robot()
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