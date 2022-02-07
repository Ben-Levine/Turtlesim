#!/usr/bin/env python
#
# Benjamin Levine
# SES 598: Autonomous Exploration Systems, SP22
# Dr. Jnaneshwar Das

# Import statements
import rospy
import time
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill

# Create class containing all operations for turtlesim actions
class Turtle:
    def __init__(self):
        rospy.init_node('Test', anonymous=True)

        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)

        self.msg = Twist()
        self.msg_rate = rospy.Rate(10)
        self.msg_rate.sleep()
        self.pose = Pose()
        self.velo = 1

    # Spawn turtle at specific location
    def new_turt_spawn(self, x, y):
        # Kill original turtle
        kill_self = rospy.ServiceProxy('/kill', Kill)
        kill = kill_self("turtle1")

        # Create new turtle
        spawn_turt = rospy.ServiceProxy('/spawn', Spawn)
        new_spawn = spawn_turt(x, y, 0, "turtle2")
        time.sleep(0.5)
        self.pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber('/turtle2/pose', Pose, self.update_pose)

    # Update turtle pose
    def update_pose(self, data):
        self.pose = data

    # Calculate mean squared error
    def norm_const(self, goal_x, goal_y):
        return math.sqrt((goal_x - self.pose.x)**2 + (goal_y - self.pose.y)**2)

    # Calculate theta error
    def norm_th(self, del_x, del_y):
        return abs(math.atan2(del_y, del_x) - self.pose.theta)

    # Move along x axis for specified distance and velocity
    def move_len(self, dist, velo):
        time.sleep(1)
        self.msg.linear.x = 0
        self.msg.angular.z = 0

        # Determine goal distance
        if(self.pose.theta > (math.pi/2)):
            goalx = self.pose.x - dist
        else:
            goalx = self.pose.x + dist

        # Distance threshhold for barrier
        if(goalx > 11.05):
            goalx = 11.05

        goaly = self.pose.y
        error = self.norm_const(goalx, goaly)

        # Move forward at specified velocity until error threshhold is reached
        while error > 0.05:
            self.msg.linear.x = velo
            self.msg.angular.z = 0
            self.pub.publish(self.msg)
            error = self.norm_const(goalx, goaly)

        self.msg.linear.x = 0
        self.pub.publish(self.msg)

    # Move along y axis for specified distance and velocity
    def move_wid(self, dist, velo):
        time.sleep(1)
        self.msg.linear.x = 0
        self.msg.angular.z = 0
        goalx = self.pose.x
        goaly = self.pose.y + dist

        # Distance threshhold for barrier
        if(goaly > 11.05):
            goaly = 11.05
        dist_error = self.norm_const(goalx, goaly)

        while dist_error > 0.05:
            self.msg.linear.x = velo
            self.pub.publish(self.msg)
            dist_error = self.norm_const(goalx, goaly)
        self.msg.linear.x = 0
        self.pub.publish(self.msg)

    # Turn left
    def turn_left(self):
        time.sleep(1)
        self.msg.linear.x = 0
        self.msg.angular.z = 0
        th_error = self.norm_th(0, 5)

        # Determine turtle orientation for theta calculation
        if(th_error < 0.1):
            th_error = self.norm_th(-5, 0)
            while th_error > 0.01:
                self.msg.linear.x = 0
                self.msg.angular.z = self.velo
                self.pub.publish(self.msg)
                th_error = self.norm_th(-5, 0)
        else:
            while th_error > 0.01:
                self.msg.linear.x = 0
                self.msg.angular.z = self.velo
                self.pub.publish(self.msg)
                th_error = self.norm_th(0, 5)

        self.msg.angular.z = 0
        self.pub.publish(self.msg)

    # Turn right
    def turn_right(self):
        time.sleep(1)
        self.msg.linear.x = 0
        self.msg.angular.z = 0
        th_error = self.norm_th(0, 5)

        # Determine turtle orientation
        if (th_error < 0.1):
            th_error = self.norm_th(5, 0)
            while th_error > 0.01:
                self.msg.linear.x = 0
                self.msg.angular.z = -self.velo
                self.pub.publish(self.msg)
                th_error = self.norm_th(5, 0)
        else:
            while th_error > 0.01:
                self.msg.linear.x = 0
                self.msg.angular.z = -self.velo
                self.pub.publish(self.msg)
                th_error = self.norm_th(0, 5)
        self.msg.angular.z = 0
        self.pub.publish(self.msg)

    # Perform lawnmower pattern for specific length, width, and velocity
    def lawnmower(self, length, width, velo):
        self.move_len(length, velo)
        self.turn_left()
        self.move_wid(width, velo)
        self.turn_left()
        self.move_len(length, velo)
        self.turn_right()
        self.move_wid(width, velo)
        self.turn_right()
        self.move_len(length, velo)

# Main method
def main():
    count = 0       # Counter for loop
    length = 3      # Define length of lawnmower
    width = 1       # Define width of lawnmower
    velo = 1        # Define velocity of lawnmower
    t1 = Turtle()   # Create new turtlesim object
    t1.new_turt_spawn(2, 3)    # Spawn turtle at custom location (x, y)
    while not rospy.is_shutdown():
        while(count < 1):
            t1.lawnmower(length, width, velo)
            count += 1

if __name__ == '__main__':
    main()