# Source: modelled off of code at http://wiki.ros.org/turtlesim/Tutorials/Rotating%20Left%20and%20Right
# Code used to make robot follow a set path during our experiements. 

import rospy
from geometry_msgs.msg import Twist

PI = 3.1415926535897
SLEEP = 1
RATE = 10

L_SPEED = 0.3 # Linear velocity.
A_SPEED = L_SPEED * 20 # Angular velocity.

RIGHT = True # Clockwise.
LEFT = False # Counterclockwise.
FORWARD = True
BACKWARDS = False

# Parameters: publisher, angle in degrees, direction right or left
def rotate(velocity_publisher, angle, clockwise):
    vel_msg = Twist()
    speed = A_SPEED
    
    #Converting from angles to radians
    angular_speed = speed*2*PI/360
    relative_angle = angle*2*PI/360
    
    #We wont use linear components except x
    vel_msg.linear.x=0.06 # Through trial & error
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    
    # Checking if our movement is CW or CCW
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
       vel_msg.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    
    while(current_angle < relative_angle):
        velocity_publisher.publish(vel_msg)
        rospy.Rate(RATE).sleep()
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)
    
    #Forcing our robot to stop
    vel_msg.angular.z = 0
    vel_msg.linear.x = 0
    print("STOP!!!")
    velocity_publisher.publish(vel_msg)
    rospy.Rate(RATE).sleep()

# Parameters: publisher, angle in degrees, direction forward or backwards
def move(velocity_publisher, distance, forward):
    vel_msg = Twist()
    speed = L_SPEED
    
    # Only use linear components
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x=0
    vel_msg.angular.y=0
    vel_msg.angular.z=0

    # Checking if our movement is CW or CCW
    if forward:
        vel_msg.linear.x = speed
    else:
        vel_msg.linear.x = -speed
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_distance = 0
    
    while(current_distance < distance):
        velocity_publisher.publish(vel_msg)
        rospy.Rate(RATE).sleep()
        t1 = rospy.Time.now().to_sec()
        current_distance = speed*(t1-t0)
    
    #Forcing our robot to stop
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)
    rospy.Rate(RATE).sleep()

if __name__ == '__main__':
    rospy.init_node('move_bot', anonymous=True) # Start a new node
    publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=0)
    rate = rospy.Rate(RATE)
    rate.sleep()

    # CREATE PATH HERE
    move(publisher, 0.2, FORWARD)
    
    rotate(publisher, 30, RIGHT)
    move(publisher, 0.1, FORWARD)

    rotate(publisher, 60, LEFT)
    move(publisher, 0.2, FORWARD)

    rotate(publisher, 30, RIGHT)
    move(publisher, 0.8, BACKWARDS)
    
