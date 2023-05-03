#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan


robot_pose = None
obstacles = []
goal = None

# Callback functions
def pose_callback(msg):
    global robot_pose
    # Update robot's pose
    robot_pose = msg.pose

def obstacles_callback(msg):
    global obstacles
    # Update obstacle positions
    obstacles = [process_obstacle_reading(r) for r in msg.ranges]

def goal_callback(msg):
    global goal
    # Update the goal position
    goal = (msg.pose.position.x, msg.pose.position.y)

# Main function
def main():
    global robot_pose, obstacles, goal

    # Initialize the node
    rospy.init_node('my_navigation_node')

    # Subscribe to the necessary topics
    rospy.Subscriber('/robot_pose', PoseStamped, pose_callback)
    rospy.Subscriber('/obstacles', LaserScan, obstacles_callback)
    rospy.Subscriber('/goal', PoseStamped, goal_callback)

    # Create a publisher for the robot's velocity commands
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Set the rate at which to send velocity commands
    rate = rospy.Rate(10)  # 10 Hz

    # Main loop
    while not rospy.is_shutdown():
        # Make sure all data is available
        if robot_pose is None or obstacles is None or goal is None:
            continue

        # Call the modified algorithm function to calculate vL and vR
        vL, vR = modified_algorithm(robot_pose, obstacles, goal)

        # Create and populate the Twist message
        cmd_vel = Twist()
        cmd_vel.linear.x = (vL + vR) / 2
        cmd_vel.angular.z = (vR - vL) / robot_width

        # Publish the message
        cmd_vel_pub.publish(cmd_vel)

        # Sleep to maintain the desired rate
        rate.sleep()

if __name__ == '__main__':
    main()
