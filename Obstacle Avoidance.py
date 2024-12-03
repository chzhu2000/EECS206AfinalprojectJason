import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback(data):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    twist = Twist()

    # Check distances in front of the robot
    min_distance = min(data.ranges[0:30] + data.ranges[330:359])  # 0-30 and 330-359 degrees
    rospy.loginfo(f"Min distance: {min_distance}")

    if min_distance < 0.5:  # If an obstacle is closer than 0.5 meters
        twist.linear.x = 0.0  # Stop
    else:
        twist.linear.x = 0.2  # Move forward
    pub.publish(twist)

def listener():
    rospy.init_node('obstacle_avoidance', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
