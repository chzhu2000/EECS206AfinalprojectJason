import rospy
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers

class ARTagFollower:
    def __init__(self):
        rospy.init_node('ar_tag_follower', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.callback)
        self.twist = Twist()
        self.target_tag_id = 1  # Set the AR tag ID you want to follow

    def callback(self, data):
        if not data.markers:
            rospy.loginfo("No AR tag detected.")
            self.stop_robot()
            return

        for marker in data.markers:
            if marker.id == self.target_tag_id:
                x = marker.pose.pose.position.x
                z = marker.pose.pose.position.z

                rospy.loginfo(f"Detected AR tag {marker.id} at x: {x}, z: {z}")

                # Movement logic
                if z > 0.5:  # Maintain a distance of 0.5 meters
                    self.twist.linear.x = 0.2
                else:
                    self.twist.linear.x = 0.0

                self.twist.angular.z = -x * 0.5  # Turn toward the tag
                self.pub.publish(self.twist)
                return

        rospy.loginfo("Target AR tag not detected.")
        self.stop_robot()

    def stop_robot(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.pub.publish(self.twist)

if __name__ == "__main__":
    try:
        ARTagFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
