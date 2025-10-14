import rospy
import rospkg
import os
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Odometry, Path

class GlobalPath:
    def __init__(self, file_name):
        rospy.init_node('global_path', anonymous=True)
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('planning_pkg')
        file_path = os.path.join(pkg_path, 'data', file_name)

        f = open(file_path, "r")
        lines = f.readlines()
        f.close()

        for line in lines: 
            x, y, z = line.split() # ENU coord
            read_pose = PoseStamped()
            read_pose.pose.position.x = float(x)
            read_pose.pose.position.y = float(y)
            read_pose.pose.orientation.w=1
            self.global_path_msg.poses.append(read_pose)        

    def publish_path(self):
        rate = rospy.Rate(5) # 100hz
        while not rospy.is_shutdown():
            self.global_path_pub.publish(self.global_path_msg)
            rospy.loginfo_once("publish global path!")
            rate.sleep()

if __name__ == "__main__":
    try:
        node = GlobalPath("25molit_main_mission.txt")
        node.publish_path()
    except rospy.ROSInterruptException:
        pass





