import rospy
from std_msgs.msg import String

class demo_class:
    def __init__(self):
        rospy.init_node("demo_py_node", anonymous=False)
        rospy.Subscriber("cb", String, self.cb)
        rospy.spin()
    
    def cb(self, msg):
        rospy.loginfo(msg)

if __name__ == "__main__":
    a = demo_class()