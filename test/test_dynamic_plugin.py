import rospy
import sys
from time import sleep
import math
from geometry_msgs.msg import Vector3
import gazebo_msgs.msg

class test_fin_plugin():
    def __init__(self):
        self._rate = rospy.Rate(10)
        self._count = 0
        rospy.init_node('test_fins', anonymous=True)

        self.fin_input_pub_1 = rospy.Publisher('robodact1_id0/tail_joint_1/command', Vector3, queue_size=10)
        self.fin_input_pub_2 = rospy.Publisher('robodact1_id0/tail_joint_2/command', Vector3, queue_size=10)

    def run(self):
        while not rospy.is_shutdown():
            self._count += 1
            print("Count: ", self._count)
            ros_msg_1 = Vector3()
            ros_msg_2 = Vector3()
            amp = 30 / 180 * 3.14
            ros_msg_1.x = 0 # angle
            ros_msg_1.y = 0.1* amp * 4*3.14*math.cos(self._count/5.0) #velocity
            ros_msg_1.z = -0.1* amp * 4*3.14* 4*3.14*math.sin(self._count/5.0) #acceleration

            ros_msg_2.x = 0
            ros_msg_2.y = 0.2* amp * 4*3.14*math.cos(self._count/5.0) +  0.1* amp * 4*3.14* math.cos(self._count/5.0 - 30/180*3.14)#velocity
            ros_msg_2.z = -0.2* amp * 4*3.14* 4*3.14*math.sin(self._count/5.0) - 0.1* amp * 4*3.14* 4*3.14* math.sin(self._count/5.0 - 30/180*3.14)#acceleration
            self.fin_input_pub_1.publish(ros_msg_1)
            self.fin_input_pub_2.publish(ros_msg_2)
            self._rate.sleep()


if __name__ == '__main__':
    # 初始化 ROS 节点
    rospy.init_node('test_fins', anonymous=True)

    # 创建插件对象并启动
    plugin = test_fin_plugin()
    plugin.run()