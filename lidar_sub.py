#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32

class lidar_receiver:
    def __init__(self):
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        self.stop_pub = rospy.Publisher("/warning", Int32, queue_size=5)

    def callback(self, data):
        # for i in range(len(data.ranges)):
        #     angle = data.angle_min + i * data.angle_increment
        #     if angle > -0.6 and angle < 0.6:
        #         print("{}th index range = {}". format(i, data.ranges[i]))
        count = 0
        for i in range(355,401):
            if data.ranges[i] < 1.15:    #1.3
                # print("warning")
                self.stop_pub.publish(1)
                count += 1
        if count != 0:        
            print("warning",data.ranges[360])
        if count == 0:
            print("safe",data.ranges[i])
            self.stop_pub.publish(0)

def run():
    rospy.init_node('lidar_sub', anonymous=True)
    lidar_receiver_a = lidar_receiver()
    rospy.spin()

if __name__ == "__main__":
    run()
