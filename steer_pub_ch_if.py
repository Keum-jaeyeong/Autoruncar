
#! /usr/bin/env python

import rospy
import time
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32
from std_msgs.msg import String
from ar_track_alvar_msgs.msg import AlvarMarkers

class lidar_receiver:
    def __init__(self):
        self.stop_sub = rospy.Subscriber("/warning", Int32, self.callback2)
        self.drive_pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=5)
        self.camera_sub = rospy.Subscriber("/line", Int32, self.callback)
        self.camera_cor_sub = rospy.Subscriber("/corner", Int32, self.callback3)
        self.msg_sub = rospy.Subscriber("/tflag",String, self.callback4)
        self.ar_sub=rospy.Subscriber("/zed/ar_pose_marker", AlvarMarkers, self.callback5)

        self.lider_data = None
        self.image_data = None
        self.corner_data = None
        self.trf_data = None
        self.ar_data = None  
        self.ar_dis = None   
   
    def callback5(self, data):
        n = len(data.markers)
        a= None
        b= None
        if n != 0:
            a = data.markers[0].id
            b = data.markers[0].pose.pose.position.x
        self.ar_data = a
        self.ar_dis = b
        
            
    def callback4(self, data):
            self.trf_data = data.data

    def callback2(self, data):
        self.lider_data = data.data
        self.overlap()
    
    def callback3(self,data):
        self.corner_data = data.data

    def callback(self, data):
        self.image_data = data.data

    def overlap(self):
        tmp_data = AckermannDriveStamped()
        print('corner', self.corner_data)
        print('traffic', self.trf_data)
        print("av_data", self.ar_data, self.ar_dis)

        if self.ar_data != None and 0.1 < self.ar_dis < 0.2:
            #0 -> letf
            if self.ar_data == 0:
                print("0is Deteced and turn left")
                target_tick = time.time() +0.8 # find time
                while time.time() < target_tick:

                    tmp_data = AckermannDriveStamped()
                    tmp_data.drive.steering_angle = +0.5
                    tmp_data.drive.speed = 1
                    #print("ar_mark = 0")
                    self.drive_pub.publish(tmp_data)

                target_tick2 = time.time() +0.8 # find time
                while time.time() < target_tick2:

                    tmp_data = AckermannDriveStamped()
                    tmp_data.drive.steering_angle = -0.5
                    tmp_data.drive.speed = 1
                    #print("ar_mark = 0")
                    self.drive_pub.publish(tmp_data)
            #1 -> rithgt
            elif self.ar_data == 1:
                target_tick = time.time() +0.7 # find time
                while time.time() < target_tick:

                    tmp_data = AckermannDriveStamped()
                    tmp_data.drive.steering_angle = -0.5
                    tmp_data.drive.speed = 1 
                    #print("ar_mark = 1")
                    self.drive_pub.publish(tmp_data)
                target_tick2 = time.time() +0.35 # find time
                while time.time() < target_tick2:

                    tmp_data = AckermannDriveStamped()
                    tmp_data.drive.steering_angle = +0.5
                    tmp_data.drive.speed = 1
                    #print("ar_mark = 0")
                    self.drive_pub.publish(tmp_data)
        else : #marker is None --> line tracing


            if self.lider_data == 1 or self.trf_data == "R" :#or self.trf_data == "Y":
                print("stop")
                tmp_data.drive.steering_angle = 0
                tmp_data.drive.speed = 0
                self.drive_pub.publish(tmp_data)

            elif self.lider_data == 0 and self.trf_data == "Y":
                print("run_by_camera")
                tmp_data = AckermannDriveStamped()
                middle_data = 365
                multiply_rate = 0.0025  #3--> 4
                if self.corner_data == 1:
                    multiply_rate = 0.0025 #3
            
                if self.image_data < middle_data and self.image_data > 0:
                    tmp_data.drive.steering_angle = +abs(self.image_data-middle_data)*multiply_rate #Plus is left turn
                    tmp_data.drive.speed = 0.6 
                    self.drive_pub.publish(tmp_data)
                elif self.image_data >= middle_data:
                    tmp_data.drive.steering_angle = -abs(self.image_data-middle_data)*multiply_rate #minus is right turn
                    tmp_data.drive.speed = 0.6 
                    self.drive_pub.publish(tmp_data)
                # self.drive_pub.publish(tmp_data)

                

            elif self.lider_data == 0 and self.trf_data == "G":
                print("run_by_camera")
                tmp_data = AckermannDriveStamped()
                middle_data = 365
                multiply_rate = 0.0025  #3--> 4
                if self.corner_data == 1:
                    multiply_rate = 0.0025 #3
            
                if self.image_data < middle_data and self.image_data > 0:
                    tmp_data.drive.steering_angle = +abs(self.image_data-middle_data)*multiply_rate #Plus is left turn
                    tmp_data.drive.speed = 1.08#1.11 # 1.2 
                    print("left")
                    self.drive_pub.publish(tmp_data)
                elif self.image_data >= middle_data:
                    tmp_data.drive.steering_angle = -abs(self.image_data-middle_data)*multiply_rate #minus is right turn
                    tmp_data.drive.speed = 1.08#1.11 # 1.2 
                    print("right")
                    self.drive_pub.publish(tmp_data)
                # self.drive_pub.publish(tmp_data)

def run():
    rospy.init_node('steer_pub_ch_if', anonymous=True)
    lidar_receiver_a = lidar_receiver()
    rospy.spin()

if __name__ == "__main__":
    run()
