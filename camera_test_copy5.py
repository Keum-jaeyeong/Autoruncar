
#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np


class lidar_receiver:
    def __init__(self):
        self.lidar_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.callback)
        self.stop_pub = rospy.Publisher("/check", Int32, queue_size=5)
        self.camera_pub = rospy.Publisher("/line", Int32, queue_size=5)
        self.camera_cor_pub = rospy.Publisher("/corner", Int32, queue_size=5)
        self.cvbridge = CvBridge()

    def callback(self, data):
        frame = self.cvbridge.imgmsg_to_cv2(data, "bgr8")
	
        def grayscale(img):
	    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        def canny(img, low_threshold, high_threshold): # Canny
            return cv2.Canny(img, low_threshold, high_threshold)

        def gaussian_blur(img, kernel_size): # Gausian
            return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

        def region_of_interest(img,vertices,color3=(255,255,255),color1 = 255):
            mask = np.zeros_like(img)
            if len(img.shape) >2:
                color = color3
            else:
                color =color1
            cv2.fillPoly(mask, vertices,color)
            ROI_img= cv2.bitwise_and(img,mask)
            return ROI_img


        def draw_lines(img, lines, color=[0, 0, 255], thickness=2): # draw
            for line in lines:
                for x1,y1,x2,y2 in line:
                    cv2.line(img, (x1, y1), (x2, y2), color, thickness)
        def draw_fit_line(img, lines, color=[255, 0, 0], thickness=10): # draw_fit
            cv2.line(img, (lines[0], lines[1]), (lines[2], lines[3]), color, thickness)

        def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap): # hough
            lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
            #line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
            #draw_lines(line_img, lines)
            return lines

        def weighted_img(img, initial_img, a=1, b=1., c=0.): # over
            return cv2.addWeighted(initial_img, a, img, b, c)

        def get_fitline(img, f_lines): # find fitline   
            lines = np.squeeze(f_lines)
            lines = lines.reshape(lines.shape[0]*2,2)
            rows,cols = img.shape[:2]
            output = cv2.fitLine(lines,cv2.DIST_L2,0, 0.01, 0.01)
            vx, vy, x, y = output[0], output[1], output[2], output[3]
            x1, y1 = int(((img.shape[0]-1)-y)/vy*vx + x) , img.shape[0]-1
            x2, y2 = int(((img.shape[0]/2+100)-y)/vy*vx + x) , int(img.shape[0]/2+100)
    
            result = [x1,y1,x2,y2]
            return result


        # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # lower_th = np.array([0, 50, 60])#bgr_mask
        # higher_th = np.array([50, 100, 255])#bgr_mask
        # mask = cv2.inRange(hsv, lower_th, higher_th)
        # colored_frame = np.zeros_like(frame)
        # res = cv2.bitwise_and(frame, frame, mask = mask)
        pts1 = np.float32([[200,275], [500,275], [600, 350], [100, 350]])
        pts2 = np.float32([[0,0], [640,0], [640, 480], [0, 480]])
        pts3 = np.float32([[300,275], [400,275], [450, 350], [250, 350]]) # change under TOO

        M = cv2.getPerspectiveTransform(pts1, pts2)
        inv_M = cv2.getPerspectiveTransform(pts2, pts1)
        BEV = cv2.warpPerspective(frame, M, (640,480))

        height , width =frame.shape[:2]


        gray_img = grayscale(frame)
        blur_img = gaussian_blur(gray_img, 3)
        canny_img = canny(blur_img, 70, 210)
    
        vertices = np.array([[(200,275),(500,275),(600,350),(100,350)]], dtype=np.int32)
    
        ROI_img = region_of_interest(canny_img,vertices)
        cv2.imshow("ROI", ROI_img)

        #hough_img = hough_lines(canny_img, 1, 1*np.pi/180, 30, 30, 20)
        
        try: # two line( go straight)
            #set region of remove(cross line)
            vertices2 = np.array([[(300,275),(400,275),(450,350),(250,350)]], dtype=np.int32)
            ROI_img2 = region_of_interest(canny_img,[vertices,vertices2])
            
            
            #find slope
            line_arr = hough_lines(ROI_img2, 1, 1*np.pi/180, 30, 30, 20)#threshold 30-->10
            line_arr = np.squeeze(line_arr)
            slope_degree = (np.arctan2(line_arr[:,1] - line_arr[:,3], line_arr[:,0] - line_arr[:,2]) * 180) / np.pi

            # horizon slope limit
            line_arr = line_arr[np.abs(slope_degree)<160]
            slope_degree = slope_degree[np.abs(slope_degree)<160]
            # vertical slope limit
            line_arr = line_arr[np.abs(slope_degree)>95]
            slope_degree = slope_degree[np.abs(slope_degree)>95]
            # except filltered lines
            L_lines, R_lines = line_arr[(slope_degree>0),:], line_arr[(slope_degree<0),:]
            temp = np.zeros((frame.shape[0], frame.shape[1], 3), dtype=np.uint8)
            L_lines, R_lines = L_lines[:,None], R_lines[:,None]
            # drawing lines
            left_fit_line = get_fitline(frame, L_lines)
            right_fit_line = get_fitline(frame, R_lines)

            draw_fit_line(temp, left_fit_line)
            draw_fit_line(temp, right_fit_line)
            # draw left line start point and end point
            #print("L",left_fit_line)
            
            car_line = weighted_img(temp, frame)
    
            left_x = left_fit_line[2]
            right_x = right_fit_line[2]
            middle_x = (left_x + right_x)/2
            print(middle_x)

            #line_degree =(float(line_data[3])-line_data[1])/(line_data[0]-line_data[2])
            #print(line_degree)	
            for i in pts1:
                cv2.circle(car_line, tuple(i), 5, (0, 255, 0), -1)
            for i in pts3:
                cv2.circle(car_line, tuple(i), 5, (255 ,0, 0), -1)
            cv2.imshow("frame", frame)
            #cv2.imshow("BEV", BEV)
            cv2.imshow("ROI2", ROI_img2)
            # cv2.imshow("hsv", hsv)
            # cv2.imshow("mask", mask)
            # cv2.imshow("gray", gray_img)
            # cv2.imshow("blur", blur_img)
            #cv2.imshow("canny", canny_img)
            #cv2.imshow("hough", hough_img)
            cv2.imshow("car_line", car_line)
            # cv2.imshow("colored_frame", res)

            self.camera_pub.publish(middle_x)
            self.camera_cor_pub.publish(0)
        except: # right line --> curve
            #find slope
            line_arr = hough_lines(ROI_img, 1, 1*np.pi/180, 30, 30, 20)#threshold 30-->10
            line_arr = np.squeeze(line_arr)
            slope_degree = (np.arctan2(line_arr[:,1] - line_arr[:,3], line_arr[:,0] - line_arr[:,2]) * 180) / np.pi
            
    
            # horizon slope limit
            line_arr = line_arr[np.abs(slope_degree)<160]
            slope_degree = slope_degree[np.abs(slope_degree)<160]
            # vertical slope limit
            line_arr = line_arr[np.abs(slope_degree)>95]
            slope_degree = slope_degree[np.abs(slope_degree)>95]
            # except filltered lines
            L_lines, R_lines = line_arr[(slope_degree>0),:], line_arr[(slope_degree<0),:]
            temp = np.zeros((frame.shape[0], frame.shape[1], 3), dtype=np.uint8)
            L_lines, R_lines = L_lines[:,None], R_lines[:,None]
            # drawing lines
            right_fit_line = get_fitline(frame, R_lines)

            draw_fit_line(temp, right_fit_line)
            # draw left line start point and end point
            #print("L",left_fit_line)
            
            car_line = weighted_img(temp, frame)
    
            
            right_x = right_fit_line[2]
           
            print(right_x)

            #line_degree =(float(line_data[3])-line_data[1])/(line_data[0]-line_data[2])
            #print(line_degree)	
            for i in pts1:
                cv2.circle(car_line, tuple(i), 5, (0, 255, 0), -1)

            
            cv2.imshow("car_line_exceop", car_line)
            # cv2.imshow("colored_frame", res)
            print("execept",right_x)
            left_setting_line = 120 #120
            middle_data_except = (right_x+left_setting_line)/2
            
            self.camera_pub.publish(middle_data_except)
            self.camera_cor_pub.publish(1)
            print("no line", middle_data_except)

        cv2.imshow("frame1", frame) 
        
        #self.camera_pub.publish(middle_x)
	#self.camera_pub.publish(-600)
        #print(line_data)
        cv2.waitKey(1)
        #self.stop_pub.publish(1)


def run():
    rospy.init_node('camera_test_copy5', anonymous=True)
    lidar_receiver_a = lidar_receiver()
    rospy.spin()

if __name__ == "__main__":
    run()
