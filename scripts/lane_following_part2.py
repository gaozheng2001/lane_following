#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:

        def __init__(self):

                self.bridge = cv_bridge.CvBridge()

                self.image_sub = rospy.Subscriber('camera/image',
                        Image, self.image_callback)

                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=10)

                self.twist = Twist()

        def image_callback(self, msg):
                # import pdb;pdb.set_trace()
                cameraMatrix = numpy.matrix(numpy.array([[-3.313061098815712, 0.0, 160.5],
                                              [0.0, -3.313061098815712, 120.5],
                                              [0.0, 0.0, 1.0]]))
                distCoeffs = numpy.array([0.0, 0.0, 0.0, 0.0, 0.0])
                R = numpy.matrix(numpy.array([[1, 0, 1],
                                              [0, 0, 0],
                                              [0, -1, 0]]))
                T = numpy.matrix(numpy.array([0, 1, 1]).reshape(3, 1))
                n = numpy.matrix(numpy.array([0, -1, 0]).reshape(3, 1))
                d = 0.115
                #H = numpy.dot(K, numpy.dot(R - numpy.dot(T, n.reshape(1, 3))/d, K.I))
               
                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


                lower_yellow = numpy.array([ 10, 10, 10])
                upper_yellow = numpy.array([255, 255, 250])

                lower_white = numpy.array([0, 0, 221])
                upper_white = numpy.array([180, 43, 255])
               
                mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
                mask2 = cv2.inRange(hsv, lower_white, upper_white)
                '''
print(cx12, cy12, '\n', cx22, cy22, '\n', cx13, cy13, '\n', cx23, cy23)
(33, 229, '\n', 279, 229, '\n', 131, 144, '\n', 186, 145)
                '''
                h, w, d = image.shape

                #src_list = [[33, 229], [279, 229], [186, 144], [131, 145]]
                src_list = [[100, 163], [189, 162], [4, 239], [264, 239]]
                #dst_list = [[110, 230], [210, 230], [210, 5], [110, 5]]
                dst_list = [[110, 5], [210, 5], [110, 235], [210, 235]]
                H, _ = cv2.findHomography(numpy.array(src_list).reshape(-1,1,2),
                                       numpy.array(dst_list).reshape(-1,1,2), cv2.RANSAC, 5.0)
                image_homography = cv2.warpPerspective(image, H, (w, h))
                hsv_homo = cv2.cvtColor(image_homography, cv2.COLOR_BGR2HSV)
                hmask1 = cv2.inRange(hsv_homo, lower_yellow, upper_yellow)
                hmask2 = cv2.inRange(hsv_homo, lower_white, upper_white)
               
                search_top = int(2*h/3)
                # search_line = int(w/2)
                mask1[0:search_top, 0:w] = 0
                # mask1[0:h, search_line:w] = 0
                mask2[0:search_top, 0:w] = 0

                hmask1[0:1*h/3, 0:w] = 0
                hmask2[0:1*h/3, 0:w] = 0
             
                # mask2[0:h, 0:search_line] = 0

                M1 = cv2.moments(mask1)
                M2 = cv2.moments(mask2)

                hM1 = cv2.moments(hmask1)
                hM2 = cv2.moments(hmask2)
                def center(M):
                        return int(M['m10']/M['m00']), int(M['m01']/M['m00'])

                if M1['m00'] > 0:
                    cx1, cy1 = center(M1)
                    cx2, cy2 = center(M2)

                    hcx1, hcy1 = center(hM1)
                    hcx2, hcy2 = center(hM2)

                    # import pdb;pdb.set_trace()
                    fpt_x = (cx1 + cx2)/2
                    fpt_y = (cy1 + cy2)/2 + 2*h/3

                    hfpt_x = (hcx1 + hcx2)/2
                    hfpt_y = (hcy1 + hcy2)/2 + 2*h/3

                    cv2.circle(image, (cx1, cy1), 10, (0,255,255), -1)
                    cv2.circle(image, (cx2, cy2), 10, (255,255,255), -1)
                    cv2.circle(image, (fpt_x, fpt_y), 10, (128,128,128), -1)

                    cv2.circle(image_homography, (hcx1, hcy1), 10, (0,255,255), -1)
                    cv2.circle(image_homography, (hcx2, hcy2), 10, (255,255,255), -1)
                    cv2.circle(image_homography, (hfpt_x, hfpt_y), 10, (128,128,128), -1)

                    err = w/2 - fpt_x
                    herr = w/2 - hfpt_x

                    self.twist.linear.x = 0.3
                    self.twist.angular.z = (herr*90.0/160)/15
                    self.cmd_vel_pub.publish(self.twist)
                cv2.imshow("window", image)
                cv2.waitKey(1)
                cv2.imshow("Homography", image_homography)
                cv2.waitKey(1)
                # import pdb;pdb.set_trace()

rospy.init_node('lane_follower')
follower = Follower()
rospy.spin()
