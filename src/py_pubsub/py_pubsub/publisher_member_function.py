# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.pub = self.create_publisher(String, 'topic', 10)
#        timer_period = 0.5  # seconds
#        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 1
        
        self.max_iterations = 25
        self.bridge = CvBridge()
        self.video='/home/khadija/Downloads/video.mp4'
        self.retrieve_frames(self.video)
    
   #	def timer_callback(self):
   #     msg = String()
   #     msg.data = 'The gate is in square number %d' % self.i
   #     self.publisher_.publish(msg)
   #    self.get_logger().info('Publishing: "%s"' % msg.data)
   #     self.i += 1
   #    if self.i == 10:
   #         self.i = 0

            
            
    def retrieve_frames(self, video):
        iter = 0
        cap = cv2.VideoCapture(video)
        while cap.isOpened():
            ret, frame = cap.read()
            if(frame is None):
                continue
            if ret:
                #cv2.imshow('frame', frame)
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                lower_orange = np.array([0, 150, 95])
                upper_orange = np.array([14, 255, 255])
                mask = cv2.inRange(hsv, lower_orange, upper_orange)
                #cv2.imshow('mask',mask)
                kernel = np.ones((4, 4), np.uint8)
                erode = cv2.erode(mask, kernel, iterations=1)
                dilation = cv2.dilate(erode, kernel, iterations=4)
                #cv2.imshow('dilation',dilation)
                contours, hierarchy = cv2.findContours(dilation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                my_list = [] 
                l = [[0,0,192,150,'1'],
                    [193,150,436,0,'2'],
                    [437,150,639,0,'3'],
                    [0,149, 193, 350,'4'],
                    [194,151,438,349,'5'],
                    [437,152,639,351,'6'],
                    [0,350,193,477,'7'],
                    [194,351,437,478,'8'],
                    [436,351,638,478,'9']]
                x = 0 # for sqaure indicator
                for contour in contours:
                    (x, y, w, h) = cv2.boundingRect(contour)
                    if cv2.contourArea(contour) < 400:
                        continue
                    cv2.drawContours(frame, contour,-1, (0, 225, 0), 3)
                    my_list.append([x, y, w, h])
                my_list.sort()
                if len(contours)==3:
                    cv2.rectangle(frame, (my_list[0][0], my_list[0][1]), (my_list[2][0] + my_list[2][2], my_list[2][1] + my_list[2][3]),(0, 0, 255), 1)
                    x= int(my_list[1][0] - (my_list[0][0] + my_list[0][2]))
                    y= int(my_list[2][0] -(my_list[1][0] + my_list[1][2]))
                    if x<(y+50):
                        cv2.rectangle(frame, (my_list[0][0], my_list[0][1]),
                                    (my_list[1][0] + my_list[1][2], my_list[1][1] + my_list[1][3]), (0, 255, 255), 1)
                        cv2.putText(frame, "{}".format("Bonus"), (int(my_list[0][0])+20, int(my_list[0][1])-10), cv2.FONT_ITALIC, 2, (0, 255, 255))
                    else:
                        cv2.rectangle(frame, (my_list[1][0], my_list[1][1]),
                                    (my_list[2][0] + my_list[2][2], my_list[2][1] + my_list[2][3]), (0, 255, 255), 1)
                        
                        cv2.putText(frame, "{}".format("Bonus"), (int(my_list[1][0]) + 20, int(my_list[1][1]) - 10),
                                    cv2.FONT_ITALIC, 2, (0, 255, 255))
                        end_x = my_list[2][0] + my_list[2][2]
                        end_y = my_list[2][1] + my_list[2][3]
                        if(l[0][0]<=my_list[1][0]<=l[0][2] and l[0][1]<=end_y<=l[0][3]):
                            x = l[0][4]
                        else if (l[1][0]<=my_list[1][0]<=l[1][2] and l[1][1]<=end_y<=l[1][3]):
                            x = l[1][4]
                        else if(l[2][0]<=my_list[1][0]<=l[2][2] and l[2][1]<=end_y<=l[2][3]):
                            x = l[2][4]
                        else if(l[3][0]<=my_list[1][0]<=l[3][2] and l[3][1]<=end_y<=l[3][3]):
                            x = l[3][4]
                        else if(l[4][0]<=my_list[1][0]<=l[4][2] and l[4][1]<=end_y<=l[4][3]):
                            x = l[4][4]
                        else if(l[5][0]<=my_list[1][0]<=l[5][2] and l[5][1]<=end_y<=l[5][3]):
                            x = l[5][4]
                        else if(l[6][0]<=my_list[1][0]<=l[6][2] and l[6][1]<=end_y<=l[6][3]):
                            x = l[6][4]
                        else if(l[7][0]<=my_list[1][0]<=l[7][2] and l[7][1]<=end_y<=l[7][3]):
                            x = l[7][4]
                        else if(l[8][0]<=my_list[1][0]<=l[8][2] and l[8][1]<=end_y<=l[8][3]):
                            x = l[8][4]
                        else if(l[9][0]<=my_list[1][0]<=l[9][2] and l[9][1]<=end_y<=l[9][3]):
                            x = l[9][4]
                        
                #cv2.imshow('contour', frame)
                #if cv2.waitKey(10) & 0xFF == ord('q'):
                #    break
                img = frame
                height = img.shape[0]
                width = img.shape[1]
                
                for x in range(height+100,height+1000,height+250):
                    cv2.line(img, (int(x/3), 0),(int(x/3), img.shape[0]), (0, 255, 0), 1, 1)
                for y in range(150, width+1000, 200):
                    cv2.line(img, pt1=(0, y), pt2=(width, y), color=(0,255,0), thickness=1)
                
                image_message = self.bridge.cv2_to_imgmsg(img, encoding="passthrough")

                image_message = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
                cv2.imshow('contour', frame)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
                msg = String()
                msg.data = 'The gate is in square number %d' % self.x
                self.pub.publish(msg)
                self.get_logger().info('Publishing: "%s"' % msg.data)
                self.i += 1
                if self.i == 10:
                    self.i = 0
                if(iter == self.max_iterations):
                    break
                iter += 1
                continue
            break
       # self.retrieve_frames(self.video)



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

