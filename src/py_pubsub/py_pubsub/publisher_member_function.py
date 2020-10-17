
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from concurrent.futures import ThreadPoolExecutor
import os
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.executors import Executor




class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.pub = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.max_iterations = 25
        self.bridge = CvBridge()
        self.video='/home/khadija/Downloads/video.mp4'
        self.frames = []
        self.capture(self.video,self.frames)
        self.indx = 0

    def timer_callback(self):

        gt = self.retrieve_frames(self.indx)

        msg = String()
        msg.data = 'The gate is in square number %d' % gt
        self.pub.publish(msg)
        self.get_logger().info('logger Publishing: "%s"' % msg.data)
        msg.data = ""

        self.indx+=1


    def capture(self,video,frames):
        cap = cv2.VideoCapture(video)
        while cap.isOpened():
            ret, frame = cap.read()
            if(frame is None):
                self.get_logger().info('logger Done reading')
                break
            if ret:
                self.get_logger().info('logger Adding frame')
                self.frames.append(frame)
                continue
            
            
    def retrieve_frames(self,indx):

        if indx >= len(self.frames):
            return -1

        frame = self.frames[indx]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_orange = np.array([0, 150, 95])
        upper_orange = np.array([14, 255, 255])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        kernel = np.ones((4, 4), np.uint8)
        erode = cv2.erode(mask, kernel, iterations=1)
        dilation = cv2.dilate(erode, kernel, iterations=4)
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
                elif (l[1][0]<=my_list[1][0]<=l[1][2] and l[1][1]<=end_y<=l[1][3]):
                    x = l[1][4]
                elif(l[2][0]<=my_list[1][0]<=l[2][2] and l[2][1]<=end_y<=l[2][3]):
                    x = l[2][4]
                elif(l[3][0]<=my_list[1][0]<=l[3][2] and l[3][1]<=end_y<=l[3][3]):
                    x = l[3][4]
                elif(l[4][0]<=my_list[1][0]<=l[4][2] and l[4][1]<=end_y<=l[4][3]):
                    x = l[4][4]
                elif(l[5][0]<=my_list[1][0]<=l[5][2] and l[5][1]<=end_y<=l[5][3]):
                    x = l[5][4]
                elif(l[6][0]<=my_list[1][0]<=l[6][2] and l[6][1]<=end_y<=l[6][3]):
                    x = l[6][4]
                elif(l[7][0]<=my_list[1][0]<=l[7][2] and l[7][1]<=end_y<=l[7][3]):
                    x = l[7][4]
                elif(l[8][0]<=my_list[1][0]<=l[8][2] and l[8][1]<=end_y<=l[8][3]):
                    x = l[8][4]
                elif(l[9][0]<=my_list[1][0]<=l[9][2] and l[9][1]<=end_y<=l[9][3]):
                    x = l[9][4]
                
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

        return x
                
   

def main(args=None):

    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    minimal_subscriber= MinimalSubscriber()

    executor = PriorityExecutor()
    executor.add_high_priority_node(minimal_subscriber)
    executor.add_node(minimal_publisher)
    executor.spin()
#-------------------------------------------------------------------------------
  #  rclpy.init(args=args)

#    minimal_publisher = MinimalPublisher()
 #   minimal_subscriber= MinimalSubscriber()

  #  rclpy.spin(minimal_subscriber)

   # rclpy.spin(minimal_publisher)

    #minimal_publisher.destroy_node()
    #MinimalSubscriber.destroy_node()
    #rclpy.shutdown()


#-------------------------------------------------------------------------------
    #rclpy.init(args=args)

    #self.minimal_publisher = MinimalPublisher()
 #   #rrospy.init_node('minimal_publisher', anonymous=True)
 #   self.minimal_publisher.retrieve_frames()
 #   print('HEREEEEEEEEEEEE')
 #   rospy.Subscriber(String, 'topic', JointState, listener_callback)
    
    #rospy.spin(minimal_publisher)


if __name__ == '__main__':
    main()



class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback,10)
        self.get_logger().info('subzift')

    def listener_callback(self, msg):
        if len(msg.data) < 9:
            self.get_logger().info('I heard: "%s"' % msg.data)





class PriorityExecutor(Executor):

    def __init__(self):
        super().__init__()
        self.high_priority_nodes = set()
        self.hp_executor = ThreadPoolExecutor(max_workers=os.cpu_count() or 4)
        self.lp_executor = ThreadPoolExecutor(max_workers=1)

    def add_high_priority_node(self, node):
        self.high_priority_nodes.add(node)
        # add_node inherited
        self.add_node(node)

    def spin_once(self, timeout_sec=None):
        # wait_for_ready_callbacks yields callbacks that are ready to be executed
        try:
            handler, group, node = self.wait_for_ready_callbacks(timeout_sec=timeout_sec)
        except StopIteration:
            pass
        else:
            if node in self.high_priority_nodes:
                self.hp_executor.submit(handler)
            else:
                self.lp_executor.submit(handler)
