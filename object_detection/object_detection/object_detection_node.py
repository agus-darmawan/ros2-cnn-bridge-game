# usr/bin/env python3
import sys
import cv2 as cv
import numpy as np
import copy
from loguru import logger
from keras.models import load_model
from utils.game_state import GameState

# ------------------ ROS2 ------------------
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dar_msgs.msg import BoundingBox, BoundingBoxArray
from std_msgs.msg import Int8


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        logger.remove(0)
        logger.add(sys.stderr, format = "<red>[{level}]</red> <green>{message}</green> ", colorize=True)
        self.cap = cv.VideoCapture(0)
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.state_sub = self.create_subscription(Int8, 'game/state', self.state_callback, 10)
        self.bbx_pub = self.create_publisher(BoundingBoxArray, 'vision/bounding_boxes', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()
        self.state  = GameState.IDLE
        self.model = load_model('model/model_card.h5')
        self.started_time = self.get_clock().now().to_msg().sec
        self.classes = [
            "2 spades", "3 spades", "4 spades", "5 spades", "6 spades", "7 spades", "8 spades", "9 spades", "10 spades", "J spades", "Q spades", "K spades", "A spades",
            "2 hearts", "3 hearts", "4 hearts", "5 hearts", "6 hearts", "7 hearts", "8 hearts", "9 hearts", "10 hearts", "J hearts", "Q hearts", "K hearts", "A hearts",
            "2 clubs", "3 clubs", "4 clubs", "5 clubs", "6 clubs", "7 clubs", "8 clubs", "9 clubs", "10 clubs", "J clubs", "Q clubs", "K clubs", "A clubs",
            "2 diamonds", "3 diamonds", "4 diamonds", "5 diamonds", "6 diamonds", "7 diamonds", "8 diamonds", "9 diamonds", "10 diamonds", "J diamonds", "Q diamonds", "K diamonds", "A diamonds",
        ]

    def state_callback(self, msg):
        self.state = GameState(msg.data)
    def show_fps(self, img, fps):
        """
        Parameters
        ----------
        img : np.ndarray
            BGR image
        fps : float
            fps value
        Returns
        -------
        img : np.ndarray
        """
        font = cv.FONT_HERSHEY_PLAIN
        line = cv.LINE_AA
        fps_text = 'FPS: {:.2f}'.format(fps)
        cv.putText(img, fps_text, (11, 20), font, 1.0, (32, 32, 32), 4, line)
        cv.putText(img, fps_text, (10, 20), font, 1.0, (240, 240, 240), 1, line)

        return img

    def draw_text(self, img, text, pos):
        '''
        Parameters
        ----------
        img : np.ndarray
            BGR image
        text : str
            text to be drawn
        pos : tuple
            position of text
        Returns
        -------
        img : np.ndarray
            BGR image
        '''
        font = cv.FONT_HERSHEY_SIMPLEX
        posf = pos
        fontScale = .7
        fontColor = (0, 0, 255)
        thickness = 2
        lineType = 2
        cv.putText(img, text,
                   posf,
                   font,
                   fontScale,
                   fontColor,
                   thickness,
                   lineType)
        return copy.deepcopy(img)

    def get_label(self, n):
        ''' 
        Parameters
        ----------
        n : int
            number of card
        Returns
        -------
        label : str
            '''
        if n < 13:
            return 'spades'
        elif n < 26:
            return 'hearts'
        elif n < 39:
            return 'clubs'
        else:
            return 'diamonds'
    
    def get_number(self, n):
        ''' 
        Parameters
        ----------
        n : int
            number of card
        Returns
        -------
        number : str
            '''
        print(n)
        if n < 13:
            return n
        elif n < 26:
            return n-13
        elif n < 39:
            return n-26
        else:
            return n-39
            

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            logger.info('Frame readed')
            self.start_time = self.get_clock().now().to_msg().sec
            card = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            treshold_image = cv.adaptiveThreshold(card,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY,71,10)

            len_object,_, stats,_ = cv.connectedComponentsWithStats(treshold_image, 4, cv.CV_32S)
            is_card = []
            for i in range(len_object):
                hw = stats[i,2:4]
                if (100<hw[0]<300 and 300<hw[1]<500):
                    is_card.append(i)
            
            for i in is_card:
                x,y,w,h = stats[i][0],stats[i][1],stats[i][2],stats[i][3]
                cv.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
                cv.putText(frame, str(i), (x,y), cv.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2, cv.LINE_AA)
            bbx_to_send = BoundingBoxArray()
            bbx_to_send.header.stamp = self.get_clock().now().to_msg()
            bbx_to_send.header.frame_id = 'camera'
            for i in is_card:
                x,y,w,h = stats[i][0],stats[i][1],stats[i][2],stats[i][3]
                card = frame[y:y+h, x:x+w]
                cv.imshow('card', card)
                object = []
                img = cv.resize(card,(128,128))
                img = np.asarray(img)/255
                img = img.astype('float32')                
                object.append(img)
                object = np.array(object)
                object = object.astype('float32')

                hs = self.model.predict(object,verbose = 0)
                n = np.max(np.where(hs== hs.max()))
                logger.info(f'Card detected: {self.classes[n]}')
                center_x = x + w/2
                center_y = y + h/2
                bbx = BoundingBox()
                bbx.label = self.get_label(n)
                bbx.number = int(self.get_number(n))
                bbx.class_name = self.classes[n]
                bbx.bounding_box.center.position.x = center_x
                bbx.bounding_box.center.position.y = center_y
                bbx.bounding_box.size_x = float(w)
                bbx.bounding_box.size_y = float(h)
                bbx_to_send.bounding_boxes.append(bbx)                
                self.draw_text(frame, f'{self.classes[n]} {"{:.2f}".format(hs[0,n])}', (x-95,y-11))
            fps = 1 / (self.get_clock().now().to_msg().sec - self.started_time)
            frame = self.show_fps(frame, fps)
            if self.state == GameState.IDLE:
                pass
            else:
                print('Game started')
                # bagi 2
                cv.line(frame, (0, 240), (640, 240), (0, 255, 0), 2)
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
                self.bbx_pub.publish(bbx_to_send)
            except CvBridgeError as e:
                logger.error(e)
            cv.imshow('frame', frame)
            cv.waitKey(1)
    

def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ObjectDetectionNode()
    rclpy.spin(object_detection_node)
    object_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()