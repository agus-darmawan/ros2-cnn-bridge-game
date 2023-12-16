# usr/bin/env python3
import sys
import cv2 as cv
import numpy as np
import copy
from loguru import logger
from keras.models import load_model


# ------------------ ROS2 ------------------
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        logger.remove(0)
        logger.add(sys.stderr, format = "<red>[{level}]</red> <green>{message}</green> ", colorize=True)
        self.cap = cv.VideoCapture(0)
        self.image_pub = self.create_publisher(Image, 'image', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()
        self.model = load_model('model.h5')
        self.started_time = self.get_clock().now().to_msg().sec
        self.classes = [
            "2 spades", "3 spades", "4 spades", "5 spades", "6 spades", "7 spades", "8 spades", "9 spades", "10 spades", "J spades", "Q spades", "K spades", "A spades",
            "2 hearts", "3 hearts", "4 hearts", "5 hearts", "6 hearts", "7 hearts", "8 hearts", "9 hearts", "10 hearts", "J hearts", "Q hearts", "K hearts", "A hearts",
            "2 clubs", "3 clubs", "4 clubs", "5 clubs", "6 clubs", "7 clubs", "8 clubs", "9 clubs", "10 clubs", "J clubs", "Q clubs", "K clubs", "A clubs",
            "2 diamonds", "3 diamonds", "4 diamonds", "5 diamonds", "6 diamonds", "7 diamonds", "8 diamonds", "9 diamonds", "10 diamonds", "J diamonds", "Q diamonds", "K diamonds", "A diamonds",
        ]

    def masking_card(self,img):
        '''
        Parameters
        ----------
        img : np.ndarray
            BGR image
        Returns
        -------
        img : np.ndarray
            HSV image
        '''
        img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        lower = np.array([0, 0, 0])
        upper = np.array([255, 255, 255])
        mask  = cv.inRange(img, lower, upper)
        mask  = cv.bitwise_and(img, img, mask=mask)
        card =  cv.bitwise_xor(img, mask)
        return cv.cvtColor(card, cv.COLOR_HSV2BGR)
    
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

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            logger.info('Frame readed')
            start_time = self.get_clock().now().to_msg().sec
            card = self.masking_card(frame)
            card = cv.cvtColor(card, cv.COLOR_BGR2GRAY)
            treshold_image = cv.adaptiveThreshold(card,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY,71,10)

            len_object, label_image, stats, centroids = cv.connectedComponentsWithStats(treshold_image, 4, cv.CV_32S)
            is_card = []
            for i in range(len_object):
                hw = stats[i][:2]
                if (100<hw[0]<300 and 300<hw[1]<500):
                    is_card.append(i)
                    logger.info(f'Card detected with size {hw}')
                    logger.info(f'Card length: {len(is_card)}')
            
            for i in range(is_card):
                x,y,w,h = stats[i][0],stats[i][1],stats[i][2],stats[i][3]
                cv.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
                cv.putText(frame, str(i), (x,y), cv.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2, cv.LINE_AA)
                break
            for i in range(is_card):
                x,y,w,h = stats[i][0],stats[i][1],stats[i][2],stats[i][3]
                card = frame[y:y+h, x:x+w]
                cv.imshow('card', card)

                card = cv.cvtColor(card, cv.COLOR_GRAY2BGR)
                object = []
                img = cv.resize(card,(128,128))
                img = np.asarray(img)/255
                img = img.astype('float32')
                object.append(img)
                object = np.array(object)
                object = object.astype('float32')

                hs = self.model.predict(object,verbose = 0)
                n = np.max(np.where(hs== hs.max()))
                self.draw_text(frame, f'{self.classes[n]} {"{:.2f}".format(hs[0,n])}', (x-95,y-11))
                frame = self.show_fps(frame, 1 / (self.get_clock().now().to_msg().sec - start_time))
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
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