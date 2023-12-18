# ------------------ ROS2 ------------------
import rclpy
from loguru import logger
from rclpy.node import Node
from dar_msgs.msg import BoundingBox, BoundingBoxArray


class CardData():
    def __init__(self,node : Node):
        self.node = node
        self.bbx_sub = self.node.create_subscription(BoundingBoxArray, 'vision/bounding_boxes', self.bbx_callback, 10)
        self.card_label : [str] = []
        self.card_number : [int] = []
        self.card_class : [str] = []
        self.card : [BoundingBox] = []

        self.all_cards_class : [str] = []

        self.computer_cards_class : [str] = []
        self.computer_cards_number : [int] = []
        self.computer_cards_label : [str] = []

        self.user_cards_class : [str] = []
        self.user_cards_number : [int] = []
        self.user_cards_label : [str] = []

        self.opening_card_class : str = ''
        self.opening_card_number : int = 0
        self.opening_card_label : str = ''

        self.computer_main_card_class : str = ''
        self.computer_main_card_number : int = 0
        self.computer_main_card_label : str = ''

        self.user_main_card_class : str = ''
        self.user_main_card_number : int = 0
        self.user_main_card_label : str = ''
    def bbx_callback(self, msg : BoundingBoxArray):
        for bbx in msg.bounding_boxes:
            self.card_label.append(bbx.label)
            self.card_number.append(bbx.number)
            self.card_class.append(bbx.class_name)
            self.card.append(bbx)