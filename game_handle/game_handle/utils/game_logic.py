from rclpy.node import Node

class GameLogic():
    def __init__(self,node : Node):
        self.node = node
    
    def is_new_card(self,all_cards,card):
        if card in all_cards:
            return False
        else:
            return True

    def is_same_label(self):
        pass

    def is_inside(self):
        pass