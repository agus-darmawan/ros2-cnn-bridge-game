# usr/bin/env python3
import sys
from loguru import logger
import random

# ------------------ ROS2 ------------------
import rclpy
from rclpy.node import Node
from game_handle.utils.card_data import CardData
from game_handle.utils.game_state import GameState
from game_handle.utils.game_logic import GameLogic

class GameHandleNode(Node):
    def __init__(self):
        super().__init__('game_handle_node')
        logger.remove(0)
        logger.add(sys.stderr, format = "<red>[{level}]</red> <green>{message}</green> ", colorize=True)
        self.card_data = CardData(self)
        self.game_logic = GameLogic(self)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.state = GameState.IDLE
        self.is_new_game = [True,True]
    
    def xor(self, a, b):
        return (a and not b) or (not a and b)

    def timer_callback(self):
        if self.state == GameState.IDLE:
            logger.info('STATE: IDLE')
            logger.info('Waiting for user to press start button')
            # TODO: Wait for user to press start button
            pass
        else:
            if self.state == GameState.WAITING_FOR_COMPUTER_CARDS:
                logger.info('STATE: WAITING_FOR_COMPUTER_CARDS')
                for i in self.card_data.card_class:                    
                    if self.game_logic.is_new_card(i) and len(self.card_data.computer_cards_class) <= 5:
                        self.card_data.all_cards_class.append(i.class_name)
                        self.card_data.computer_cards_class.append(i.class_name)
                        self.card_data.computer_cards_number.append(i.number)
                        self.card_data.computer_cards_label.append(i.label)
                        logger.info(f'Computer card: {i.label}')
                    elif len(self.card_data.computer_cards_class) > 5:
                        self.state = GameState.WAITING_FOR_USER_CARDS
                        break
                
            elif self.state == GameState.WAITING_FOR_USER_CARDS:
                logger.info('STATE: WAITING_FOR_USER_CARDS')
                for i in self.card_data.card_class:
                    if self.game_logic.is_new_card(i) and len(self.card_data.user_cards_class) <= 5:
                        self.card_data.all_cards_class.append(i.class_name)
                        self.card_data.user_cards_class.append(i.class_name)
                        self.card_data.user_cards_number.append(i.number)
                        self.card_data.user_cards_label.append(i.label)
                        logger.info(f'User card: {i.label}')
                    elif len(self.card_data.user_cards_class) > 5:
                        self.state = GameState.WAITING_FOR_OPENING_CARD
                        break

            elif self.state == GameState.WAITING_FOR_OPENING_CARD:
                logger.info('STATE: WAITING_FOR_OPENING_CARD')
                for i in self.card_data.card_class:
                    if self.game_logic.is_new_card(i):
                        self.card_data.all_cards_class.append(i.class_name)
                        self.card_data.opening_card_class = i.class_name
                        self.card_data.opening_card_number = i.number
                        self.card_data.opening_card_label = i.label
                        logger.info(f'Opening card: {i.label}')
                        self.state = random.choice([GameState.COMPUTER_TURN, GameState.USER_TURN])
                        break

            elif self.state == GameState.USER_TURN:
                logger.info('STATE: USER_TURN')
                if (self.xor(self.game_logic.is_same_label(self.card_data.user_cards_label, self.card_data.opening_card_label)),(self.game_logic.is_same_label(self.card_data.user_cards_label, self.card_data.computer_main_card_label))):
                    self.card_data.computer_main_card_class = self.card_data.card_class
                    self.card_data.computer_main_card_number = self.card_data.card_number
                    self.card_data.computer_main_card_label = self.card_data.card_label
                    self.card_data.user_cards_class.remove(self.card_data.card_class)
                    self.card_data.user_cards_number.remove(self.card_data.card_number)
                    self.card_data.user_cards_label.remove(self.card_data.card_label)
                    if(len(self.card_data.computer_main_card_class) == 0):
                        self.state = GameState.COMPUTER_TURN
                    else:
                        self.state = GameState.CALCULATE_SCORE
                else:
                    self.state = GameState.GIVE_USER_CARD
                        
            elif self.state == GameState.COMPUTER_TURN:
                logger.info('STATE: COMPUTER_TURN')
                if (self.xor(self.game_logic.is_same_label(self.card_data.computer_cards_label, self.card_data.opening_card_label)),(self.game_logic.is_same_label(self.card_data.computer_cards_label, self.card_data.user_main_card_label))):
                    same_label_num = 0
                    idx = 0
                    idxarr = []
                    for i in self.card_data.computer_cards_label:
                        if (self.xor(i == self.card_data.opening_card_label),(i == self.card_data.user_main_card_label)):
                            same_label_num += 1
                            idx = i 
                            idxarr.append(i) 
                    
                    if same_label_num == 1: 
                        self.card_data.computer_main_card_class = self.card_data.computer_cards_class[idx]
                        self.card_data.computer_main_card_number = self.card_data.computer_cards_number[idx]
                        self.card_data.computer_main_card_label = self.card_data.computer_cards_label[idx]
                        self.card_data.computer_cards_class.remove(self.card_data.computer_cards_class[idx])
                        self.card_data.computer_cards_number.remove(self.card_data.computer_cards_number[idx])
                        self.card_data.computer_cards_label.remove(self.card_data.computer_cards_label[idx])
                    
                    elif same_label_num > 1:
                        for i in idxarr:
                            if self.card_data.computer_cards_number[i] < self.card_data.computer_cards_number[idx]:
                                idx = i
                        self.card_data.computer_main_card_class = self.card_data.computer_cards_class[idx]
                        self.card_data.computer_main_card_number = self.card_data.computer_cards_number[idx]
                        self.card_data.computer_main_card_label = self.card_data.computer_cards_label[idx]
                        self.card_data.computer_cards_class.remove(self.card_data.computer_cards_class[idx])
                        self.card_data.computer_cards_number.remove(self.card_data.computer_cards_number[idx])
                        self.card_data.computer_cards_label.remove(self.card_data.computer_cards_label[idx])
                else:
                    self.state = GameState.GIVE_COMPUTER_CARD
                    
            elif self.state == GameState.GIVE_COMPUTER_CARD:
                logger.info('STATE: GIVE_COMPUTER_CARD')
                if not (self.xor(self.game_logic.is_same_label(self.card_data.computer_cards_label, self.card_data.opening_card_label)),(self.game_logic.is_same_label(self.card_data.computer_cards_label, self.card_data.user_main_card_label))):
                    if not self.game_logic.is_inside(self.card_data.card_class,self.card_data.all_cards_class):
                        self.card_data.all_cards_class.append(self.card_data.card_class)
                        self.card_data.computer_cards_class.append(self.card_data.card_class)
                        self.card_data.computer_cards_number.append(self.card_data.card_number)
                        self.card_data.computer_cards_label.append(self.card_data.card_label)
                else:
                    self.state = GameState.COMPUTER_TURN

            elif self.state == GameState.GIVE_USER_CARD:
                logger.info('STATE: GIVE_USER_CARD')
                # do same as GIVE_COMPUTER_CARD
                if not (self.xor(self.game_logic.is_same_label(self.card_data.computer_cards_label, self.card_data.opening_card_label)),(self.game_logic.is_same_label(self.card_data.computer_cards_label, self.card_data.user_main_card_label))):
                    if not self.game_logic.is_inside(self.card_data.card_class,self.card_data.all_cards_class):
                        self.card_data.all_cards_class.append(self.card_data.card_class)
                        self.card_data.user_cards_class.append(self.card_data.card_class)
                        self.card_data.user_cards_number.append(self.card_data.card_number)
                        self.card_data.user_cards_label.append(self.card_data.card_label)
                else:
                    self.state = GameState.USER_TURN
            
            elif self.state == GameState.CALCULATE_SCORE:
                logger.info('STATE: CALCULATE_SCORE')
                if(self.card_data.computer_main_card_number > self.card_data.user_main_card_number):
                    self.state = GameState.COMPUTER_WON
                else:
                    self.state = GameState.USER_WON
                
            elif self.state == GameState.COMPUTER_WON:
                logger.info('STATE: COMPUTER_WON')
                if(len(self.card_data.computer_cards_class) == 0):
                    logger.info('Computer WIN THE GAME')
                    self.state = GameState.IDLE
                else:
                    self.state.COMPUTER_TURN
            
            elif self.state == GameState.USER_WON:
                logger.info('STATE: USER_WON')
                if(len(self.card_data.user_cards_class) == 0):
                    logger.info('User WIN THE GAME')
                    self.state = GameState.IDLE
                else:
                    self.state.USER_TURN