#!/usr/bin/env micropython
# coding: utf-8

from multiprocessing import Pool
import sys
import threading
from time import sleep
from ev3dev2.motor import OUTPUT_A, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor import INPUT_1, INPUT_3, INPUT_4
from ev3dev2.motor import MoveSteering, MoveTank
from ev3dev2.sensor.lego import UltrasonicSensor, LightSensor


class ControlThread(threading.Thread):
    def __init__(self, function, command):
        super().__init__()
        self.function = function
        self.command = command

    def run(self):
        try:
            ret = self.function()
            return ret
        except Exception as e:
            sys.stdout.write("\033[0;31m[ERROR]: An error occured while executing '%s'\033[0;0m" % e)


class Robot:
    def __init__(self):
        self.m_left = OUTPUT_B   # Motor left
        self.m_right = OUTPUT_A  # Motor right
        self.s_us = INPUT_1      # Sensor Ultrasonic
        self.sl_left = INPUT_3   # Sensor Light left
        self.sl_right = INPUT_4  # Sensor Right left
        self.mMT = MoveTank(self.m_left, self.m_right)  # move with 2 motors
        self.mMS = MoveSteering(self.m_left, self.m_right)  # move on position
        self.sUS = UltrasonicSensor(self.s_us)
        
    def move_forward(self, speed):
        pass


if __name__ == '__main__':
    pass
