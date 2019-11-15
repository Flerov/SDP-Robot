#!/usr/bin/env python3
# coding: utf-8

from multiprocessing import Pool, Process, Value
import sys
import os
from time import sleep
import threading
from time import sleep
from ev3dev2.motor import OUTPUT_A, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor import INPUT_1, INPUT_3, INPUT_4
from ev3dev2.motor import MoveSteering, MoveTank
from ev3dev2.sensor.lego import UltrasonicSensor, LightSensor


class ControlThread(threading.Thread):
    def __init__(self, function, arg1=None, arg2=None, arg3=None):
        super().__init__()
        self.function = function
        self.len_args = 0
        if arg1 is not None:
            self.arg1 = arg1
            self.len_args += 1
        if arg2 is not None:
            self.arg2 = arg2
            self.len_args += 1
        if arg3 is not None:
            self.arg3 = arg3
            self.len_args += 1

    def run(self):
        #try:
        if self.len_args == 1:
            ret = self.function(self.arg1)
        if self.len_args == 2:
            ret = self.function(self.arg2)
        if self.len_args == 3:
            ret = self.function(self.arg3)
        if self.len_args == 0:
            ret = self.function()
        return ret
        #except Exception as e:
        #    sys.stdout.write("\033[0;31m[ERROR]: An error occured while executing '%s'\033[0;0m" % e)


class RobotHandle:
    def __init__(self, *args_module_name, load_modules=False):
        self.modules = [str(x) for x in args_module_name if str(x) in os.listdir(os.getcwd())]  # convert all given module_names to string and append to list
        self.loaded_modules = []
        self.not_loaded_modules = []
        if load_modules:
            self.load_modules(self.modules)

    def remove_module(self, module_name):
        if module_name in self.modules:
            self.modules.remove(module_name)
            return True
        else:
            return False

    def add_module(self, module_name):
        module_name = str(module_name)
        if len(module_name) < 4:
            print("[NOT-Valid]'%s' is to small, give it a more valuable name!" % module_name)
            return False
        else:
            if module_name+'.py' in os.listdir(os.getcwd()):
                self.modules.append(module_name)
                return True

    def execute_function(self, module, function):
        try:
            if module in self.modules:
                module.exc(function)
                return True
            else:
                print("[ERROR] %s is not in list of modules. Use add_module to add the module!")
                return False
        except Exception as e:
            print("[ERROR] Occured while executing [%s] in [%s]" % function, module)


class RobotControl(RobotHandle):
    def __init__(self, *args_module_name):
        super().__init__(*args_module_name)
        self.m_left = OUTPUT_B   # Motor left
        self.m_right = OUTPUT_A  # Motor right
        self.s_us = INPUT_1      # Sensor Ultrasonic
        self.sl_left = INPUT_3   # Sensor Light left
        self.sl_right = INPUT_4  # Sensor Right left
        self.mMT = MoveTank(self.m_left, self.m_right)  # move with 2 motors
        self.mMS = MoveSteering(self.m_left, self.m_right)  # move on position
        self.sUS = UltrasonicSensor(self.s_us)
        #self.distance_cm = Value('d', 0.0)

    def run(self):
        #pool_detect_danger = Process(target=self.detect_danger, args=self.distance_cm)
        #print('RUN: Distance: ', self.distance_cm.value)
        movement = ControlThread(self.mMT.on, 15, 15)
        thread = ControlThread(self.detect_danger)
        movement.start()
        thread.start()
        print("Weiter")
        return True

    def detect_danger(self):
        while True:
            d = self.sUS.distance_centimeters
            print('dd: distance:', d)
            if d < 5:
                self.mMT.stop()
            sleep(.500)


if __name__ == '__main__':
    wall_e = RobotControl().run()
