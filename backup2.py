#!/usr/bin/env python3
# coding: utf-8

#from multiprocessing import Pool, Process, Value
import sys
import os
from time import sleep
import threading
from time import sleep
import argparse
from ev3dev2.motor import OUTPUT_A, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor import INPUT_1, INPUT_3, INPUT_4
from ev3dev2.motor import MoveSteering, MoveTank, SpeedPercent, LargeMotor
from ev3dev2.sensor.lego import UltrasonicSensor, LightSensor
from ev3dev2.motor import SpeedNativeUnits, follow_for_forever, LineFollowErrorLostLine, LineFollowErrorTooFast
import time


class ControlThread(threading.Thread):
    def __init__(self, function=None, arg1=None, arg2=None, arg3=None):
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
        elif self.len_args == 2:
            ret = self.function(self.arg1, self.arg2)
        elif self.len_args == 3:
            ret = self.function(self.arg1, self.arg2, self.arg3)
        elif self.len_args == 0:
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
        self.motor_left = LargeMotor(OUTPUT_B)
        self.motor_right = LargeMotor(OUTPUT_A)
        self.s_us = INPUT_1      # Sensor Ultrasonic
        self.sl_left = INPUT_3   # Sensor Light left
        self.sl_right = INPUT_4  # Sensor Right left
        self.mMT = MoveTank(self.m_left, self.m_right)  # move with 2 motors
        self.mMS = MoveSteering(self.m_left, self.m_right)  # move on position
        self.sUS = UltrasonicSensor(self.s_us)
        self.sLS_left = LightSensor(INPUT_3)
        self.sLS_right= LightSensor(INPUT_4)
        self.thread_detect_danger = ControlThread()
        self.thread_detect_light_intesity = ControlThread()
        self.stop_detect_light_intesity = False

        self.max_speed = self.mMT.max_speed
        #self.distance_cm = Value('d', 0.0)

    def run(self):
        #pool_detect_danger = Process(target=self.detect_danger, args=self.distance_cm)
        #print('RUN: Distance: ', self.distance_cm.value)
        #movement = ControlThread(self.mMT.on, 15, 15)
        #thread_detect_light_intesity = ControlThread(self.detect_light_intensitiy)
        self.thread_detect_light_intesity.function = self.detect_light_intensitiy
        self.thread_detect_danger.function = self.detect_danger
        thread_detect_danger = ControlThread(self.detect_danger)
        try:
            #self.mMT.on(15, 15)
            # 11.3, 0.05, 3.2
            self.follow_line(8.4, 0.05, 3.5, 'left', SpeedPercent(20))
            #self.follow_line(args.kp, args.ki, args.kd, 'left', SpeedPercent(10))
        except Exception:
            self.mMT.off()
            raise
        #self.mMT.off()
        #movement.start()
        self.thread_detect_light_intesity.start()
        self.thread_detect_danger.start()
        return True

    def detect_light_intensitiy(self):
        while True:
            if self.stop_detect_light_intesity:
                break
            if not args.light_mode:
                args.light_mode = 'ambient'
            if args.light_mode == 'ambient':
                light_intesitiy_left = self.sLS_left.ambient_light_intensity
                light_intesitiy_right = self.sLS_right.ambient_light_intensity
                sleep(.500)
            elif args.light_mode == 'reflected':
                light_intesitiy_left = self.sLS_left.reflected_light_intensity
                light_intesitiy_right = self.sLS_right.reflected_light_intensity
                sleep(.500)
            print('LEFT[{}]: {}'.format(args.light_mode, light_intesitiy_left))
            print('RIGHT[{}]: {}'.format(args.light_mode, light_intesitiy_right))

    def get_light_intensitiy(self, side: str):  # side = left, or right
        if args.light_mode == 'ambient':
            left = self.sLS_left.ambient_light_intensity
            right = self.sLS_right.ambient_light_intensity
        else:
            left = self.sLS_left.reflected_light_intensity
            right = self.sLS_right.reflected_light_intensity
        if side == 'left':
            return left
        elif side == 'right':
            return right

    def detect_danger(self):
        while True:
            d = self.sUS.distance_centimeters
            #print('dd: distance:', d)
            if d < 10:
                self.turn_on_wand()
                self.mMT.stop()
                #args.light_mode = 'reflected'
                pass
            sleep(.500)

    def turn_on_wand(self):
        self.mMS.on_for_degrees(-100, 20, parser.d)

    def now_stay_on_that_line(self):
        #  kp=3, ki=0, kd=0
        #self.mMT.cs = self.sl_left
        #self.mMT.follow_line(3, 0, 0, 'left', SpeedPercent(20))
        pass

    def define_target_light(self, motor: LightSensor, speed: SpeedPercent):
        speed_native_units = -speed.to_native_units(motor)

    def follow_line(self,
                    kp, ki, kd,
                    side,
                    speed=SpeedPercent(50),
                    target_light_intensity=None,
                    follow_left_edge=True,
                    white=60,
                    off_line_count_max=20,
                    sleep_time=0.01,
                    follow_for=follow_for_forever,
                    **kwargs
                    ):

        self.cs = self.sLS_left
        self.ls = self.sLS_right

        if target_light_intensity is None:
            time.sleep(1)
            target_light_intensity = self.get_light_intensitiy(side)
            buffed = target_light_intensity+2.5
            target_light_intensity_ls = self.ls.reflected_light_intensity-3

        integral = 0.0
        last_error = 0.0
        derivative = 0.0
        off_line_count = 0
        speed_native_units = -speed.to_native_units(self.motor_left)
        MAX_SPEED = SpeedNativeUnits(self.max_speed/2)
        switch_locked = False

        while follow_for(self, **kwargs):
            if args.light_mode == 'ambient':
                reflected_light_intensity = self.cs.ambient_light_intensity
                reflected_light_intensity_ls = self.ls.ambient_light_intensity
            else:
                reflected_light_intensity = self.cs.reflected_light_intensity
                reflected_light_intensity_ls = self.ls.reflected_light_intensity

            print("{:.3f}->{:.3f} | {:.3f}->{:.3f} | {:.3f} <-> {:.3f} | {}".format(reflected_light_intensity, target_light_intensity,
                                                                 reflected_light_intensity_ls, target_light_intensity_ls,
                                                                                    target_light_intensity,
                                                                                    target_light_intensity_ls,
                                                                                    follow_left_edge))
            if reflected_light_intensity_ls < buffed and not switch_locked:
                follow_left_edge = False
            else:
                follow_left_edge = True
            #if reflected_light_intensity > buffed*2 and follow_left_edge is False and reflected_light_intensity_ls > target_light_intensity_ls*2:
            #    follow_left_edge = True
            #    switch_locked = False

            error = target_light_intensity - reflected_light_intensity
            integral = integral + error
            derivative = error - last_error
            last_error = error
            turn_native_units = -((kp * error) + (ki * integral) + (kd * derivative))

            if not follow_left_edge:
                turn_native_units *= -1

            #if error < last_error:
            #    speed_native_units -= 2
            #elif error > last_error:
            #    speed_native_units += 5

            right_speed = SpeedNativeUnits(speed_native_units - turn_native_units)
            left_speed = SpeedNativeUnits(speed_native_units + turn_native_units)
            #print(turn_native_units, speed_native_units)

            # DEBUG
            #print('Error: %s' % error)
            #print('reflected_light_intesity: : %s' % reflected_light_intensity)
            #print('integral: %s' % integral)
            #print('derivative: %s' % derivative)
            #print('last_error: %s' % last_error)
            #print('turn_native_units: %s' % turn_native_units)
            #print('left_speed: %s' % left_speed)
            #print('right_speed: %s' % right_speed)

            if left_speed > MAX_SPEED:
                print("%s: left_speed %s is greater than MAX_SPEED %s" % (self, left_speed, MAX_SPEED))
                self.mMT.stop()
                raise LineFollowErrorTooFast("The robot is moving too fast to follow the line")

            if right_speed > MAX_SPEED:
                print("%s: right_speed %s is greater than MAX_SPEED %s" % (self, right_speed, MAX_SPEED))
                self.mMT.stop()
                raise LineFollowErrorTooFast("The robot is moving too fast to follow the line")

            # Have we lost the line?
            if reflected_light_intensity >= white:
                off_line_count += 1

                if off_line_count >= off_line_count_max:
                    #self.mMT.stop()
                    #raise LineFollowErrorLostLine("we lost the line")
                    pass
            else:
                off_line_count = 0

            if sleep_time:
                time.sleep(sleep_time)

            #self.mMT.on(left_speed, right_speed)

        self.mMT.stop()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--verbose', help='Activates verbose mode (gives more output)')
    parser.add_argument('--light_mode', help='Specify the light detection mode (ambient or reflected)',
                        type=str)
    parser.add_argument('--kp', help='KP')
    parser.add_argument('--ki', help='KI')
    parser.add_argument('--kd', help='KD')
    args = parser.parse_args()
    wall_e = RobotControl()
    try:
        wall_e.run()
    except KeyboardInterrupt:
        print("---[EXIT]---")
    finally:
        wall_e.mMT.off()

