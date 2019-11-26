#!/usr/bin/env python3
# coding: utf-8
# @author: Joel Stanciu (https://github.com/Flerov)
# boah nervt dieses light, dass ständig anbleibt...

from ev3dev2.sensor.lego import LightSensor
from ev3dev2.sensor import INPUT_3, INPUT_4
import sys
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--mode', help='Force the mode on both Light Sensors (a)mbient or (r)eflected')
    args = parser.parse_args()
    left = LightSensor(INPUT_3)
    right = LightSensor(INPUT_4)
    MODE_REFLECT = 'REFLECT'
    MODE_AMBIENT = 'AMBIENT'
    if args.mode == 'r' or args.mode == 'reflected':
        print('light mode: MODE_REFLECTED')
        left._ensure_mode(MODE_REFLECT)
        right._ensure_mode(MODE_REFLECT)
    elif args.mode == 'a' or args.mode == 'ambient':
        print('light mode: MODE_AMBIENT')
        left._ensure_mode(MODE_AMBIENT)
        right._ensure_mode(MODE_AMBIENT)