#!/usr/bin/env micropython
# coding: utf-8

import sys


RED = "\033[0;31m"
RED_BOLD = "\033[1;31m"
BLUE = "\033[0;34m"
WHITE = "\033[0;47m"
YELLOW = "\033[0;43m"
MAGENTA = "\033[0;45m"
CYAN = "\033[0;36m"
GREEN = "\033[0;32m"
RESET = "\033[0;0m"
BOLD = "\033[;1m"
REVERSE = "\033[;7m"


def color(*args):
    for i in args:
        sys.stdout.write(i)