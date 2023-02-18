import math
import logging

import numpy as np
from scipy.interpolate import interp1d
from logdecorator import log_on_start, log_on_end, log_on_error

from modules import Ultrasonic


class Sensor:

    def __init__(self):

        self.us_structs = [Ultrasonic(pin) for pin in ('D2','D3')]
    
    def read_ultrasonic(self):

        return [ultrasonic.read() for ultrasonic in self.adc_structs]


class Interpreter:

    pass


class Controller:

    pass


