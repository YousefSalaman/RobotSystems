import math
import logging

import numpy as np
from scipy.interpolate import interp1d
from logdecorator import log_on_start, log_on_end, log_on_error

from modules import ADC


class Sensor:

    def __init__(self):

        self.adc_structs = [ADC(pin) for pin in ("A0", "A1", "A2")]

    def read_adcs(self):
        """Get current adc reading"""

        return [adc.read() for adc in self.adc_structs]


class Interpreter:

    def __init__(self):

        self.polarity = 1
        self.sensitivity = 200
        self.output = None
        self.map = interp1d(np.linspace(0, self.sensitivity, 10), np.linspace(-1, 1, 10))

    def as_difference(self, adc_read):
        """Express ADC readings as a difference"""

        return [abs(adc_read[i] - adc_read[i+1]) for i in range(2)]

    def interpret(self, adc_read):
        """Return a value from -1 to 1 indicating which direction to turn"""

        if adc_read is not None:

            adc_read_diff = self.as_difference(adc_read)

            # If values are similar, car is aligned
            if math.isclose(adc_read_diff[0], adc_read_diff[1], rel_tol=self.sensitivity):
                return 0

            # Align car in correct direction
            scaled_output = self.map(abs(adc_read_diff[0] - adc_read_diff[1])) * self.polarity
            if adc_read_diff[0] > adc_read_diff[1]:
                scaled_output *= -1

            self.output = scaled_output

            return scaled_output

    def get_scaled_output(self):

        return self.output


class Controller:
    """Class that mimics a proportional controller"""

    def __init__(self, k_p=1):

        self.k_p = k_p

    def control(self, scaled_output, car, speed):

        dist = speed * .1
        angle = scaled_output * self.k_p
        car.forward(dist, angle)

        return angle