import sys
import time
import math
import functools

from picarx_improved import Picarx


# Helper functions and classes

def ask_for_angle(invert=False):
    """Decorator to ask for angle"""

    def ask_for_angle_decorator(func):

        @functools.wraps(func)
        def input_angle_wrapper(car):
            """Wrapped maneuver that asks for angle before adding input"""

            try:
                angle = max(min(int(input("Enter an angle for the maneuver (-40 degrees to 40 degrees): ")), 40), -40)
                if invert:
                    angle *= -1
                func(car, angle)
            except TypeError:
                print("You wrote an invalid angle. Please try again.")

        return input_angle_wrapper

    return ask_for_angle_decorator


class Maneuver:
    """Class that facilitates associates maneuvers to keyboard inputs"""

    _maneuvers = {}

    def __init__(self, key, name, maneuver_func):

        self.key = key
        self.name = name
        self.maneuver = maneuver_func  # Maneuver function should return nothing if you want to continue operating the car

        self._maneuvers[key] = self  # Store name in storage
    
    @classmethod
    def is_defined(cls, key):

        return key in cls._maneuvers

    @classmethod
    def get_maneuver(cls, key):
        """Get a manuever with the given key"""

        return cls._maneuvers.get(key)
    
    @classmethod
    def available_maneuvers(cls):
        """List of available maneuvers and keys"""

        return [f"{key} = {maneuver.name}" for key, maneuver in cls._maneuvers.items()]

# Main function to run the car

def run_car():
    """Main car running function"""

    pi_car = Picarx()  # Create picar instance

    # Set up maneuvers
    Maneuver('x', "terminate_program", lambda : 0)
    Maneuver('a', "forward", lambda : ask_for_angle()(pi_car.forward))
    Maneuver('s', "backward", lambda : ask_for_angle()(pi_car.backward))
    Maneuver('d', "parallel_park_left", lambda : ask_for_angle()(pi_car.parallel_park))
    Maneuver('f', "parallel_park_right", lambda : ask_for_angle(invert=True)(pi_car.parallel_park))
    Maneuver('j', "three_point_turn_left", lambda : ask_for_angle()(pi_car.three_point_turn))
    Maneuver('k', "three_point_turn_right", lambda : ask_for_angle(invert=True)(pi_car.three_point_turn))
    Maneuver('c', "calibrate_car", lambda : pi_car.calibrate_steering)

    manuever_list_str = f"List of available maneuvers: {', '.join(Maneuver.available_maneuvers())}"

    # Main program loop
    while True:
        key = input(manuever_list_str + "\n\nEnter one of the keys above: ")
        if Maneuver.is_defined(key):
            maneuver = Maneuver.get_maneuver(key)
            if maneuver() is not None:
                break
        else:
            print(f"'{key}' is not a valid key maneuver input.")

    sys.exit()  # Terminate program


if __name__  == "__main__":

    run_car()
    