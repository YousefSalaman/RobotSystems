import time


class Servo:
    """Mock Servo class"""

    def __init__(self, pwm):
        pass

    # angle ranges -90 to 90 degrees
    def angle(self, angle):
        pass

    # pwm_value ranges MIN_PW 500 to MAX_PW 2500 degrees
    def set_pwm(self,pwm_value):
        pass


class PWM:
    REG_CHN = 0x20
    REG_FRE = 0x30
    REG_PSC = 0x40
    REG_ARR = 0x44

    ADDR = 0x14

    CLOCK = 72000000

    def __init__(self, channel, debug="critical"):
        pass

    def i2c_write(self, reg, value):
        pass

    def freq(self, *freq):
    
        if len(freq) == 0:
            return 1

    def prescaler(self, *prescaler):
        if len(prescaler) == 0:
            return 1

    def period(self, *arr):

        if len(arr) == 0:
            return 1

    def pulse_width(self, *pulse_width):
        if len(pulse_width) == 0:
            return 1

    def pulse_width_percent(self, *pulse_width_percent):

        if len(pulse_width_percent) == 0:
            return 1


class fileDB(object):
	"""A file based database.

    A file based database, read and write arguements in the specific file.
    """
	def __init__(self, db: str, mode:str=None, owner:str=None):  
		'''Init the db_file is a file to save the datas.'''

	def file_check_create(self, file_path:str, mode:str=None, owner:str=None):
		pass
	
	def get(self, name, default_value=None):
		"""Get value by data's name. Default value is for the arguemants do not exist"""
	
	def set(self, name, value):
		"""Set value by data's name. Or create one if the arguement does not exist"""


class Pin:
    """Mock Pin class"""

    _dict = {
        "BOARD_TYPE": 12,
    }

    _dict_1 = {
        "D0":  17,
        "D1":  18,
        "D2":  27,
        "D3":  22,
        "D4":  23,
        "D5":  24,
        "D6":  25,
        "D7":  4,
        "D8":  5,
        "D9":  6,
        "D10": 12,
        "D11": 13,
        "D12": 19,
        "D13": 16,
        "D14": 26,
        "D15": 20,
        "D16": 21, 
        "SW":  19,
        "USER": 19,        
        "LED": 26,
        "BOARD_TYPE": 12,
        "RST": 16,
        "BLEINT": 13,
        "BLERST": 20,
        "MCURST": 21,

    }

    _dict_2 = {
        "D0":  17,
        "D1":   4, # Changed
        "D2":  27,
        "D3":  22,
        "D4":  23,
        "D5":  24,
        "D6":  25, # Removed
        "D7":   4, # Removed
        "D8":   5, # Removed
        "D9":   6,
        "D10": 12,
        "D11": 13,
        "D12": 19,
        "D13": 16,
        "D14": 26,
        "D15": 20,
        "D16": 21,     
        "SW":  25, # Changed
        "USER": 25,
        "LED": 26,
        "BOARD_TYPE": 12,
        "RST": 16,
        "BLEINT": 13,
        "BLERST": 20,
        "MCURST":  5, # Changed
    }

    def __init__(self, *value):
        
        if len(value) > 0:
            pin = value[0]
        if len(value) > 1:
            mode = value[1]
        else:
            mode = None
        if len(value) > 2:
            setup = value[2]
        else:
            setup = None
        if isinstance(pin, str):
            try:
                self._board_name = pin
                self._pin = self.dict()[pin]
            except Exception as e:
                print(e)
        elif isinstance(pin, int):
            self._pin = pin
        self._value = 0
        self.init(mode, pull=setup)
        
    def check_board_type(self):
        pass

    def init(self, mode, pull=None):
        self._pull = pull
        self._mode = mode

    def dict(self, *_dict):
        if len(_dict) == 0:
            return dict()

    def __call__(self, value):
        return self.value(value)

    def value(self, *value):
        if len(value) == 0:
            return 1
        else:
            value = value[0]
            return value

    def on(self):
        return self.value(1)

    def off(self):
        return self.value(0)

    def high(self):
        return self.on()

    def low(self):
        return self.off()

    def mode(self, *value):
        if len(value) == 0:
            return (self._mode, self._pull)
        else:
            self._mode = value[0]
            if len(value) == 2:
                self._pull = value[1]

    def pull(self, *value):
        return self._pull

    def irq(self, handler=None, trigger=None, bouncetime=200):
        self.mode(1)

    def name(self):
        return "GPIO%s"%self._pin

    def names(self):
        return [self.name, self._board_name]

    class cpu(object):

        def __init__(self):
            pass


class Grayscale_Module:

    def __init__(self, pin0, pin1, pin2, reference=1000):
        self.reference = reference

    def get_line_status(self,fl_list):

        if fl_list[0] > self.reference and fl_list[1] > self.reference and fl_list[2] > self.reference:
            return 'stop'
            
        elif fl_list[1] <= self.reference:
            return 'forward'
        
        elif fl_list[0] <= self.reference:
            return 'right'

        elif fl_list[2] <= self.reference:
            return 'left'

    def get_grayscale_data(self):
        adc_value_list = []
        return adc_value_list


class Ultrasonic():

    def __init__(self, trig, echo, timeout=0.02):
        self.trig = trig
        self.echo = echo
        self.timeout = timeout

    def _read(self):
        return 1
        # self.trig.low()
        # time.sleep(0.01)
        # self.trig.high()
        # time.sleep(0.00001)
        # self.trig.low()
        # pulse_end = 0
        # pulse_start = 0
        # timeout_start = time.time()
        # while self.echo.value()==0:
        #     pulse_start = time.time()
        #     if pulse_start - timeout_start > self.timeout:
        #         return -1
        # while self.echo.value()==1:
        #     pulse_end = time.time()
        #     if pulse_end - timeout_start > self.timeout:
        #         return -1
        # during = pulse_end - pulse_start
        # cm = round(during * 340 / 2 * 100, 2)
        # return cm

    def read(self, times=10):

        return 1
        # for i in range(times):
        #     a = self._read()
        #     if a != -1:
        #         return a
        # return -1