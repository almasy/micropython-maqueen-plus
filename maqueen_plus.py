# Simple Micropython / MicroBit 2.x driver for DFRobot's
# MaqueenPlus v 2.1.0 
# (see https://www.dfrobot.com/product-2487.html)
#
# Author: Peter Almásy, https://github.com/almasy/micropython-maqueen-plus
# Version: 0.3.1
# License: MIT License

from micropython import const
from machine import time_pulse_us
from utime import sleep_us
from microbit import i2c, pin13, pin14, pin15
from neopixel import NeoPixel

# Maqueen's I2C address
I2C_ADDR: int = const(0x10)

# These seem to be hardwired into the robot
SONAR_TRIG_PIN = pin13
SONAR_ECHO_PIN = pin14
CHASSIS_RGB_PIN = pin15

# Motor related constants
MotorDirection = int
FORWARD: MotorDirection = const(0)
BACKWARD: MotorDirection = const(1)

MotorSpeed = int
MIN_MOTOR_SPEED: MotorSpeed = const(0)
MAX_MOTOR_SPEED: MotorSpeed = const(255)
MOTOR_SPEED_IGNORE: MotorSpeed = const(-1)

_LEFT_MOTOR_REG: int = const(0x00)
_RIGHT_MOTOR_REG: int = const(0x02)
# Indexes into I2C buffer for motor control
_M_REG_I: int = const(0)
_M_DIR_I: int = const(1)
_M_SPEED_I: int = const(2)

class _Motor:
    """ Motor driver class. Contains elementary motor functions. """

    def __init__(self, register: int) -> None:
        if register not in [_LEFT_MOTOR_REG, _RIGHT_MOTOR_REG]:
            raise ValueError('Unsupported motor register value: {}'.format(register))
        self._register = bytes((register,))
        self._buffer = bytearray(3)
        self._buffer[_M_REG_I] = register
        self._buffer[_M_DIR_I] = FORWARD
        self._buffer[_M_SPEED_I] = MIN_MOTOR_SPEED

    def set_speed(self, speed: MotorSpeed) -> None:
        """
            Sets a speed of the motor to the given value if and 
            only if the value is within the supported range.
            
            Requests with speed out of the supported range are
            quietly ignored.

            Parameters:
            speed: integer value from 0 (MIN_MOTOR_SPEED) 
                    to 255 (MAX_MOTOR_SPEED)
        """
        if speed >= MIN_MOTOR_SPEED and speed <= MAX_MOTOR_SPEED:
            self._buffer[_M_SPEED_I] = speed
            i2c.write(I2C_ADDR, self._buffer)
    
    def get_speed(self) -> MotorSpeed:
        """ 
            Provides current speed of the motor.

            Returns:
            int: speed within a range of 0 (MIN_MOTOR_SPEED) 
                to 255 (MAX_MOTOR_SPEED)
        """
        i2c.write(I2C_ADDR, self._register)
        response = i2c.read(I2C_ADDR, 2)
        return response[_M_SPEED_I - 1]
    
    def _move(self, direction: MotorDirection, speed: MotorSpeed) -> None:
        """ Auxiliary function. Do not use externally! """
        send_change = False
        if speed >= MIN_MOTOR_SPEED and speed <= MAX_MOTOR_SPEED:
            self._buffer[_M_SPEED_I] = speed
            send_change = True
        if self._buffer[_M_DIR_I] != direction:
            self._buffer[_M_DIR_I] = direction
            send_change = True
        if send_change:
            i2c.write(I2C_ADDR, self._buffer)

    def forwards(self, speed: MotorSpeed = MOTOR_SPEED_IGNORE) -> None:
        """
            Sets motor's direction to 'forward' and, if 
            a speed value is provided and it's within a range
            of 0 (MIN_MOTOR_SPEED) to 255 (MAX_MOTOR_SPEED), 
            the motor's speed is updated as well. 

            Out of range speed values are ignored and motor's
            speed is not updated.
        """
        self._move(FORWARD, speed)

    def backwards(self, speed: MotorSpeed = MOTOR_SPEED_IGNORE) -> None:
        """
            Sets motor's direction to 'backward' and, if 
            a speed value is provided and it's within a range
            of 0 (MIN_MOTOR_SPEED) to 255 (MAX_MOTOR_SPEED), 
            the motor's speed is updated as well. 

            Out of range speed values are ignored and motor's
            speed is not updated.
        """
        self._move(BACKWARD, speed)

    def reverse(self) -> None:
        """ 
            Flips motor's direction (i.e. from 'forward' to 'backward'
            or vice versa)
        """
        if self._buffer[_M_DIR_I] == BACKWARD:
            self._buffer[_M_DIR_I] = FORWARD
        else:
            self._buffer[_M_DIR_I] = BACKWARD
        i2c.write(I2C_ADDR, self._buffer)

    def get_direction(self) -> MotorDirection:
        """ 
            Provides current direction of the motor.
        
            Returns:
            MotorDirection: FORWARD or BACKWARD
        """
        i2c.write(I2C_ADDR, self._register)
        response = i2c.read(I2C_ADDR, 2)
        return response[_M_DIR_I - 1]

# Motor control constants
TURN_SHARPNESS_MIN: int = const(1)
TURN_SHARPNESS_MAX: int = const(20)
_TURN_TO_RATIO: int = const(10) # used for conversion

def _assure_range(val: int, min: int, max: int) -> int:
    """ Auxiliary function. Don't use externally! """
    if val < min:
        val = min
    elif val > max:
        val = max
    return val

class _Motors:
    """ Class aggregating both robot motors. Provides some convenience functions. """

    def __init__(self) -> None:
        self.left = _Motor(_LEFT_MOTOR_REG)
        self.right = _Motor(_RIGHT_MOTOR_REG)

    def compensate(self, left: int = 0, right: int = 0) -> None:
        """ T.B.D. """
        pass

    def stop(self) -> None:
        """ Stops both robot's motors immediately. """
        self.left.set_speed(0)
        self.right.set_speed(0)

    def _percentage_to_speed(self, percentage: int) -> int:
        """ Auxiliary function. Don't use externally! """
        return int(MAX_MOTOR_SPEED * _assure_range(percentage, 0, 100) / 100 + 0.5)

    def forwards(self, percentage: int) -> None:
        """
            Sets direction of both robot's motors to 'forward'
            and updates speed of both motors to value calculated
            from given percentage. 

            E.g. percentage = 100 will yield motor speed of 255 
            (MAX_MOTOR_SPEED), percentage = 50 will set speed to 128. 
        """
        speed = self._percentage_to_speed(percentage)
        self.left.forwards(speed)
        self.right.forwards(speed)

    def backwards(self, percentage: int) -> None:
        """
            Sets direction of both robot's motors to 'backward'
            and updates speed of both motors to value calculated
            from given percentage. 

            E.g. percentage = 100 will yield motor speed of 255 
            (MAX_MOTOR_SPEED), percentage = 50 will set speed to 128. 
        """
        speed = self._percentage_to_speed(percentage)
        self.left.backwards(speed)
        self.right.backwards(speed)

    def reverse(self) -> None:
        """ Flips direction of both robot's motors. """
        self.left.reverse()
        self.right.reverse()

    def _turn_ratio(self, sharpness: int) -> 'tuple[float, bool]':
        """ Auxiliary function. Don't use externally! """
        sharpness = _assure_range(sharpness, TURN_SHARPNESS_MIN, TURN_SHARPNESS_MAX)
        ratio = (TURN_SHARPNESS_MAX / 2 - sharpness) / _TURN_TO_RATIO
        if ratio < 0:
            reverse = True
            ratio = -ratio
        else:
            reverse = False
        return (ratio, reverse)

    def _turn(self, sharpness: int, keep: _Motor, adjust: _Motor):
        """ Auxiliary function. Don't use externally! """
        ratio, reverse = self._turn_ratio(sharpness)
        if reverse:
            adjust.reverse()
        adjust.set_speed(int(keep.get_speed() * ratio))
    
    def turn_left(self, sharpness: int) -> None:
        """ 
            Adjusts the speed of left motor proportionally to the speed
            of right motor, effectively turning the robot left. The sharpness
            value controls how big the speed difference between the motors
            will be. The higher value results in sharper robot turns.

            The minimal turn sharpness is 1 (TURN_SHARPNESS_MIN), the maximal 
            is 20 (TURN_SHARPNESS_MAX)
        """
        if self.right.get_direction() != self.left.get_direction():
            self.left.reverse()
        self._turn(sharpness, self.right, self.left)

    def turn_right(self, sharpness: int) -> None:
        """ 
            Adjusts the speed of left motor proportionally to the speed
            of right motor, effectively turning the robot left. The sharpness
            value controls how big the speed difference between the motors
            will be. The higher value results in sharper robot turns.

            The minimal turn sharpness is 1 (TURN_SHARPNESS_MIN), the maximal 
            is 20 (TURN_SHARPNESS_MAX)
        """
        if self.right.get_direction() != self.left.get_direction():
            self.right.reverse()
        self._turn(sharpness, self.left, self.right)


# Headlights LED related constants
_LEFT_LED_REG: int = const(0x0B)
_RIGHT_LED_REG: int = const(0x0C)

class _LED:
    """ Headlight LED driver class. Contains elementary LED functions."""

    def __init__(self, register: int) -> None:
        if register not in [_LEFT_LED_REG, _RIGHT_LED_REG]:
            raise ValueError('Unsupported LED register value: {}'.format(register))
        self._buffer = bytearray(2)
        self._buffer[0] = register
    
    def on(self) -> None:
        """ Turns headlight LED on. """
        self._buffer[1] = 1
        i2c.write(I2C_ADDR, self._buffer)
    
    def off(self) -> None:
        """ Turns headlight LED off. """
        self._buffer[1] = 0
        i2c.write(I2C_ADDR, self._buffer)


class _Headlights:
    """ Class aggregating robot's headlight LEDs. Provides some convenience functions. """

    def __init__(self) -> None:
        self.left = _LED(_LEFT_LED_REG)
        self.right = _LED(_RIGHT_LED_REG)

    def on(self) -> None:
        """ Turns all robots headlight LEDs on. """
        self.left.on()
        self.right.on()

    def off(self) -> None:
        """ Turns all robots headlight LEDs off. """
        self.left.off()
        self.right.off()


# Line sensors related constants
_LINE_STATE_REG: bytes = b'\x1D'
_ADC_RIGHT2: 'tuple[bytes, int, str]' = (b'\x1E', 0x01, 'right2')
_ADC_RIGHT1: 'tuple[bytes, int, str]' = (b'\x20', 0x02, 'right1')
_ADC_MIDDLE: 'tuple[bytes, int, str]' = (b'\x22', 0x04, 'middle')
_ADC_LEFT1: 'tuple[bytes, int, str]'  = (b'\x24', 0x08, 'left1')
_ADC_LEFT2: 'tuple[bytes, int, str]'  = (b'\x26', 0x10, 'left2')

_KNOWN_ADCS = (_ADC_RIGHT2, _ADC_RIGHT1, _ADC_MIDDLE, _ADC_LEFT1, _ADC_LEFT2)

class _LineSensor:
    """ 
        Line (grayscale ADC) tracking sensor driver class. Please note that
        the line sensor may require calibration before it can provide correct 
        readings (see Maqueen Plus user manual / WIKI for additional info).
    """

    def __init__(self, sensor_info: 'tuple[bytes, int, str]') -> None:
        if sensor_info not in _KNOWN_ADCS:
            raise ValueError('Unsupported line sensor: {}'.format(sensor_info))
        self._register = sensor_info[0]
        self._status_bit = sensor_info[1]
        self.name = sensor_info[2]

    def has_track(self) -> bool:
        """ Returns True in case sensor reads 'black line', False otherwise. """
        i2c.write(I2C_ADDR, _LINE_STATE_REG)
        value = i2c.read(I2C_ADDR, 1)[0]
        return (value & self._status_bit) == self._status_bit

    def get_value(self) -> int:
        """ 
            Provides current sensor reading (i.e. level of brightness). 
            The higher the returned value, the brighter (i.e. less black)
            the ground colour is.
        """
        i2c.write(I2C_ADDR, self._register)
        value = i2c.read(I2C_ADDR, 2)
        return value[1] << 8 | value[0]


class _LineSensors:
    """ 
        Class aggregating all robot's line tracking sensors. Allows 
        individual access to every sensor as well as iteration trough 
        all of the sensors.
    """

    def __init__(self) -> None:
        self._sensors = (
            _LineSensor(_ADC_RIGHT2),
            _LineSensor(_ADC_RIGHT1),
            _LineSensor(_ADC_MIDDLE),
            _LineSensor(_ADC_LEFT1),
            _LineSensor(_ADC_LEFT2)
        )

    def __iter__(self):
        """ Allow iteration trough sensors (e.g. in for loop) """
        return iter(self._sensors)

    def _find_sensor(self, name: str):
        """ Auxiliary function. Do not use externally! """
        result = [s for s in self._sensors if s.name == name]
        return result[0] if result else None

    def __getitem__(self, key: 'int|str') -> _LineSensor:
        """ 
            Override the [] operator for easy sensor retrieval via 
            its position/index or name.
        """
        if type(key) is int:
            return self._sensors[key]
        if type(key) is not str:
            raise KeyError("'_LineSensors' doesn't support key type '{}'".format(type(key)))
        if sensor := self._find_sensor(key):
            return sensor
        raise KeyError("Unknown '_LineSensor' name '{}'".format(key))

    def __getattr__(self, name: str) -> _LineSensor:
        """ Allow access to individual sensors via their names. """
        if sensor := self._find_sensor(name):
            return sensor
        raise AttributeError("Unknown '_LineSensor' name '{}'".format(name))

    def __len__(self) -> int:
        """ Provide proper length of sensor sequence. """
        return len(self._sensors)
  
    def get_on_track(self) -> 'tuple[_LineSensor, ...]':
        """
            Returns a tuple of all sensors which currently read a black
            line / track bellow them.
        """
        return tuple(s for s in self._sensors if s.has_track())


_TIME_TO_DISTANCE = const(59) # == 1 / (0.034 / 2)
""" 
    Sound travels 0.034 cm in 1 us, we take half of that
    as the sensor measures the sound wave roundtrip.
"""
_MAX_DISTANCE_CM = const(500)
_MAX_PULSE_TIMEOUT = const(29500) # == 500 * 59 microseconds
""" Limit distance to approx 5 meters """
_MIN_TRIGGER_TIME = const(10) # microseconds

class _DistanceSensor:
    """ Distance ultrasonic sensor (HC-SR04) driver class. """

    def __init__(self, trigPin, echoPin) -> None:
        self._trigPin = trigPin
        self._echoPin = echoPin

    def get_distance(self) -> int:
        """
            Provides current distance of the robot (more 
            precisely its ultrasonic sensor) from an obstacle.

            The reading comes out in centimeters. The effective
            range of the sensor is approx. 2 cm to 500 cm. The
            precision of the measurement is affected by the size
            of the obstacle (the manufacturer recommends objects
            with surface larger than 0.5 square meters when further
            away).

            Returns:
            int: a distance in centimeters (range of 1 to 500). 
            
            Please note that 0 indicates there's been some issue 
            during the measurement.
        """
        self._trigPin.write_digital(0)
        sleep_us(2)
        self._trigPin.write_digital(1)
        sleep_us(_MIN_TRIGGER_TIME)
        self._trigPin.write_digital(0)
        pulse = time_pulse_us(self._echoPin, 1, _MAX_PULSE_TIMEOUT)
        distance = int(pulse / _TIME_TO_DISTANCE + 0.5)
        if  distance < 0:
            distance = 0
        elif distance > _MAX_DISTANCE_CM:
            distance = _MAX_DISTANCE_CM
        return distance


class _InfraReceiver:
    """ T.B.D. """
    def __init__(self) -> None:
        pass

class _Sensors:
    """ Class aggregating all robot's sensors. """
    def __init__(self) -> None:
        self.distance = _DistanceSensor(SONAR_TRIG_PIN, SONAR_ECHO_PIN)
        self.line = _LineSensors()
        self.infra = _InfraReceiver()

# Robot version / status registers
_VERSION_CNT_REG: bytes = b'\x32'
_VERSION_DATA_REG: bytes = b'\x33'

_RGB_LED_CNT: int = const(4)

class _MaqueenPlus:
    """ Main Maqueen Plus robot class holds all motor, sensor, LED and other HW drivers. """

    def __init__(self) -> None:
        self.motors = _Motors()
        self.sensors = _Sensors()
        self.headlights = _Headlights()
        self.chassis_RGBs = NeoPixel(CHASSIS_RGB_PIN, _RGB_LED_CNT)

    def is_ready(self) -> bool:
        """ Returns True in case the robot is ready to work, False otherwise. """
        i2c.write(I2C_ADDR, _VERSION_CNT_REG)
        return i2c.read(I2C_ADDR, 1)[0] != 0

    def get_version(self) -> str:
        """ Provides a string with robot's version information. """
        i2c.write(I2C_ADDR, _VERSION_CNT_REG)
        size = i2c.read(I2C_ADDR, 1)[0]
        i2c.write(I2C_ADDR, _VERSION_DATA_REG)
        version = i2c.read(I2C_ADDR, size)
        return version.decode('utf-8')


maqueen_plus = _MaqueenPlus()
