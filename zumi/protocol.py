from enum import Enum


class Accelerometer:
    X = 0x3B
    Y = 0x3D
    Z = 0x3F


class Gyro:
    X = 0x43
    Y = 0x45
    Z = 0x47


"""
Commands are given as a single byte, where the bits are parsed as shown:

XX YYY ZZZ

XX - Command Type
YYY - Command
ZZZ - Extra Bits - Command-specific bits that can be used to alter commands (e.g. enable/disable accleration when 
moving)
"""


class CommandType:
    Motor = 0
    Audio = 1
    LED = 2


class MotorCommand:
    Stop = 0
    Drive = 1
    SetMotorsToSpeed = 2
    SetDefaultSpeed = 3
    ChangeForceDrive = 4
    ChangeLineFollower = 5


class LEDCommand:
    ChangeAllLEDs = 0
    ChangeFrontLEDs = 1
    ChangeBackLEDs = 2
    FlashAllLEDs = 3
    FlashLeftLEDs = 4
    FlashRightLEDs = 5


class Note:
    C2 = 1
    CS2 = 2
    D2 = 3
    DS2 = 4
    E2 = 5
    F2 = 6
    FS2 = 7
    G2 = 8
    GS2 = 9
    A2 = 10
    AS2 = 11
    B2 = 12
    C3 = 13
    CS3 = 14
    D3 = 15
    DS3 = 16
    E3 = 17
    F3 = 18
    FS3 = 19
    G3 = 20
    GS3 = 21
    A3 = 22
    AS3 = 23
    B3 = 24
    C4 = 25
    CS4 = 26
    D4 = 27
    DS4 = 28
    E4 = 29
    F4 = 30
    FS4 = 31
    G4 = 32
    GS4 = 33
    A4 = 34
    AS4 = 35
    B4 = 36
    C5 = 37
    CS5 = 38
    D5 = 39
    DS5 = 40
    E5 = 41
    F5 = 42
    FS5 = 43
    G5 = 44
    GS5 = 45
    A5 = 46
    AS5 = 47
    B5 = 48
    C6 = 49
    CS6 = 50
    D6 = 51
    DS6 = 52
    E6 = 53
    F6 = 54
    FS6 = 55
    G6 = 56
    GS6 = 57
    A6 = 58
    AS6 = 59
    B6 = 60


class IR:
    front_right = 0
    bottom_right = 1
    back_right = 2
    bottom_left = 3
    back_left = 4
    front_left = 5


class Device:
    Arduino = 0x04
    MPU = 0x68


class MPURegister:
    PWR_M = 0x6B
    DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    INT_EN = 0x38
    TEMP = 0x41


class Axis:
    def __init__(self, x=0, y=0, z=0, scalar=1):
        self.X = x
        self.Y = y
        self.Z = z
        self.scalar = scalar

    def calculate_error(self, count):
        self.X /= (count * self.scalar)
        self.Y /= (count * self.scalar)
        self.Z /= (count * self.scalar)

    def add_value(self, args):
        self.X += args[0]
        self.Y += args[1]
        self.Z += args[2]


class Command(Enum):
    up = 0
    down = 1
    left = 2
    right = 4
    circle = 5
    triangle = 6
    square = 7
