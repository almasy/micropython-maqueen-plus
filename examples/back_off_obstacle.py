# This is an example demonstrating usage of the maqueen_plus 
# library.
#
# After checking the robot's readiness, this demo program goes
# into an infinite loop in which it regularly reads the distance
# sensor and in case there's an obstacle closer than 5 cm, the
# robots backs off a bit.
#
# Please make sure your robot is in a location with enough space 
# beyond it before running this program!

from microbit import display, sleep, Image
from maqueen_plus import maqueen_plus as robot

while not robot.is_ready():
    display.show(Image.SKULL)
    sleep(250)

display.show(Image.HAPPY)
sleep(250)

PERSONAL_ZONE = 5
distance = robot.sensors.distance
while True:
    if distance.get_distance() < PERSONAL_ZONE:
        display.show(Image.SURPRISED)
        robot.motors.backwards(50)
        sleep(250)
        robot.motors.stop()
    sleep(250)
    display.show(Image.HAPPY)
