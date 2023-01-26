# This is an example demonstrating usage of the maqueen_plus 
# library.
#
# The code will check the robot for readiness and then will blink 
# each of the chassis RGBs with red, green and blue colours. Finally,
# all RGBs will fade-in and fade-out in while colour.

from microbit import display, sleep, Image
from maqueen_plus import maqueen_plus as robot

while not robot.is_ready():
    display.show(Image.SKULL)
    sleep(250)

display.show(Image.HAPPY)
sleep(250)

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

for rgb in (RED, GREEN, BLUE):
    for i in range(4):
        robot.chassis_RGBs[i] = rgb
        robot.chassis_RGBs.show()
        sleep(500)
        robot.chassis_RGBs.clear()
        sleep(500)

for level in range(0, 256, 8):
    robot.chassis_RGBs.fill((level, level, level))
    robot.chassis_RGBs.show()
    sleep(50)

for level in range(255, 0, -8):
    robot.chassis_RGBs.fill((level, level, level))
    robot.chassis_RGBs.show()
    sleep(50)

robot.chassis_RGBs.clear()
display.clear()
