# This is an example demonstrating usage of the maqueen_plus 
# library.
#
# The code will check the robot for readiness and then will
# alternate robot's left and right front LEDs blinking for 
# 15 seconds.

from microbit import display, sleep, Image
from maqueen_plus import maqueen_plus

while not maqueen_plus.is_ready():
    display.show(Image.SKULL)
    sleep(250)

display.show(Image.HAPPY)
sleep(250)

lights = maqueen_plus.front_lights
for _ in range(15):
    lights.left.on()
    sleep(500)
    lights.right.on()
    lights.left.off()
    sleep(500)
    lights.right.off()

display.clear()
