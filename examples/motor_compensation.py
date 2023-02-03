# This is an example demonstrating usage of the maqueen_plus 
# library.
#
# A simple program allowing to find out the best motor compensation
# value specific to your particular copy of MaqueenPlus robot.
#  
# Each touch of the Logo "button" on the Micro:Bit board will trigger 
# a short forward and backward robot movement. Under the ideal conditions,
# the robot should travel straight for five seconds and then return back 
# to its original position. 
#
# When the motors aren't calibrated properly, the robot tends to turn 
# slightly left or right. To compensate for the right turns, press the 
# A button on the Micro:Bit board to increase speed of right motor. Then 
# press touch the Logo "button" to check, whether the robot goes straight
# already. 
#
# The B button decreases the compensation level caused by the A button,
# and eventually starts to compensate for the unwanted left turns.
#
# The current left / right compensation values are continuously shown
# on Micro:Bit's LED display. Moreover, each time the Logo "button" 
# is touched, both left and right compensation values are written 
# to Micro:Bit's storage using the `log` module. This creates a HTML 
# report which can be copied to your computer and viewed in a Web 
# browser.

import log
from microbit import display, sleep, Image, button_a, button_b, pin_logo
from maqueen_plus import maqueen_plus as robot

while not robot.is_ready():
    display.show(Image.SKULL)
    sleep(250)

display.show(Image.HAPPY)
sleep(250)

cmp_right = 0
cmp_left = 0
while True:

    if pin_logo.is_touched():
        while pin_logo.is_touched():
            sleep(50)
        sleep(150)
        if cmp_right > 0:
            robot.motors.compensate_right(cmp_right)
        else:
            robot.motors.compensate_left(cmp_left)
        log.add({ 'right compensation': cmp_right, 'left compensation': cmp_left})
        robot.motors.forwards(50)
        sleep(5000)
        robot.motors.reverse()
        sleep(5000)
        robot.motors.stop()

    if button_a.was_pressed():
        if cmp_left > 0:
            cmp_left -= 1
        else:
            cmp_right += 1
        display.scroll('L: {}, R:{}'.format(cmp_left, cmp_right), wait=False, loop=True)
    
    if button_b.was_pressed():
        if cmp_right > 0:
            cmp_right -= 1
        else:
            cmp_left += 1
        display.scroll('L: {}, R:{}'.format(cmp_left, cmp_right), wait=False, loop=True)
