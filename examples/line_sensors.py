# This is an example demonstrating usage of the maqueen_plus 
# library.
#
# After checking the robot's readiness, this demo program goes
# into an infinite loop in which it regularly reads all 5 of the 
# line tracking sensors and in case particular sensor detects 
# a line, two Micro:bit display LEDs are lit on a position analogic
# to the sensor's position. 
#
# The robot is not moving by itself, hence it's necessary to place
# it onto a map with black track(s) and manually move it around to
# see how different LEDs light up or switch off.

from microbit import display, sleep, Image
from maqueen_plus import maqueen_plus as robot

while not robot.is_ready():
    display.show(Image.SKULL)
    sleep(250)

display.show(Image.HAPPY)
sleep(500)
display.clear()

def draw_sensor(pixel_sequence, brightness):
    for x,y in pixel_sequence:
        display.set_pixel(x, y, brightness)

sensors_on = {
    'right2': ((0, 3), (0, 4)),
    'right1': ((0, 0), (0, 1)),
    'middle': ((2, 0), (2, 1)),
    'left1':  ((4, 0), (4, 1)),
    'left2':  ((4, 3), (4, 4))
}

while True:
    for sensor in robot.sensors.line:
        brightness = 9 if sensor.has_track() else 0
        draw_sensor(sensors_on[sensor.name], brightness)
    sleep(250)
