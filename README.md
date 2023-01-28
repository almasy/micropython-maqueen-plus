# micropython-maqueen-plus

BBC Micro:bit Micropython driver library for DFRobot's [Maqueen Plus 2.0](https://www.dfrobot.com/product-2487.html).

## Prerequisites

- [BBC Micro:bit v2.x](https://microbit.org/buy/) board
- [Micropython for Micro:bit v2.x](https://github.com/microbit-foundation/micropython-microbit-v2/releases/) installed on the Micro:bit board
- Maqueen Plus 2.0 robot
  - [18650 battery version](https://www.dfrobot.com/product-2487.html)
  - [Ni MH rechargeable batteries version](https://www.dfrobot.com/product-2026.html)
- Micropython development tools (see [Usage section](#usage))

Earlier versions of Micro:bit boards (e.g. v1.5) are very unlikely to work due to the extremely low RAM they come equipped with (compared to v2.x).

Please note that previous versions of Maqueen Plus robots (i.e. pre v2.0) come with different hardware (e.g. 6 line tracking sensors, URM10 ultrasonic sensor) and are not likely to work with this library.

## Features

The purpose of the **maqueen-plus** library is to provide user-friendly(ish) control of the Maqueen Plus robot by "shielding" the programmer from low-level details such as I2C communication with robot's components.

The library builds an abstraction in a form of several python classes composed into one ready to use variable named `maqueen_plus` which can by obtained by simple `import` statement. Different properties and functions accessible trough the `maqueen_plus` will allow the programmer to control

- Left and Right Motor
- Five Line (ADC gray scale) Sensors
- Left and Right Front LED
- SR04 Distance Sensor
- Infra Receiver
- 4 built-in RGB LEDs
- Maqueen Mechanic Kit Addons

Please note that this is still work in progress as following features are not
yet implemented:

- Infra Receiver
- Maqueen Mechanic Kit Addons

## Usage

Make sure your Micro:bit board is connected to a computer via USB cable and you have flashed a recent version of Micropython onto it. Some tools (such as official [Online Micro:Bit Python Editor](https://python.microbit.org/v/3/) will do it for you automatically, while others (e.g. [Mu](https://codewith.mu/en/) or [Thonny](https://thonny.org/)) may require some additional steps.

To use the **maqueen-plus** library, simply copy the [maqueen_plus.py](./maqueen_plus.py) file onto the Micro:bit board.

Then add following `import` statement into your own script which will control the robot (usually it's a file named `main.py`)

```python
from maqueen_plus import maqueen_plus
```

Please remember, that your script must be copied to the Micro:bit as well.

You can give the imported variable any name of your choice using `import as` statement, for example

```python
from maqueen_plus import maqueen_plus as robot
```

Before using any of the robot's features, perform a one-time check of the robot's readines (i.e. verify, that it has initialized itself correctly) by making sure the return value of the `is_ready()` function call equals to `True`.

Simple approach could be to wait in a loop until the robot reports it's ready

```python
# this snippet assumes maqueen_plus is already imported
from microbit import display, sleep, Image

while not maqueen_plus.is_ready():
    display.show(Image.SAD)
    sleep(250)
```

After a successful initialization, a full range of robot features is available. The `maqueen_plus` contains following objects

- `motors`
  - `right`
  - `left`
- `sensors`
  - `distance`
  - `infra`
  - `line`
    - `right2` also available as `line[0]`
    - `right1` also available as `line[1]`
    - `middle` also available as `line[2]`
    - `left1` also available as `line[3]`
    - `left2` also available as `line[4]`
- `front_lights`
  - `right`
  - `left`
- `chassis_RGBs` four NeoPixel RGBs
  - `chassis_RGBs[0]` located at front left side
  - `chassis_RGBs[1]` located at back left side
  - `chassis_RGBs[2]` located at back right side
  - `chassis_RGBs[3]` located at front right side

Simple front LED blinking example

```python
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
```

Please check the provided [examples](./examples/) and the `docstrings` of the **maqueen-plus** classes for an additional info.

## See Also

- [DFRobot's Maqueen Plus 2.0 official WIKI](https://wiki.dfrobot.com/SKU_MBT0021-EN_Maqueen_Plus_STEAM_Programming_Educational_Robot)
- [Latest MicroPython for Micro:bit documentation](https://microbit-micropython.readthedocs.io/en/v2-docs/)
