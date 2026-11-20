from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()
color_sensor = ColorSensor(Port.F)

while True:
    data = color_sensor.hsv()
    print(data.h, data.s, data.v)
    wait(1000)

