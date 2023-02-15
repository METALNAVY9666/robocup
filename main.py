#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from time import sleep


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.

class Robot:

    def __init__(self):
        self.droit = Motor(Port.B, Direction.CLOCKWISE)
        self.gauche = Motor(Port.C, Direction.CLOCKWISE)

    def avance(self, v, t):
        """fait avancer les 2 moteurs pendant t millisecondes à v vitesse"""
        for i in range(t):
            sleep(0.001)
            self.droit.run(-v)
            self.gauche.run(-v)

    def tourne_droit(self, v, t):
        """faire tourner le robot à droite (90°)"""
        for i in range(t):
            sleep(0.001)
            self.droit.run(-v)
            self.gauche.run(0)
    
    def demi_tour(self, v, t):
        for i in range(t):
            sleep(0.001)
            self.droit.run(-v)
    def fete(self, v, t):
        for i in range(t):
            sleep(0.001)
            self.droit.run(-v)
            self.gauche.run(v)



        

bot = Robot()
"""bot.avance(1000,101)
sleep(0.01)
bot.tourne_droit(1000, 370)
sleep(0.01)
bot.avance(1000,300)"""
bot.fete(1001, 1500)