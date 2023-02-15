#!/usr/bin/env pybricks-micropython
from time import sleep
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase


class Robot64ans:

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


class Robot:
    """crée un robot de manière cool"""
    def __init__(self):
        right_motor = Motor(Port.B)
        left_motor = Motor(Port.C)

        line_sensor_d = ColorSensor(Port.S2)
        line_sensor_g = ColorSensor(Port.S3)

        self.robot = DriveBase(left_motor, right_motor, wheel_diameter=32, axle_track=135)

    def avancer(self, distance, vitesse):
        """fait avancer le robot de distance mm"""
        self.robot.settings(straight_speed=vitesse)
        self.robot.straight(-distance)

    def tourne(self, degres, vitesse):
        """fait pivoter le robot de degrés degres à vitesse vitesse"""
        print("parametrage")
        # self.robot.settings(turn_rate=vitesse)
        print("parametré")
        self.robot.turn(degres)
        print("tourné")
    
robot = Robot()

robot.avancer(100, 50)
robot.tourne(90, 90)