#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, Image, ImageFile, Font
from pybricks.messaging import BluetoothMailboxServer, BluetoothMailboxClient, LogicMailbox, NumericMailbox, TextMailbox
from threading import Thread
from random import choice
from math import fmod
import sys
import os
import math
import struct

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

#####################################################################
#####################################################################
##########~~~~~PROGRAM WRITTEN BY JOZUA VAN RAVENHORST~~~~~##########
##########~~~~~~~~~~~~~~~~T BOT GANTRY ROBOT~~~~~~~~~~~~~~~##########
##########~~~~~~~~~~~~~YOUTUBE CHANNEL: MR JOS~~~~~~~~~~~~~##########
#####################################################################
#####################################################################


##########~~~~~~~~~~HARDWARE CONFIGURATION~~~~~~~~~~##########
ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
arm_motor = Motor(Port.C)
transport_belt = Motor(Port.B)

pin_color = ColorSensor(Port.S4)


##########~~~~~~~~~~MAXIMUM SPEED, MAXIMUM ACCELERATION, MAXIMUM POWER~~~~~~~~~~##########
feed_speed = 900
left_motor.control.limits(900, 3600, 100)
right_motor.control.limits(900, 3600, 100)
arm_motor.control.limits(1400, 3600, 100)
transport_belt.control.limits(900, 720, 100)
x_adjust = 0
y_adjust = 30


##########~~~~~~~~~~BRICK STARTUP SETTINGS~~~~~~~~~~##########
ev3.speaker.set_volume(volume=80, which='_all_')   #Set the volume for all sounds (speaking and beeps etc)
ev3.speaker.set_speech_options(language='en', voice='m7', speed=None, pitch=None)   #Select speaking language, and a voice (male/female)
small_font = Font(size=6)       # 6 pixel height for text on screen
normal_font = Font(size=10)     #10 pixel height for text on screen
big_font = Font(size=16)        #16 pixel height for text on screen
ev3.screen.set_font(normal_font)   #Choose a preset font for writing next texts
ev3.screen.clear()              #Make the screen empty (all pixels white)
ev3.speaker.beep()              #Brick will make a beep sound 1 time
ev3.light.off()                 #Turn the lights off on the brick
#ev3.screen.draw_text(4,  2, "Pin 1: ")     #X/Y position for writing on the screen



##########~~~~~~~~~~CREATING A FILE THAT IS SAVED OFFLINE~~~~~~~~~~##########
#Not used right now, it can be used to store coordinates of positions to go to, and call them back when restarting the program
#create_file = open("saveddata.txt", "a")    #Create a file if it does not exist and open it, if it does exist just open it
#create_file.write("")                       #Write "Nothing" to the file to have atleast 1 line in the file
#create_file.close()                         #Close the file again, to be able to call it later again

def stud_pos(x_studs, y_studs, z_studs, bump):
    next_pos(- x_adjust - y_adjust - 21.74 * x_studs - 21.74 * y_studs, x_adjust - y_adjust + 21.74 * x_studs - 21.74 * y_studs, 50 * z_studs, bump)

def next_pos(l_pos, r_pos, a_pos, bump):
    speed_next = feed_speed / bump
    l_move = math.fabs(left_motor.angle() - l_pos)
    r_move = math.fabs(right_motor.angle() - r_pos)
    #print(l_move, r_move, l_pos, r_pos)
    if l_move >= r_move:
        if r_move == 0:
            left_motor.run_target(speed_next, l_pos, then=Stop.COAST, wait=True)
        else:
            left_motor.run_target(speed_next, l_pos, then=Stop.COAST, wait=False)
            right_motor.run_target(speed_next * r_move / l_move, r_pos, then=Stop.COAST, wait=True)    
    else:
        if l_move == 0:
            right_motor.run_target(speed_next, r_pos, then=Stop.COAST, wait=True)
        else:
            left_motor.run_target(speed_next * l_move / r_move, l_pos, then=Stop.COAST, wait=False)
            right_motor.run_target(speed_next, r_pos, then=Stop.COAST, wait=True)   
    arm_motor.run_target(1200, a_pos)


pin_ready = None
pin_picked_up = False
pin_dropped = True
pin_length = 0


right_motor.run(-55)
left_motor.run_until_stalled(80, then=Stop.COAST, duty_limit= 16)
right_motor.stop()
ev3.speaker.beep()
wait(1000)
left_motor.run(80)
right_motor.run_until_stalled(80, then=Stop.COAST, duty_limit= 16)
left_motor.stop()
wait(1000)
ev3.speaker.beep()
left_motor.reset_angle(-40)  ###-50   -20
right_motor.reset_angle(0)

left_motor.run_target(feed_speed, 0, then=Stop.HOLD, wait=False)
right_motor.run_target(feed_speed, 0, then=Stop.HOLD, wait=True)

wait(200)

ev3.speaker.beep()
ev3.speaker.beep()
stud_pos(10, 3, 0, 1)

arm_motor.run_until_stalled(300, then=Stop.COAST, duty_limit= 40)
arm_motor.reset_angle(350) #720 350
arm_motor.run_target(900, 0)

for x in range(2):
    arm_motor.run_target(1200, 300)
    arm_motor.run_target(1200, 0)


def auto_belt():
    global pin_ready
    global pin_picked_up
    global pin_dropped
    global pin_length
    while True:
        if pin_picked_up == True or pin_dropped == True:
            transport_belt.run(450)
            while True:
                clr_now = pin_color.color()
                if clr_now == Color.BLUE or clr_now == Color.RED:
                    break
            pos_start = transport_belt.angle()
            while pin_color.color() != Color.BLACK:
                continue
            pos_end = transport_belt.angle()
            pin_length = pos_end - pos_start
            while pos_start + 240 > transport_belt.angle():
                continue
            transport_belt.hold()
            while pin_dropped == False:
                continue
            pin_dropped = False
            pin_picked_up = False
            pin_ready = clr_now




sub_auto_belt = Thread(target=auto_belt)
sub_auto_belt.start()

stud_pos( 1, 14, 0, 1)
stud_pos( 1, 18.5, 0, 4)
while True:
    if pin_ready != None:
        stud_pos( 1, 18.5, 6, 4)
        stud_pos( 1, 14, 6, 1)
        pin_picked_up = True
        print(pin_ready, pin_length)
        if pin_ready == Color.BLUE:
            if pin_length < 50:
                stud_pos( 7, 14, 0, 1)
            elif pin_length < 80:
                stud_pos(12, 14, 0, 1)
            else:
                stud_pos(17, 14, 0, 1)
        if pin_ready == Color.RED:
            stud_pos(23, 14, 0, 1) 
        stud_pos( 1, 14, 0, 1)
        stud_pos( 1, 18.5, 0, 4)
        pin_ready = None
        pin_dropped = True


