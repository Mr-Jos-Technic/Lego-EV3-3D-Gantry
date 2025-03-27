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
left_motor = Motor(Port.B)
right_motor = Motor(Port.A)
arm_motor = Motor(Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
drive_motor = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)

pin_color = ColorSensor(Port.S2)
touch_drive = TouchSensor(Port.S4)


##########~~~~~~~~~~MAXIMUM SPEED, MAXIMUM ACCELERATION, MAXIMUM POWER~~~~~~~~~~##########
feed_speed = 900
left_motor.control.limits(900, 3600, 100)
right_motor.control.limits(900, 3600, 100)
arm_motor.control.limits(1400, 3600, 100)
drive_motor.control.limits(900, 3600, 100)
drive_motor.control.target_tolerances(1000, 40)                                 #(50, 5)  = Default
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

timer_pin = StopWatch()
pin_rgb_list = []
pin_values = [0, 0, 0]
pin_list = []
drop_low = True

                #Name           #Amount counted #Dropoff position             #Color sensor  < Length <         < Red <    < Green <   < Blue <
pins_scanned = {"ReScan"     : {"counter" : 0 , "location" : [10,  6,    0] , "dataset" : [    0,    0,         0,    0,    0,    0,    0,    0]} , \
                "Reject"     : {"counter" : 0 , "location" : [10,  6,    0] , "dataset" : [    0,    0,         0,    0,    0,    0,    0,    0]} , \
                "Axle 2L Red": {"counter" : 0 , "location" : [10,  6,  920] , "dataset" : [  300,  390,      14.0, 22.0,  0.0,  2.2,  0.0,  1.0]} , \
                "Axle 3L RB" : {"counter" : 0 , "location" : [10,  6,  770] , "dataset" : [  400,  500,       4.0,  8.0,  1.5,  4.5,  0.5,  6.0]} , \
                "Axle 3L Tan": {"counter" : 0 , "location" : [18,  6,  770] , "dataset" : [  460,  540,       9.0, 11.0,  6.8,  8.5,  6.5,  9.0]} , \
                "Conn 3L Whi": {"counter" : 0 , "location" : [18,  6,  920] , "dataset" : [  400,  650,      15.0, 20.0, 15.0, 20.0, 24.0, 34.0]} , \
                "Black 2L"   : {"counter" : 0 , "location" : [10,  6,  450] , "dataset" : [   45,  255,       0.5,  2.0,  0.5,  1.5,  0.0,  1.0]} , \
                "Black 3L"   : {"counter" : 0 , "location" : [18,  6,    0] , "dataset" : [    0,    0,         0,    0,    0,    0,    0,    0]} , \
                "DBG 3L"     : {"counter" : 0 , "location" : [18,  6,  590] , "dataset" : [  380,  500,       3.5,  4.7,  4.2,  5.3,  6.2,  8.3]} , \
                "DBG 1.5L"   : {"counter" : 0 , "location" : [10,  6,  590] , "dataset" : [  190,  270,       3.5,  5.1,  2.9,  5.1,  2.9,  6.1]} , \
                "Blue 3L"    : {"counter" : 0 , "location" : [18,  6,  300] , "dataset" : [  400,  500,       0.7,  3.0,  4.2,  7.5, 22.0, 40.0]} , \
                "Blue 2L"    : {"counter" : 0 , "location" : [10,  6,  300] , "dataset" : [  260,  370,       0.7,  3.0,  4.2,  7.5, 22.0, 40.0]} , \
                "Blue 1.25L" : {"counter" : 0 , "location" : [ 2,  6,  300] , "dataset" : [  150,  250,       0.7,  3.0,  4.2,  7.5, 22.0, 40.0]} , \
                "Tan 3L"     : {"counter" : 0 , "location" : [18,  6,   20] , "dataset" : [  430,  510,      15.0, 22.0, 13.0, 19.0, 15.0, 24.0]} , \
                "Tan 2L"     : {"counter" : 0 , "location" : [10,  6,   20] , "dataset" : [  320,  410,      15.0, 22.0, 12.0, 19.0, 12.0, 24.0]} , \
                "Tan 1.5L"   : {"counter" : 0 , "location" : [ 2,  6,   20] , "dataset" : [  210,  310,      15.0, 22.0, 13.0, 19.0, 15.0, 24.0]} , \
                "Red 3L"     : {"counter" : 0 , "location" : [18,  6,  450] , "dataset" : [  400,  500,      15.0, 22.0,  1.5,  4.0,  3.0,  5.0]} , \
                "LBG 1.25L"  : {"counter" : 0 , "location" : [ 2,  6,  160] , "dataset" : [  170,  250,       7.0, 13.2,  8.5, 14.2, 13.4, 27.0]} , \
                "LBG 2L"     : {"counter" : 0 , "location" : [10,  6,  160] , "dataset" : [  290,  340,       7.0, 13.2,  8.5, 12.6, 12.4, 24.0]} , \
                "LBG 3L"     : {"counter" : 0 , "location" : [18,  6,  160] , "dataset" : [  420,  500,       7.0, 13.2,  8.5, 12.6, 13.4, 24.0]} }

def check_result_scans():
    for x in pins_scanned:                                                          #It will automatically loop for every pin defined in the dictionary, so you can add your own pins at line 116
        if  pins_scanned[x]["dataset"][ 0] <  pin_list[-1][0] <  pins_scanned[x]["dataset"][ 1] and pins_scanned[x]["dataset"][ 2] <= pin_list[-1][1] <= pins_scanned[x]["dataset"][ 3] and \
            pins_scanned[x]["dataset"][ 4] <= pin_list[-1][2] <= pins_scanned[x]["dataset"][ 5] and pins_scanned[x]["dataset"][ 6] <= pin_list[-1][3] <= pins_scanned[x]["dataset"][ 7]: return x
    return "ReScan"                                                                 #If it does not match with any pin, it will send back that a rescan is needed


##########~~~~~~~~~~CREATING A FILE THAT IS SAVED OFFLINE~~~~~~~~~~##########
#Not used right now, it can be used to store coordinates of positions to go to, and call them back when restarting the program
#create_file = open("saveddata.txt", "a")    #Create a file if it does not exist and open it, if it does exist just open it
#create_file.write("")                       #Write "Nothing" to the file to have atleast 1 line in the file
#create_file.close()                         #Close the file again, to be able to call it later again


def stud_pos(x_studs, y_studs, z_studs, gripper, speed):
    next_pos(- x_adjust - y_adjust - 21.74 * x_studs - 21.74 * y_studs, x_adjust - y_adjust + 21.74 * x_studs - 21.74 * y_studs, z_studs, 50 * gripper, speed)

def next_pos(l_pos, r_pos, y_pos, g_pos, speed):
    speed_next = feed_speed / speed
    l_move = math.fabs(left_motor.angle() - l_pos)
    r_move = math.fabs(right_motor.angle() - r_pos)
    #print(l_move, r_move, l_pos, r_pos)
    if y_pos >= 0:
        drive_motor.run_target(speed_next, y_pos, then=Stop.COAST, wait=False)
    if l_move >= r_move:
        if r_move < 2:
            left_motor.run_target(speed_next, l_pos, then=Stop.COAST, wait=True)
        else:
            left_motor.run_target(speed_next, l_pos, then=Stop.COAST, wait=False)
            right_motor.run_target(speed_next * r_move / l_move, r_pos, then=Stop.COAST, wait=True)    
    else:
        if l_move < 2:
            right_motor.run_target(speed_next, r_pos, then=Stop.COAST, wait=True)
        else:
            left_motor.run_target(speed_next * l_move / r_move, l_pos, then=Stop.COAST, wait=False)
            right_motor.run_target(speed_next, r_pos, then=Stop.COAST, wait=True)
    if y_pos >= 0:
        while drive_motor.control.done() == False: continue
    arm_motor.run_target(1400, g_pos)  #1200
#COAST instead of HOLD normally

pin_length = 0
pin_rgb_list = []
pin_values = [0, 0, 0]

def auto_scan():
    global pin_list
    global pin_rgb_list
    global pin_values
    global pin_length

    while True:
        clr_now = pin_color.rgb()
        if clr_now != (0, 0, 0):
            pin_start = timer_pin.time()
            while clr_now != (0, 0, 0):
                pin_rgb_list.append(clr_now)
                clr_now = pin_color.rgb()
            pin_end = timer_pin.time()
            pin_length = pin_end - pin_start
            if len(pin_rgb_list) <= 60:
                if len(pin_rgb_list) > 20:          #Extra short detection for black 2L pins
                    if 0 <= pin_rgb_list[math.ceil(len(pin_rgb_list) / 2)][0] <= 2 and 0 <= pin_rgb_list[math.ceil(len(pin_rgb_list) / 2)][1] <= 2 and 0 <= pin_rgb_list[math.ceil(len(pin_rgb_list) / 2)][2] <= 1:
                        pin_list.append([pin_length, 0, 0, 0, pin_start + (pin_length / 2), "Black 2L"])
                        pins_scanned["Black 2L"]["counter"] += 1
                        print("Black 2L - Detected by short version")
                        pin_rgb_list = []
                        ev3.speaker.beep()
                        continue
                else:
                    print("To short detection" , pin_length, "ms long", len(pin_rgb_list), "measuring points")
                    pin_rgb_list = []
                    continue

            data_points = len(pin_rgb_list) - 60
            for x in range(data_points):
                for y in range(3):
                    pin_values[y] += pin_rgb_list[30 + x][y]
            for x in range(3):
                pin_values[x] /= data_points
            pin_list.append([pin_length, round(pin_values[0], 2), round(pin_values[1], 2), round(pin_values[2], 2), pin_start + (pin_length / 2)])
            pin_result = check_result_scans()
            pin_list[-1].extend([pin_result])
            pins_scanned[pin_result]["counter"] += 1
            if pin_result == "ReScan": print(pin_list[-1])
            else: print(pin_result)
            pin_rgb_list = []
            pin_values = [0, 0, 0]
            ev3.speaker.beep()
            wait(200)


right_motor.run(-75)
left_motor.run_until_stalled(80, then=Stop.COAST, duty_limit= 25)
right_motor.stop()
ev3.speaker.beep()
wait(1000)
left_motor.run(80)
right_motor.run_until_stalled(80, then=Stop.COAST, duty_limit= 20)
left_motor.stop()
wait(1000)
ev3.speaker.beep()
left_motor.reset_angle(-40)  ###-50   -20    -40
right_motor.reset_angle(0)

drive_motor.run(-100)
while touch_drive.pressed() == False: continue
drive_motor.hold()
drive_motor.reset_angle(0)

left_motor.run_target(feed_speed, 0, then=Stop.HOLD, wait=False)
right_motor.run_target(feed_speed, 0, then=Stop.HOLD, wait=True)


ev3.speaker.beep()
ev3.speaker.beep()

stud_pos(10, 3, 0, 0, 1)

arm_motor.run_until_stalled(300, then=Stop.COAST, duty_limit= 40)
arm_motor.reset_angle(350) #720 350
arm_motor.run_target(1400, 0)

sub_auto_scan = Thread(target=auto_scan)
sub_auto_scan.start()

pin_handled = 0

while True:
    if len(pin_list) <= pin_handled: stud_pos( 0.6,  6,  1210, 0, 1)
    else:
        while len(pin_list[pin_handled]) < 6: continue
        if pin_list[pin_handled][5] == "ReScan":
            pin_handled += 1
            continue
        else:
            adjust_pickup_pos = ((timer_pin.time() - pin_list[pin_handled][4]) / 8) + ((math.fabs(((timer_pin.time() - pin_list[pin_handled][4]) / 8) - drive_motor.angle())) / 5)
            if adjust_pickup_pos > 710:             #Prevent running into the bumpers of the rails
                pins_scanned[pin_list[pin_handled][5]]["counter"] -= 1
                pins_scanned["Reject"]["counter"] += 1
                pin_handled += 1
                continue
            else: stud_pos( 0.6,  6, 1210 - adjust_pickup_pos, 0, 1)
    if len(pin_list) <= pin_handled:
        while len(pin_list) <= pin_handled: continue
    else:
        while len(pin_list[pin_handled]) < 6: continue
        if pin_list[pin_handled][5] == "ReScan":
            pin_handled += 1
            continue
        else:
            adjust_pickup_pos = ((timer_pin.time() - pin_list[pin_handled][4]) / 8.5)
            if adjust_pickup_pos > 710:             #Prevent running into the bumpers of the rails 
                pins_scanned[pin_list[pin_handled][5]]["counter"] -= 1
                pins_scanned["Reject"]["counter"] += 1
                pin_handled += 1
                continue
            else: drive_motor.track_target(1210 - adjust_pickup_pos)
    while len(pin_list[pin_handled]) < 6: continue
    if pin_list[pin_handled][5] == "ReScan":
        pin_handled += 1
        continue
    else:
        drive_motor.run(-90)
        stud_pos( 0.6, 7.6,   -1, 6, 10)                                                #( 0.6, 7.6,   -1, 6, 10)
        stud_pos( 0.6, 6,   -1, 6, 1)
        dropoff_pos = pins_scanned[pin_list[pin_handled][5]]["location"]
        if drop_low == True:
            stud_pos(dropoff_pos[0], dropoff_pos[1], dropoff_pos[2], 6, 1)
            stud_pos(dropoff_pos[0], 19            , -1            , 0, 1)
            stud_pos(dropoff_pos[0], dropoff_pos[1], -1            , 0, 1)
        else:
            stud_pos(dropoff_pos[0], dropoff_pos[1], dropoff_pos[2], 0, 1)
        pin_handled += 1

















while True:
    random_one   = choice([1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20])
    random_two   = choice([1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18])
    random_three = 50 * choice([1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25])
    random_four  = choice([0,6])
    stud_pos(random_one, random_two, random_three, random_four, 1)


stud_pos( 1, 14, 0, 0, 1)
stud_pos( 1, 18.5, 0, 0, 4)



while True:
    stud_pos(20, 18.5, 500, 0, 1)
    stud_pos( 4, 18.5, 40, 6, 1)
    stud_pos( 4, 1, 1250, 6, 1)
    stud_pos( 4, 5, 40, 0, 1)


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


