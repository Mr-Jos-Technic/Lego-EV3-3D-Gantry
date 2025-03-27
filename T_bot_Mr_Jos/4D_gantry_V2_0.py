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
right_motor = Motor(Port.B)
arm_motor = Motor(Port.C) #, positive_direction=Direction.COUNTERCLOCKWISE)
drive_motor = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)

pin_color = ColorSensor(Port.S2)
touch_drive = TouchSensor(Port.S4)


##########~~~~~~~~~~MAXIMUM SPEED, MAXIMUM ACCELERATION, MAXIMUM POWER~~~~~~~~~~##########
feed_speed = 1050 #1400 possible?
left_motor.control.limits   (1400, 3600, 100)   #9000
right_motor.control.limits  (1400, 3600, 100)   #9000
arm_motor.control.limits    (1400, 3600, 100)
drive_motor.control.limits  ( 820, 3600, 100)
drive_motor.control.target_tolerances(800, 2)                                 #(50, 5)  = Default #(1000, 40)
left_motor.control.target_tolerances (50, 5)     #NEW 5,1
right_motor.control.target_tolerances(50, 5)     #NEW 5,1
x_adjust = 0
z_adjust = -5    #30 negative = go higher
chain_gear      = 24
track_length    = 2490
track_width     = 17
track_depth     = 8.5
arm_max_closing = 225
balls_in_row    = 11        #Amount of balls for each color to sort before returning them
ball_grip_power = 80        #75 = losing balls
store_distance  = 136
scan_depth      = 3.9


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
stud_constant   = 870
pin_location    = 0
#empty_belt = [ 6,  8,  5, 11, 15, 11] #1Stud high, for pins
empty_belt = [ 0,  0,  0, 8, 10, 8]
pin_list = []
pins_full_checked = 0
machine_on = True


                #Name           #Amount counted #Dropoff position             #Color sensor < Length <        < Red <    < Green <   < Blue <
ins_scanned = {"ReScan"     : {"counter" : 0 , "location" : [10,  6,    0] , "dataset" : [   0,   0,         0,    0,    0,    0,    0,    0]} , \
                "Reject"     : {"counter" : 0 , "location" : [10,  6,    0] , "dataset" : [   0,   0,         0,    0,    0,    0,    0,    0]} , \
                "White 3L"   : {"counter" : 0 , "location" : [17,  7, 1000] , "dataset" : [ 170, 210,        20,   36,   24,   40,   45,   55]} , \
                "Black 3L"   : {"counter" : 0 , "location" : [17,  7, 1200] , "dataset" : [ 130, 175,         2,    7,    3,    8,    0,    3]} , \
                "Black 2L"   : {"counter" : 0 , "location" : [13,  7, 1200] , "dataset" : [  80, 120,         2,    7,    3,    8,    0,    3]} , \
                "Red 3L"     : {"counter" : 0 , "location" : [17,  7, 2200] , "dataset" : [ 130, 175,        20,   26,    9,   11,    3,    7]} , \
                "Red 1.25L"  : {"counter" : 0 , "location" : [ 5,  7, 2200] , "dataset" : [  65, 105,        15,   26,    9,   12,    3,    7]} , \
                "DBG 3L"     : {"counter" : 0 , "location" : [17,  7, 2000] , "dataset" : [   0,   0,         0,    0,    0,    0,    0,    0]} , \
                "DBG 1.5L"   : {"counter" : 0 , "location" : [ 9,  7, 2000] , "dataset" : [   0,   0,         0,    0,    0,    0,    0,    0]} , \
                "Blue 3L"    : {"counter" : 0 , "location" : [17,  7, 1400] , "dataset" : [ 134, 175,         4,   10,   12, 16.5,   20,   40]} , \
                "Blue 2L"    : {"counter" : 0 , "location" : [13,  7, 1400] , "dataset" : [  95, 120,         4,   10,   12, 16.5,   20,   40]} , \
                "Blue 1.25L" : {"counter" : 0 , "location" : [ 5,  7, 1400] , "dataset" : [  70,  85,         4,   10,   12, 16.5,   20,   40]} , \
                "Tan 3L"     : {"counter" : 0 , "location" : [17,  7, 1800] , "dataset" : [ 150, 180,        20,   30,   17,   27,   18,   26]} , \
                "Tan 2L"     : {"counter" : 0 , "location" : [13,  7, 1800] , "dataset" : [ 105, 134,        20,   30,   17,   27,   18,   26]} , \
                "Tan 1.5L"   : {"counter" : 0 , "location" : [ 9,  7, 1800] , "dataset" : [  80, 100,        20,   30,   17,   27,   18,   26]} , \
                "LBG 3L"     : {"counter" : 0 , "location" : [17,  7, 1600] , "dataset" : [ 125, 175,        13,   18,   17,   22,   18,   28]} , \
                "LBG 2L"     : {"counter" : 0 , "location" : [13,  7, 1600] , "dataset" : [  90, 100,        13,   20,   17,   22,   18,   32]} , \
                "LBG 1.25L"  : {"counter" : 0 , "location" : [ 5,  7, 1600] , "dataset" : [  60,  75,        13,   18,   17,   22,   18,   32]} , \
                "Green 1.25L": {"counter" : 0 , "location" : [ 5,  7, 1200] , "dataset" : [   0,   0,         0,    0,    0,    0,    0,    0]} }


                #Name           #Amount counted #Dropoff position             #Color sensor < Length <        < Red <    < Green <   < Blue <
pins_scanned = {"ReScan"     : {"counter" : 0 , "location" : [10,  6,    0] , "dataset" : [   0,   0,         0,    0,    0,    0,    0,    0]} , \
                "Reject"     : {"counter" : 0 , "location" : [10,  6,    0] , "dataset" : [   0,   0,         0,    0,    0,    0,    0,    0]} , \
                "White Ball" : {"counter" : 0 , "location" : [15.5,  8,  910] , "dataset" : [ 75, 140,        20,   60,   20,   60,   25,   70]} , \
                "Red Ball"   : {"counter" : 0 , "location" : [12.5,  8,  910] , "dataset" : [ 65, 120,        17,   45,    0,    5,    0,    5]} , \
                "Blue Ball"  : {"counter" : 0 , "location" : [ 9.5,  8,  910] , "dataset" : [ 75, 120,         0,    8,    5,   25,   18,   50]} , \
                "Yellow Ball": {"counter" : 0 , "location" : [ 6.5,  8,  910] , "dataset" : [ 75, 140,        20,   60,   13,   40,    0,    8]} }


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


def stud_pos(x_studs, z_studs, y_degrees, grip_power, speed, scan_clr, endgrip):
    if x_studs < 0: x_studs = 0
    elif x_studs > track_width: x_studs = track_width
    if z_studs < 0: z_studs = 0
    elif z_studs > track_depth: z_studs = track_depth
    if grip_power < 0: grip_power = 0
    elif grip_power > 100: grip_power = 100
    next_pos(x_adjust + z_adjust + stud_constant / chain_gear * x_studs + stud_constant / chain_gear * z_studs, - x_adjust - z_adjust + stud_constant / chain_gear * x_studs - stud_constant / chain_gear * z_studs, y_degrees, grip_power * arm_max_closing / 100, speed, scan_clr, endgrip)


def next_pos(l_pos, r_pos, y_pos, gripper, speed, scan_clr, endgrip):
    global pin_location
    global pin_list

    pin_started = False
    clr_picked  = False

    pin_rgb_list= []
    pin_values  = [0, 0, 0]
    pin_length  = 0

    speed_next = feed_speed * speed
    l_move = math.fabs(left_motor.angle() - l_pos)
    r_move = math.fabs(right_motor.angle() - r_pos)
    #print(l_move, r_move, l_pos, r_pos)
    while machine_on == False: wait(100)
    if endgrip == False: arm_motor.run_target(1400, gripper, then=Stop.COAST, wait=False)
    if y_pos >= 0:
        if y_pos < track_length: drive_motor.run_target(speed_next, y_pos, then=Stop.COAST, wait=False)
        else: drive_motor.run_target(speed_next, track_length, then=Stop.COAST, wait=False)
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
        while drive_motor.control.done() == False:
            if scan_clr == True:
                clr_now = pin_color.rgb()
                if clr_now[0] < empty_belt[0] or clr_now[0] > empty_belt[3] or clr_now[1] < empty_belt[1] or clr_now[1] > empty_belt[4] or clr_now[2] < empty_belt[2] or clr_now[2] > empty_belt[5]:
                    print(clr_now) #For background checking
                    pin_started = True
                    pin_start = drive_motor.angle()
                    while clr_now[0] < empty_belt[0] or clr_now[0] > empty_belt[3] or clr_now[1] < empty_belt[1] or clr_now[1] > empty_belt[4] or clr_now[2] < empty_belt[2] or clr_now[2] > empty_belt[5]:
                        clr_now = pin_color.rgb()
                        if drive_motor.angle() > pin_start + 10:
                            pin_rgb_list.append(clr_now)
                            clr_picked = True
                        if drive_motor.control.done() == True: 
                            pin_started = False
                            break #No pin end found while scanning
                    if pin_started == True:
                        pin_end = drive_motor.angle()
                        pin_length = pin_end - pin_start
                        if pin_length < 20:
                            pin_started = False
                            clr_picked  = False
                            pin_rgb_list = []
                        else:
                            for x in range(len(pin_rgb_list) / 3):
                                for y in range(3):
                                    pin_values[y] += pin_rgb_list[x][y]
                            for x in range(3):
                                pin_values[x] /= (len(pin_rgb_list) / 3)
                            pin_list.append([pin_length, round(pin_values[0], 2), round(pin_values[1], 2), round(pin_values[2], 2), pin_start + (pin_length / 3)]) # / 2 for pins
                            pin_result = check_result_scans()
                            pin_list[-1].extend([pin_result])
                            pins_scanned[pin_result]["counter"] += 1
                            if pin_result == "ReScan": 
                                print(pin_list[-1])
                                pin_rgb_list= []
                                pin_values  = [0, 0, 0]
                                pin_length  = 0
                                del pin_list[pins_full_checked:]
                            else: print(pin_result)
                            drive_motor.stop()
                            #pin_location = pin_start + ((pin_end - pin_start) / 2)       #Halfway the pin
                        
            else: continue
    if endgrip == True: arm_motor.run_target(1400, gripper)  #1200
#COAST instead of HOLD normally


def scan_sort_pins():
    global pins_full_checked
    
    while True:
        while machine_on == False: wait(100)
        stud_pos(  0,   0,  400,   0,    1, False, False)
        stud_pos(  0,   6,  400,   0,    1, False, True)
        stud_pos(  0,   6,  900,   0, 0.25, True,  True)   #0.05 speed ok, 0.25 also, maybe 1 (100%) also?!
        if pins_full_checked < len(pin_list):
            stud_pos(  5, 5.6,  pin_list[pins_full_checked][4],   0,    1, False, True)
            stud_pos(  5, 6.6,  pin_list[pins_full_checked][4],  90,  0.1, False, True)
            stud_pos(  5,   5,  pin_list[pins_full_checked][4],  90,    1, False, True)
            stud_pos( pins_scanned[pin_list[pins_full_checked][5]]["location"][0], 5                                                          , pins_scanned[pin_list[pins_full_checked][5]]["location"][2],  90,    1, False, True)
            stud_pos( pins_scanned[pin_list[pins_full_checked][5]]["location"][0], pins_scanned[pin_list[pins_full_checked][5]]["location"][1], pins_scanned[pin_list[pins_full_checked][5]]["location"][2],   0,  0.3, False, True)
            stud_pos( pins_scanned[pin_list[pins_full_checked][5]]["location"][0], 5                                                          , pins_scanned[pin_list[pins_full_checked][5]]["location"][2],   0,    1, False, True)
            pins_full_checked += 1


def scan_sort_balls():
    global pins_full_checked
    
    while True:
        stud_pos(  7,   0         ,  100,   0,    1, False, False)
        stud_pos(  7,   scan_depth,  100,   0,    1, False, True)
        stud_pos(  7,   scan_depth,  920,   0, 0.25, True , True)   #0.05 speed ok, 0.25 also, maybe 1 (100%) also?!
        if pins_full_checked < len(pin_list):
            stud_pos(  7,   scan_depth,  pin_list[pins_full_checked][4],   0,    1, False, True)
            stud_pos( 12,   scan_depth + 0.5,  pin_list[pins_full_checked][4],  ball_grip_power,    1, False, True)
            stud_pos( 12,   scan_depth - 1.8,  pin_list[pins_full_checked][4],  ball_grip_power,    1, False, True)
            if pins_scanned[pin_list[pins_full_checked][5]]["location"][0] > 14 and drive_motor.angle() < 500: #To prevent a crash against the support pillar
               stud_pos( 12                                                      , scan_depth - 1.8                                             , 700                                                                                                                                   ,  ball_grip_power,    1, False, True) 
            stud_pos( pins_scanned[pin_list[pins_full_checked][5]]["location"][0], scan_depth - 1.8                                             , pins_scanned[pin_list[pins_full_checked][5]]["location"][2] + pins_scanned[pin_list[pins_full_checked][5]]["counter"] * store_distance,  ball_grip_power,    1, False, True)
            stud_pos( pins_scanned[pin_list[pins_full_checked][5]]["location"][0], pins_scanned[pin_list[pins_full_checked][5]]["location"][1], pins_scanned[pin_list[pins_full_checked][5]]["location"][2] + pins_scanned[pin_list[pins_full_checked][5]]["counter"] * store_distance,               30,  0.3, False, True)
            stud_pos( pins_scanned[pin_list[pins_full_checked][5]]["location"][0], scan_depth                                                 , pins_scanned[pin_list[pins_full_checked][5]]["location"][2] + pins_scanned[pin_list[pins_full_checked][5]]["counter"] * store_distance,                0,    1, False, False)
            if pins_scanned[pin_list[pins_full_checked][5]]["counter"] == balls_in_row:
                stud_pos( pins_scanned[pin_list[pins_full_checked][5]]["location"][0], 6                                                          , pins_scanned[pin_list[pins_full_checked][5]]["location"][2] + store_distance,  ball_grip_power                 ,    1, False, False)
                for x in range(balls_in_row):
                    stud_pos( pins_scanned[pin_list[pins_full_checked][5]]["location"][0], 6                                                          , pins_scanned[pin_list[pins_full_checked][5]]["location"][2] + (x + 1) * store_distance,  30                 ,    1, False, True)
                    stud_pos( pins_scanned[pin_list[pins_full_checked][5]]["location"][0], pins_scanned[pin_list[pins_full_checked][5]]["location"][1] + 0.2, pins_scanned[pin_list[pins_full_checked][5]]["location"][2] + (x + 1) * store_distance,  ball_grip_power    ,    1, False, True)
                    if pins_scanned[pin_list[pins_full_checked][5]]["location"][0] < 13:
                        stud_pos( pins_scanned[pin_list[pins_full_checked][5]]["location"][0], 0                                                          , pins_scanned[pin_list[pins_full_checked][5]]["location"][2] + (x + 1) * store_distance,  ball_grip_power,    1, False, True)
                    else:
                        stud_pos( pins_scanned[pin_list[pins_full_checked][5]]["location"][0], 4                                                          , pins_scanned[pin_list[pins_full_checked][5]]["location"][2] + (x + 1) * store_distance,  ball_grip_power,    1, False, True)
                    stud_pos( 0.5                                                            , 0                                                          , pins_scanned[pin_list[pins_full_checked][5]]["location"][2] + (x + 1) * store_distance,  30             ,    1, False, True)
                    if pins_scanned[pin_list[pins_full_checked][5]]["location"][0] < 10:
                        stud_pos( 4.5                                                        , 0                                                          , pins_scanned[pin_list[pins_full_checked][5]]["location"][2] + (x + 1) * store_distance,  30             ,    1, False, True)
                pins_scanned[pin_list[pins_full_checked][5]]["counter"] = 0
                stud_pos(       3,    0,  450        ,  90,    1, False, False)
                stud_pos(       3,    4,  450        ,  90,  0.1, False, True)
                for x in range(10):
                    stud_pos(   3,  4.5,  450        ,  90,  0.5, False, True)
                    stud_pos(   3,    4,  450        ,  90,  0.5, False, True)  
                stud_pos(       3,    0,  450        ,  90,    1, False, True)
            new_ball_lever()
            pins_full_checked += 1
        elif drive_motor.angle() > 900:
            new_ball_lever()


def new_ball_lever():
    stud_pos(  3.5,   0,  250,   ball_grip_power,    1, True, False)
    stud_pos(  3.5,   8,  250,   ball_grip_power,    1, True, True)
    stud_pos(  3.5,   0,  250,   ball_grip_power,    1, True, True)


#####HOMING#####
def homing():
    global machine_on

    arm_motor.run_target(1400, -50)
    right_motor.run(75)
    left_motor.run_until_stalled(-80, then=Stop.COAST, duty_limit= 60)  #40
    right_motor.stop()
    ev3.speaker.beep()
    wait(1000)
    left_motor.run(-80)
    right_motor.run_until_stalled(-80, then=Stop.COAST, duty_limit= 60)  #40
    left_motor.stop()
    wait(1000)
    ev3.speaker.beep()
    left_motor.reset_angle(-20)  ###   -40
    right_motor.reset_angle(0)

    stud_pos(track_width/2, 0, 0, 0, 1, False, True)

    if touch_drive.pressed() == True: drive_motor.run_angle(200, 180)
    drive_motor.run(-500)
    while touch_drive.pressed() == False: continue
    drive_motor.hold()
    drive_motor.reset_angle(-10)

    stud_pos(track_width/2, track_depth/4, 0, 0, 1, False, True)

    arm_motor.run_until_stalled(300, then=Stop.COAST, duty_limit= 40)
    arm_motor.reset_angle(arm_max_closing+50)
    arm_motor.run_target(1400, 0)

    stud_pos(track_width/2, 0, 450, 0, 1, False, True)
    machine_on = False


##########~~~~~~~~~~SCREEN OPERATION~~~~~~~~~~##########
def pushingbuttons():                                                               #Function to wait for a button to be pressed on the EV3 brick, and return which one was pressed
    while True:
        if   ev3.buttons.pressed() == [Button.UP]:
            wait_for_release_buttons()
            return "up"
        if ev3.buttons.pressed() == [Button.DOWN]:
            wait_for_release_buttons()
            return "down"
        elif ev3.buttons.pressed() == [Button.LEFT]:
            wait_for_release_buttons()
            return "left"
        elif ev3.buttons.pressed() == [Button.RIGHT]:
            wait_for_release_buttons()
            return "right"
        elif ev3.buttons.pressed() == [Button.CENTER]:
            wait_for_release_buttons()
            return "center"


def wait_for_release_buttons():                                                     #Function to wait for the EV3 buttons to be all released
    while ev3.buttons.pressed() != []: continue


def clear_screen():
    ev3.screen.clear()                                                                  #Empty the complete screen on the EV3 brick
    ev3.screen.draw_text(103, 114, "Mr Jos creation", text_color=Color.BLACK, background_color=Color.WHITE)     #Write text on the EV3 screen on the XY grid


sub_scan_sort_pins = Thread(target=scan_sort_pins)
sub_scan_sort_balls = Thread(target=scan_sort_balls)

clear_screen()
homing()
#sub_scan_sort_pins.start()
sub_scan_sort_balls.start()


while True:
    lastpress = pushingbuttons()
    if   lastpress == "center":                                                     #If the last button press was the center button;
        if   machine_on == True:  machine_on = False                                                 #Start running program is selected, so start normal operation
        elif machine_on == False: machine_on = True
    elif lastpress == "down": z_adjust += 2                    #Move the cursor position one line down
    elif lastpress == "up"  : z_adjust -= 2                    #Move the cursor position one line up
    elif lastpress == "left": continue
