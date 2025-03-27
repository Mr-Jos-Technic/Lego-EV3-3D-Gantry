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
feed_speed = 820
left_motor.control.limits   ( 820, 3600, 100)
right_motor.control.limits  ( 820, 3600, 100)
arm_motor.control.limits    (1400, 3600, 100)
drive_motor.control.limits  ( 820, 3600, 100)
drive_motor.control.target_tolerances(800, 2)                                 #(50, 5)  = Default #(1000, 40)
left_motor.control.target_tolerances (5, 1)     #NEW
right_motor.control.target_tolerances(5, 1)     #NEW
x_adjust = 0
z_adjust = 30
chain_gear = 24


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
                "Axle 2L Red": {"counter" : 0 , "location" : [10,  6,  920] , "dataset" : [  900, 1100,      14.0, 22.0,  0.0,  2.2,  0.0,  1.0]} , \
                "Axle 3L RB" : {"counter" : 0 , "location" : [10,  6,  770] , "dataset" : [ 1150, 1350,       3.9,  8.0,  1.5,  4.5,  0.5,  6.0]} , \
                "Axle 3L Tan": {"counter" : 0 , "location" : [18,  6,  770] , "dataset" : [ 1450, 1600,       8.4, 11.0,  6.8,  8.5,  6.4,  9.0]} , \
                "Conn 3L Whi": {"counter" : 0 , "location" : [18,  6,  920] , "dataset" : [ 1600, 1800,      15.0, 20.0, 15.0, 21.0, 24.0, 35.0]} , \
                "Black 2L"   : {"counter" : 0 , "location" : [10,  6,  450] , "dataset" : [  150,  400,       0.5,  2.0,  0.5,  1.5,  0.0,  1.0]} , \
                "Black 3L"   : {"counter" : 0 , "location" : [18,  6,    0] , "dataset" : [    0,    0,         0,    0,    0,    0,    0,    0]} , \
                "DBG 3L"     : {"counter" : 0 , "location" : [18,  6,  590] , "dataset" : [ 1350, 1500,       3.5,  4.7,  4.2,  5.3,  6.2,  8.3]} , \
                "DBG 1.5L"   : {"counter" : 0 , "location" : [10,  6,  590] , "dataset" : [  600,  750,       3.5,  5.1,  2.9,  5.1,  2.9,  6.6]} , \
                "Blue 3L"    : {"counter" : 0 , "location" : [18,  6,  300] , "dataset" : [ 1350, 1500,       0.7,  3.0,  4.2,  7.5, 22.0, 40.0]} , \
                "Blue 2L"    : {"counter" : 0 , "location" : [10,  6,  300] , "dataset" : [  900, 1100,       0.7,  3.0,  4.2,  7.5, 22.0, 40.0]} , \
                "Blue 1.25L" : {"counter" : 0 , "location" : [ 2,  6,  300] , "dataset" : [  600,  750,       0.7,  3.0,  4.2,  7.5, 18.0, 40.0]} , \
                "Tan 3L"     : {"counter" : 0 , "location" : [18,  6,   20] , "dataset" : [ 1400, 1500,      15.0, 22.0, 13.0, 19.0, 15.0, 24.0]} , \
                "Tan 2L"     : {"counter" : 0 , "location" : [10,  6,   20] , "dataset" : [  950, 1150,      15.0, 22.0, 12.0, 19.0, 12.0, 24.0]} , \
                "Tan 1.5L"   : {"counter" : 0 , "location" : [ 2,  6,   20] , "dataset" : [  750,  850,      15.0, 22.0, 12.0, 19.0, 13.0, 24.0]} , \
                "Red 3L"     : {"counter" : 0 , "location" : [18,  6,  450] , "dataset" : [ 1350, 1500,      13.0, 22.0,  1.5,  4.0,  3.0,  5.0]} , \
                "LBG 1.25L"  : {"counter" : 0 , "location" : [ 2,  6,  160] , "dataset" : [  600,  750,       7.0, 13.2,  8.5, 14.2, 13.4, 27.0]} , \
                "LBG 2L"     : {"counter" : 0 , "location" : [10,  6,  160] , "dataset" : [  950, 1100,       7.0, 13.2,  8.5, 12.6, 12.4, 24.0]} , \
                "LBG 3L"     : {"counter" : 0 , "location" : [18,  6,  160] , "dataset" : [ 1350, 1500,       7.0, 13.2,  8.5, 12.6, 13.4, 24.0]} }

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


def stud_pos(x_studs, z_studs, y_degrees, gripper, speed, pre_drop):
    next_pos(- x_adjust - z_adjust - 870 / chain_gear * x_studs - 870 / chain_gear * z_studs, x_adjust - z_adjust + 870 / chain_gear * x_studs - 870 / chain_gear * z_studs, y_degrees, 50 * gripper, speed, pre_drop)

def next_pos(l_pos, r_pos, y_pos, g_pos, speed, pre_drop):
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
        while drive_motor.control.done() == False:
            if pre_drop == True:
                pre_drop = False
                stud_pos(wait_pos_x, pick_pos_z, no_drive_y, clamp_open, high_speed, False)
            else: continue
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
            pin_list.append([pin_length, round(pin_values[0], 2), round(pin_values[1], 2), round(pin_values[2], 2), pin_start + 300])# + (pin_length / 2)])
            pin_result = check_result_scans()
            pin_list[-1].extend([pin_result])
            pins_scanned[pin_result]["counter"] += 1
            if pin_result == "ReScan": print(pin_list[-1])
            else: print(pin_result)
            pin_rgb_list = []
            pin_values = [0, 0, 0]
            ev3.speaker.beep()
            wait(200)


fronttime = []
backtime  = []
counter = 0

while False: ##Maximal measured speed is ~800°/s
    counter += 1
    timer_pin.reset()
    drive_motor.run_target(800, -1100, then=Stop.HOLD, wait=False)
    while drive_motor.control.done() == False: continue
    fronttime.append(timer_pin.time())
    print("Measure points %s. Current speed %s°/s, time since start %sms. Average time %sms. Current motor position %s."%(counter, drive_motor.speed(), timer_pin.time(), sum(fronttime)/len(fronttime) , drive_motor.angle()))
    wait(20000)
    timer_pin.reset()
    drive_motor.run_target(800, 0, then=Stop.HOLD, wait=False)
    while drive_motor.control.done() == False: continue
    backtime.append(timer_pin.time())
    print("Measure points %s. Current speed %s°/s, time since start %sms. Average time %sms. Current motor position %s."%(counter, drive_motor.speed(), timer_pin.time(), sum(backtime)/len(backtime) , drive_motor.angle()))
    wait(20000)


#####HOMING#####
arm_motor.run_target(1400, -50)
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

stud_pos(10, 3, 0, 0, 1, False)

arm_motor.run_until_stalled(300, then=Stop.COAST, duty_limit= 40)
arm_motor.reset_angle(350) #720 350
arm_motor.run_target(1400, 0)

sub_auto_scan = Thread(target=auto_scan)
sub_auto_scan.start()

pin_handled = 0

def print_drive_angle():
    print(drive_motor.angle())



#       |===|===============================|===|
#       |   |                               |   |
#       | _ |\     ||                       | _ |\
#       ||_|| \    ||                       ||_|| \  ⤡ Z-direction 
#       |   |\ \   ||# Color sensor         |   |\ \
#       |   | \ \  ||- wait_pos_y           |   | \ \
#       | _ |      ||\                      | _ |
#       ||_||      || \                     ||_||
#       |   |      ||\                      |   |
#       |   |      || \                     |   |
#       |   |      ||- max_pos_y            |   |
#       |   |               ^               |   |
#       |   |               |Y-direction    |   |
#       |   |               ⌄               |   |
#       |   |                               |   |
#       |===|===============================|===|
#         \ \        <-> X-direction          \ \
#          \ \                                 \ \
#           \ \                                 \ \
#            \ \                                 \ \

high_speed      = 1         #Dividing factor for the max speed
low_speed       = 10        #Dividing factor for the max speed
clamp_open      = 0         #Position for a opened headpiece
clamp_closed    = 5         #(force/distance setting for closing the headpiece)
wait_pos_x      = 0.6       #(studs) to the right, after homing left
wait_pos_y      = 1210      #(°) after homing at the front
no_drive_y      = -1        #No given speed for driving (if already another drive is given it continues this old one)
wait_pos_z      = 5.5       #(studs) down after top homing, safe height waiting for a pin pickup
pick_pos_z      = 7.3       #(studs) down after top homing, for picking a pin from the conveyor belt
drop_pos_z      = 23        #(studs) down after top homing, dropoff height
max_pos_y       = 550       #(°) 512
y_follow_speed  = -42       #(°/sec) Speed of the ganty driving motor when following the belt at the same speed
deg_per_ms      = 0.042     #(°/ms) Speed of the conveyor belt (and gantry when following the belt = 42/1000)
deg_per_mm      = 2.36559   #(°/mm) Motor angle to run for 1mm displacement
dist_wait_pos   = 20        #(mm) Center color sensor, to center pin at waiting position distance
time_wait_pos   = dist_wait_pos * deg_per_mm / deg_per_ms   #(23 * 2.36559 / 0.042 = 1295ms)
pre_lowering    = True


while True:
    if len(pin_list) <= pin_handled: stud_pos( wait_pos_x, wait_pos_z, wait_pos_y , clamp_open, high_speed, pre_lowering) #If there is no new pin being started scanning, go to default wait position
    else:
        while len(pin_list[pin_handled]) < 6: continue                                              #Waiting for a full argument list for the next pin
        if pin_list[pin_handled][5] == "ReScan":                                                    #If the result is undefined, return to start
            pin_handled += 1
            continue
        else:
            pickup_pos_y = wait_pos_y - ((timer_pin.time() - (pin_list[pin_handled][4] + time_wait_pos)) * deg_per_ms)
            current_pos_y = drive_motor.angle()
            fly_distance_y = math.fabs(current_pos_y - pickup_pos_y)
            if current_pos_y < pickup_pos_y:                                                        #If the gantry is in the front
                projected_pickup_pos_y = pickup_pos_y - (fly_distance_y * 1.7 * deg_per_ms)
                print(pickup_pos_y, current_pos_y, projected_pickup_pos_y)
            else:
                projected_pickup_pos_y = pickup_pos_y                                               ####Add more
            if projected_pickup_pos_y < max_pos_y:                                                  #Prevent running into the bumpers of the rails
                pins_scanned[pin_list[pin_handled][5]]["counter"] -= 1
                pins_scanned["Reject"]["counter"] += 1
                pin_handled += 1
                continue
            else: stud_pos(wait_pos_x, wait_pos_z, projected_pickup_pos_y, clamp_open, high_speed, pre_lowering)
    if len(pin_list) <= pin_handled:                                                                #If the next pin hasn't started scanning yet
        while len(pin_list) <= pin_handled: continue                                                #Waiting for the next pin started to scan
    while len(pin_list[pin_handled]) < 6: continue                                                  #Waiting for a full argument list for the next pin
    if pin_list[pin_handled][5] == "ReScan":                                                        #From here works ok, slowly following from wherever pickup, and dropoff
        pin_handled += 1
        continue
    else:
        while pin_list[pin_handled][4] + time_wait_pos > timer_pin.time(): continue
        drive_motor.run_target(feed_speed, wait_pos_y - ((timer_pin.time() - (pin_list[pin_handled][4] + time_wait_pos)) * deg_per_ms), then=Stop.HOLD, wait=True)  #Drive a last time to accurate pin position
        drive_motor.run(y_follow_speed)
        stud_pos(wait_pos_x, pick_pos_z, no_drive_y, clamp_closed, high_speed , False)
        stud_pos(wait_pos_x, wait_pos_z, no_drive_y, clamp_closed, high_speed, False)

        dropoff_pos = pins_scanned[pin_list[pin_handled][5]]["location"]
        if drop_low == True:
            stud_pos(dropoff_pos[0], dropoff_pos[1], dropoff_pos[2], clamp_closed, high_speed, False)
            stud_pos(dropoff_pos[0], drop_pos_z    , no_drive_y    , clamp_open  , high_speed, False)
            stud_pos(dropoff_pos[0], dropoff_pos[1], no_drive_y    , clamp_open  , high_speed, False)
        else:
            stud_pos(dropoff_pos[0], dropoff_pos[1], dropoff_pos[2], clamp_open  , high_speed, False)
        pin_handled += 1


while True:
    stud_pos( 0.6,  6,  1210, 0, 1, False)
    drive_motor.run_target(43, 500)


while True:
    stud_pos( 0.6,   6,  1210, 0, 1, False) 
    #wait(5000)
    stud_pos( 0.6, 7.5,   500, 5, 1, False)  #Good depth at 0.6 and 7.5 and pickup at end off the rail (500)
    #wait(5000)
    stud_pos(   19,   6,  570, 5, 1, False) 
    stud_pos(   19,  23,  570, 0, 1, False) 
    #wait(5000)
    stud_pos(   7,    6, 1210, 0, 1, False) 
    #wait(5000)
    
while False:    
    stud_pos(  18,  12,  400, 5, 1, False)
    stud_pos(   8,   9, 1300, 0, 1, False)
    stud_pos(  15,   1,  300, 0, 1, False)
    stud_pos(  12,  14,  800, 5, 1, False)
    stud_pos(   2,   6,   20, 0, 1, False)


while True:
    stud_pos(   9,   2, 1200, 5, 1, False) 
    stud_pos(  18,  12,  400, 5, 1, False)
    stud_pos(   8,   9, 1300, 0, 1, False)
    stud_pos(  15,   1,  300, 0, 1, False)
    stud_pos(  12,  14,  800, 5, 1, False)
    stud_pos(   2,   6,   20, 0, 1, False)















while True:
    random_one   = choice([1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20])
    random_two   = choice([1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18])
    random_three = 50 * choice([1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25])
    random_four  = choice([0,6])
    stud_pos(random_one, random_two, random_three, random_four, 1, False)


stud_pos( 1, 14, 0, 0, 1, False)
stud_pos( 1, 18.5, 0, 0, 4, False)



while True:
    stud_pos(20, 18.5, 500, 0, 1, False)
    stud_pos( 4, 18.5, 40, 6, 1, False)
    stud_pos( 4, 1, 1250, 6, 1, False)
    stud_pos( 4, 5, 40, 0, 1, False)


while True:
    if pin_ready != None:
        stud_pos( 1, 18.5, 6, 4, False)
        stud_pos( 1, 14, 6, 1, False)
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


