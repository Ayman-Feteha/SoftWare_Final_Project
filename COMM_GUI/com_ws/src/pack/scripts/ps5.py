#! /usr/bin/python3
from os import stat
import pygame
from motion import *
import sys
import rospy
from std_msgs.msg import String
from sys import getsizeof
import math
buffer = [0,0,0,0,0]
buttons_last = "d"

pygame.init()
speed=100

done = False
check=True

# Initialize the joysticks

class Publisher():
    def __init__(self):
        self.name="station"
        rospy.init_node(self.name)
        self.pub=rospy.Publisher("/toROV",String,queue_size=10)

    def send(self,msg):
        self.pub.publish(msg)
        rospy.loginfo(msg)
        #rospy.Rate(2)

station = Publisher()

pygame.init()
clock=pygame.time.Clock()
pygame.joystick.init()

def bin_dec(button_ls):
    temp=1
    dec=0
    for i in buttons_ls:
        dec=dec+i*temp
        temp*=2
        #print (dec)
    return str(dec)

def states(ls,speed = 0,buttons_dec ="d"):
    ROV=motion()
    global buttons_last
    for i in range(5): 
        if abs(ls[i])<0.051 :
            ls[i]=0
    if (ls != buffer) or (buttons_dec != buttons_last) :
        #print(ls)
        for i in range(5): 
            buffer[i] = ls[i]
        buttons_last = buttons_dec
        #print(buffer)
        msg = ROV.motorSpeed(ls,speed,buttons_dec)
        station.send(msg)
        #print (msg)    
        #ROV.decoder()


# -------- Main Program Loop -----------
while (not done)  and (not rospy.is_shutdown()):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
        # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN
        # JOYBUTTONUP JOYHATMOTION
        #_____________________only for test __________#
        if event.type == pygame.JOYBUTTONDOWN:     
            print("Joystick button pressed.")
        if event.type == pygame.JOYBUTTONUP:
            print("Joystick button released.")
        #_____________________________________________#
        try:
            check=True
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            axis_RX = round(joystick.get_axis(3),3)
            axis_RY = round(joystick.get_axis(4),3)
            axis_LX = round(joystick.get_axis(0),3)
            axis_LY = round(joystick.get_axis(1),3)

            axis_Forward = round(((joystick.get_axis(5)+1)/-2) + ((joystick.get_axis(2)+1)/2),3)
            btn_light = joystick.get_button(4)
            btn_speed = joystick.get_button(5)

            btn_gribber1 = joystick.get_button(0)
            btn_gribber2 = joystick.get_button(1)
            btn_gribber3 = joystick.get_button(2)
            btn_gribber4 = joystick.get_button(3)
            buttons_ls=[btn_light,btn_gribber1,btn_gribber2,btn_gribber3,btn_gribber4]
            buttons_dec=bin_dec(buttons_ls)
            if (btn_speed==1):
                speed+=50
                if speed>200:
                    speed=100
            ls=[axis_Forward,axis_RX,axis_LX,-axis_RY,-axis_LY]
            states(ls,speed,buttons_dec)
                     
        except pygame.error:
            print("Joystick is not connected")
            z = [0,0,0,0,0]
            states(z)
            
            

            
            





# Close the window and quit.
# If you forget this line, the program will 'hang'
# on exit if running from IDLE.
pygame.quit()


        