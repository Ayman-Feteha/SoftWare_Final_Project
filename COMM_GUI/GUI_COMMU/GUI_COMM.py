#! /usr/bin/python3
import pygame
from motion import *
import sys
import rospy
from std_msgs.msg import String
from time import sleep

from threading import Thread
from GUI_code import * 

                            #creating object from imported motion class
ROV=motion()
                            #all commented lines of code are for testing purposes
buffer = [0,0,0,0,0]        #buffer to store old values and compare them to new readings
buttons_last = [0,0,0,0,0]     #buffer to store old button readings
btn_states=[0,0,0,0,0]

speed=100                   #maximum speed initial value

done = False
last_recieved_msg=""

def ROVcb(data):            #Callback function for msg recieved from ROV
    global last_recieved_msg
    if last_recieved_msg!=data.data:
        ROV.decoder(data.data)
        last_recieved_msg=data.data
    #rospy.loginfo(ROVmsg)
    
class Publisher():          #This class initialises the station publisher node
    def __init__(self):
        self.name="station"
        rospy.init_node(self.name)
        self.pub=rospy.Publisher("/toROV",String,queue_size=10)

    def send(self,msg):     #function in class to send the required signal to be sent "String Value"
        self.pub.publish(msg)
        #rospy.loginfo(msg)
        #rospy.Rate(2)

    def recieve(self):      #function responsible for recieving data from ROV
        try:
            rospy.Subscriber("/fromROV",String,ROVcb)
        except TypeError:
            pass

def bin_dec(button_ls):     #this function takes button inputs in a list and return the equivalent decimal value to be encoded
    temp=1
    dec=0
    for i in button_ls:
        dec=dec+i*temp
        temp*=2
    dec+=50
        #print (dec)
    return chr(dec)

station = Publisher()

def states(ls,speed = 0,button_ls =[0,0,0,0,0]):

    global buttons_last
    global btn_states
    
                            #limiting some readings to fix errors caused by joystick
    for i in range(5): 
        if abs(ls[i])<0.051 :
            ls[i]=0
                            #making sure of change in readings before sending signals
    if (ls != buffer) or (button_ls != buttons_last) :
        #print(ls)
        for i in range(5): 
            buffer[i] = ls[i]
        for i in range (5):
            if (button_ls[i]==1):
                btn_states[i]=not btn_states[i]
                buttons_last[i]=button_ls[i]
        buttons_dec=bin_dec(btn_states)
        
        #print(buffer)
                            #passing readings to motion class to get encoded msg of speed values and buttons
        msg = ROV.motorSpeed(ls,speed,buttons_dec)
                            #sending msg and recieving response from the ROV arduino
        station.send(msg)
        station.recieve()
        #print (msg)    
        #ROV.decoder()

def main():
    global done
    pygame.init()
    clock=pygame.time.Clock()
    pygame.joystick.init()      #initialising joystick

    while (not done)  and (not rospy.is_shutdown()):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
                                #for testing purposes
            if event.type == pygame.JOYBUTTONDOWN:     
                print("Joystick button pressed.")
            if event.type == pygame.JOYBUTTONUP:
                print("Joystick button released.")
            
            try:                #try/except to detect if anything happens in joystick bluetooth connection with pc
                joystick = pygame.joystick.Joystick(0)
                joystick.init() #getting axis readings
                axis_RX = round(joystick.get_axis(3),3)
                axis_RY = round(joystick.get_axis(4),3)
                axis_LX = round(joystick.get_axis(0),3)
                axis_LY = round(joystick.get_axis(1),3)
                                #merging two axis together
                axis_Forward = round(((joystick.get_axis(5)+1)/-2) + ((joystick.get_axis(2)+1)/2),3)
                btn_light = joystick.get_button(4)
                btn_speed = joystick.get_button(5)
                                #getting buttons readings
                btn_gribber1 = joystick.get_button(0)
                btn_gribber2 = joystick.get_button(1)
                btn_gribber3 = joystick.get_button(2)
                btn_gribber4 = joystick.get_button(3)
                buttons_ls=[btn_light,btn_gribber1,btn_gribber2,btn_gribber3,btn_gribber4]
                
                                #passing button readings list to function to get decimal value
                #buttons_dec=bin_dec(buttons_ls)
                                #toggling maximum speed depending on speed button reading
                global speed
                if (btn_speed==1):
                    speed+=50
                    if speed>200:
                        speed=100
                                #list containing axis readings
                ls=[axis_Forward,axis_RX,axis_LX,-axis_RY,-axis_LY]
                                #passing axis and buttons readings to states function
                states(ls,speed,buttons_ls)

                                #detecting errors in joystick connection         
            except pygame.error:
                                #hinting of the error to user
                print("Joystick is not connected")
                z = [0,0,0,0,0]
                                #sending signal to states to stop ROV completely
                states(z)
        sleep(0.0001)
                
    pygame.quit()               # line to close the window and quit.
def GUI():
    app = QApplication(sys.argv)
    ex = App()
    app.exec_()
    
if __name__=='__main__':
                    
    thread = Thread(target =main)
    thread.start()
    GUI()