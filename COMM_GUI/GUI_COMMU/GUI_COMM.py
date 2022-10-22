#! /usr/bin/python3
import pygame
from motion import *
import sys
import rospy
from std_msgs.msg import String
from time import sleep

from threading import Thread
from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QMainWindow ,QLCDNumber,QSizePolicy
from PyQt5.QtGui import QPainter, QColor, QPen, QFont, QBrush
from PyQt5.QtCore import Qt, QTimer

import sys
import random
import matplotlib
matplotlib.use('Qt5Agg')

from PyQt5 import QtCore, QtWidgets

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib import pyplot as plt

from motion import *
msg=[0,0,0,0,0,0]

#check gripper state
GRIPPER1_STATE = 0
GRIPPER2_STATE = 0
GRIPPER3_STATE = 0
GRIPPER4_STATE = 1

#check lightening state
LIGHT_STATE  = "ON"
#check if ps5 connected or not 
PS5_STATE = "ON"

#thrust speeds
thrust_FR = 10
thrust_FL = 200
thrust_TF = 200
thrust_TB = 200
thrust_BR = 150
thrust_BL = 150
maxspeed =200

#imu reading 
pitch = 30
yaw = 20
roll = 50



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
    global msg
    global last_recieved_msg
    if last_recieved_msg!=data.data:
        msg=ROV.decoder(data.data)
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
    class MplCanvas(FigureCanvas):

        def __init__(self, parent=None, width=5, height=4, dpi=100):
            fig = Figure(figsize=(width, height), dpi=dpi)
            self.axes = fig.add_subplot(111)
            FigureCanvas.__init__(self, fig)
            self.setParent(parent)
            FigureCanvas.updateGeometry(self)

    class App(QMainWindow):

        def __init__(self):
            super(App, self).__init__()
            
            self.title = 'ROV'
            self.left = 200
            self.top = 200
            self.width = 1600
            self.height = 900

            # Setup a timer to trigger the redraw by calling update_plot.
            self.timer = QtCore.QTimer()
            self.timer.setInterval(100)
            self.timer.timeout.connect(self.updateEvent)
            self.timer.timeout.connect(self.update_plot)
            self.timer.start()

            self.initUI()
        
        def initUI(self):
            self.setWindowTitle(self.title)
            self.setGeometry(self.left, self.top, self.width, self.height)
            
            #self.setAutoFillBackground(True)
            self.setStyleSheet("background:rgb(20, 20, 20)")  

    #----------display Max speed------------------
            
            self.labelMAXspeed = QLabel(self)
            self.labelMAXspeed.setText("MAX SPEED : ")
            self.labelMAXspeed.setGeometry(300 , 250, 260, 50)
            self.labelMAXspeed.setFont(QFont("Arial", 30))
            self.labelMAXspeed.setStyleSheet("QLabel {color : cyan}")

            self.labelMAXspeed = QLabel(self)
            self.labelMAXspeed.setText(str(maxspeed))
            self.labelMAXspeed.setGeometry(570 , 250, 260, 50)
            self.labelMAXspeed.setFont(QFont("Arial", 30))
            self.labelMAXspeed.setStyleSheet("QLabel {color : cyan}")
    # ----------display thrust speeds-------------

            #THRUST1
            self.Thrust1 = QLCDNumber(self)
            self.Thrust1.display(thrust_FR)
            self.Thrust1.setGeometry(60 , 85, 80, 50)

            self.labelThrustName1 = QLabel(self)
            self.labelThrustName1.setText("Thrust FR")
            self.labelThrustName1.setGeometry(40, 195, 120, 50)
            self.labelThrustName1.setFont(QFont("Arial", 20))
            self.labelThrustName1.setStyleSheet("QLabel {color : cyan}")

            #THRUST2
            self.Thrust2 = QLCDNumber(self)
            self.Thrust2.display(thrust_FL)
            self.Thrust2.setGeometry(280, 85, 80, 50)

            self.labelThrustName2 = QLabel(self)
            self.labelThrustName2.setText("Thrust FL")
            self.labelThrustName2.setGeometry(270, 195, 120, 50)
            self.labelThrustName2.setFont(QFont("Arial", 20))
            self.labelThrustName2.setStyleSheet("QLabel {color : cyan}")

            #THRUST3
            self.Thrust3 = QLCDNumber(self)
            self.Thrust3.display(thrust_TF)
            self.Thrust3.setGeometry(500, 85, 80, 50)
            
            self.labelThrustName3 = QLabel(self)
            self.labelThrustName3.setText("Thrust TF")
            self.labelThrustName3.setGeometry(490, 195, 120, 50)
            self.labelThrustName3.setFont(QFont("Arial", 20))
            self.labelThrustName3.setStyleSheet("QLabel {color : cyan}")

            #THRUST4
            self.Thrust4 = QLCDNumber(self)
            self.Thrust4.display(thrust_TB)
            self.Thrust4.setGeometry(720, 85, 80, 50)
            

            self.labelThrustName4 = QLabel(self)
            self.labelThrustName4.setText("Thrust TB")
            self.labelThrustName4.setGeometry(700, 195, 120, 50)
            self.labelThrustName4.setFont(QFont("Arial", 20))
            self.labelThrustName4.setStyleSheet("QLabel {color : cyan}")

            #THRUST5
            self.Thrust5 = QLCDNumber(self)
            self.Thrust5.display(thrust_BR)
            self.Thrust5.setGeometry(940, 85, 80, 50)


            self.labelThrustName5 = QLabel(self)
            self.labelThrustName5.setText("Thrust BR")
            self.labelThrustName5.setGeometry(920, 195, 120, 50)
            self.labelThrustName5.setFont(QFont("Arial", 20))
            self.labelThrustName5.setStyleSheet("QLabel {color : cyan}")

            #THRUST6
            self.Thrust6 = QLCDNumber(self)
            self.Thrust6.display(thrust_BL)
            self.Thrust6.setGeometry(1160, 85, 80, 50)


            self.labelThrustName6 = QLabel(self)
            self.labelThrustName6.setText("Thrust BL")
            self.labelThrustName6.setGeometry(1150, 195, 120, 50)
            self.labelThrustName6.setFont(QFont("Arial", 20))
            self.labelThrustName6.setStyleSheet("QLabel {color : cyan}")

    #-----------------state of controlller-------------

            self.labelPS5 = QLabel(self)
            self.labelPS5.setText("Controller state")
            self.labelPS5.setGeometry(1370,30,190,50)
            self.labelPS5.setFont(QFont("Arial", 20))
            self.labelPS5.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")

    #----------------------ON label------------------------
            self.labelPS5ON = QLabel(self)
            self.labelPS5ON.setText(PS5_STATE)
            self.labelPS5ON.setGeometry(1400,210,120,50)
            self.labelPS5ON.setFont(QFont("Arial", 40))
            self.labelPS5ON.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")

    #------------------IMU READING------------------

            self.labelimu = QLabel(self)
            self.labelimu.setText("IMU reading")
            self.labelimu.setGeometry(30,340,190,50)
            self.labelimu.setFont(QFont("Arial", 25))
            self.labelimu.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")

            self.labelpitchvalue = QLabel(self)
            self.labelpitchvalue.setText(str(pitch))
            self.labelpitchvalue.setGeometry(130,400,100,50)
            self.labelpitchvalue.setFont(QFont("Arial", 20))
            self.labelpitchvalue.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")

            self.labelpitch = QLabel(self)
            self.labelpitch.setText("PITCH : ")
            self.labelpitch.setGeometry(30,400,100,50)
            self.labelpitch.setFont(QFont("Arial", 20))
            self.labelpitch.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")

            self.labelyawvalue = QLabel(self)
            self.labelyawvalue.setText(str(yaw))
            self.labelyawvalue.setGeometry(130,450,100,50)
            self.labelyawvalue.setFont(QFont("Arial", 20))
            self.labelyawvalue.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")

            self.labelyaw = QLabel(self)
            self.labelyaw.setText("YAW : ")
            self.labelyaw.setGeometry(30,450,100,50)
            self.labelyaw.setFont(QFont("Arial", 20))
            self.labelyaw.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")

            self.labelrollvalue = QLabel(self)
            self.labelrollvalue.setText(str(roll))
            self.labelrollvalue.setGeometry(130,500,100,50)
            self.labelrollvalue.setFont(QFont("Arial", 20))
            self.labelrollvalue.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")

            self.labelroll = QLabel(self)
            self.labelroll.setText("ROLL : ")
            self.labelroll.setGeometry(30,500,100,50)
            self.labelroll.setFont(QFont("Arial", 20))
            self.labelroll.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")

    #--------------------display lightening system-----------------------
            self.labellight = QLabel(self)
            self.labellight.setText("Lightening State")
            self.labellight.setGeometry(320,340,250,50)
            self.labellight.setFont(QFont("Arial", 25))
            self.labellight.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")

    #----------------------state label------------------------
            self.labellight_state = QLabel(self)
            self.labellight_state.setText(LIGHT_STATE)
            self.labellight_state.setGeometry(380,520,120,50)
            self.labellight_state.setFont(QFont("Arial", 40))
            self.labellight_state.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")
    #----------------------GRIPPERS------------------------
        
    #--------------------display Gripper_1-----------------------
            self.labelgripper1 = QLabel(self)
            self.labelgripper1.setText("Gripper[1]")
            self.labelgripper1.setGeometry(30,650,190,50)
            self.labelgripper1.setFont(QFont("Arial", 25))
            self.labelgripper1.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")

    #--------------------display Gripper_2-----------------------
            self.labelgripper2 = QLabel(self)
            self.labelgripper2.setText("Gripper[2]")
            self.labelgripper2.setGeometry(30,700,190,50)
            self.labelgripper2.setFont(QFont("Arial", 25))
            self.labelgripper2.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")

    #--------------------display Gripper_3-----------------------
            self.labelgripper3 = QLabel(self)
            self.labelgripper3.setText("Gripper[3]")
            self.labelgripper3.setGeometry(30,750,190,50)
            self.labelgripper3.setFont(QFont("Arial", 25))
            self.labelgripper3.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")
    #--------------------display Gripper_4-----------------------
            self.labelgripper4 = QLabel(self)
            self.labelgripper4.setText("Gripper[4]")
            self.labelgripper4.setGeometry(30,800,190,50)
            self.labelgripper4.setFont(QFont("Arial", 25))
            self.labelgripper4.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")

    #----------------------graph------------------------
            self.graphlbl = QLabel(self)
            self.graphlbl.setText("IMU Reading")
            self.graphlbl.setGeometry(1000,340,980,50)
            self.graphlbl.setFont(QFont("Arial", 25))
            self.graphlbl.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")
            
            self.m = MplCanvas(self, width=9.8, height=4.8, dpi=100)
            self.m.move(600,400)
            
            n_data = 50
            self.xdata = list(range(n_data))
            self.ydata = [random.randint(0, 10) for i in range(n_data)]
            self.ydata1 = [random.randint(0, 10) for i in range(n_data)]


            self.show()
        
        def updateEvent(self):
            global thrust_FR,thrust_FL,thrust_TF,thrust_TB,thrust_BR,thrust_BL,GRIPPER1_STATE,GRIPPER2_STATE,msg
            thrust_FR = msg[0]
            self.Thrust1.display(thrust_FR)

            thrust_FL = 30
            self.Thrust2.display(thrust_FL)

            thrust_TF = 150
            self.Thrust3.display(thrust_TF)

            thrust_TB = 170
            self.Thrust4.display(thrust_TB)

            thrust_BR = 40
            self.Thrust5.display(thrust_BR)

            thrust_BL = 10
            self.Thrust6.display(thrust_BL)

            GRIPPER1_STATE = 1
            GRIPPER2_STATE = 1


            self.update()
        
        def update_plot(self):
            # Drop off the first y element, append a new one.
            
            self.ydata = self.ydata[1:] + [random.randint(0, 10)]
            self.ydata1 = self.ydata1[1:] + [random.randint(0, 10)]

            self.m.axes.cla()  # Clear the m.
            self.m.axes.plot(self.xdata, self.ydata, 'r')
            self.m.axes.plot(self.xdata, self.ydata1, 'b')

            # Trigger the m to update and redraw.
            self.m.draw()

        def paintEvent(self, event):


            painter = QPainter()
            painter.begin(self)
            painter.setRenderHint(QPainter.Antialiasing)
            
            painter.setPen(QPen(QColor(150, 150, 150), 2))
            painter.drawRect(10 , 10 , 1300, 310)
    #-------------------------thrust--------------------------------
            painter.setPen(QPen(QColor(45, 50, 60), 20))      
            painter.drawArc(30, 30, 150, 150, -90 * 16, 360 * 16)
            painter.drawArc(250, 30, 150, 150, -90 * 16, 360 * 16)
            painter.drawArc(470, 30, 150, 150, -90 * 16, 360 * 16)
            painter.drawArc(690, 30, 150, 150, -90 * 16, 360 * 16)
            painter.drawArc(910, 30, 150, 150, -90 * 16, 360 * 16)
            painter.drawArc(1130, 30, 150, 150, -90 * 16, 360 * 16)


            painter.setPen(QPen(Qt.cyan, 20))            
            painter.drawArc(30, 30, 150, 150, -90 * 16, -thrust_FR /maxspeed *360* 16)
            painter.drawArc(250, 30, 150, 150, -90 * 16, -thrust_FL /maxspeed *360* 16)
            painter.drawArc(470, 30, 150, 150, -90 * 16, -thrust_TF /maxspeed *360* 16)
            painter.drawArc(690, 30, 150, 150, -90 * 16, -thrust_TB /maxspeed *360* 16)
            painter.drawArc(910, 30, 150, 150, -90 * 16, -thrust_BR /maxspeed *360* 16)
            painter.drawArc(1130, 30, 150, 150, -90 * 16, -thrust_BL /maxspeed *360* 16)

    #-----------------------------IMU---------------

            painter.setPen(QPen(QColor(150, 150, 150), 2))
            painter.drawRect(10 , 330 , 260, 260) 

    #----------------------controller state-----------------------
            painter.setPen(QPen(QColor(150, 150, 150), 2))
            painter.drawRect(1330 , 10, 250, 310)      
            if (PS5_STATE =="ON"):
                painter.setBrush(QBrush(QColor(57, 255, 20), Qt.SolidPattern))
            else:
                painter.setBrush(QBrush(QColor(150, 150, 150), Qt.SolidPattern))
            painter.drawEllipse(1400, 90, 100, 100)


    #-------------------lightening system------------------------------  
            painter.setPen(QPen(QColor(150, 150, 150), 2))
            painter.setBrush(QBrush(Qt.transparent, Qt.SolidPattern))
            painter.drawRect(300 , 330, 280, 260)  
            if (LIGHT_STATE =="ON"):
                painter.setBrush(QBrush(QColor(57, 255, 20), Qt.SolidPattern))
            else:
                painter.setBrush(QBrush(QColor(150, 150, 150), Qt.SolidPattern))
            painter.drawEllipse(370, 400, 100, 100)

    #-------------------Grippers----------------------------------------
            painter.setPen(QPen(QColor(150, 150, 150), 2))
            painter.setBrush(QBrush(Qt.transparent, Qt.SolidPattern))
            painter.drawRect(10 , 620, 570, 260) 

            if(GRIPPER1_STATE == 1):
                painter.setBrush(QBrush(QColor(57, 255, 20), Qt.SolidPattern))
            else:
                painter.setBrush(QBrush(QColor(150, 150, 150), Qt.SolidPattern))
            painter.drawEllipse(230, 660, 35, 35)

            if(GRIPPER2_STATE == 1):
                painter.setBrush(QBrush(QColor(57, 255, 20), Qt.SolidPattern))
            else:
                painter.setBrush(QBrush(QColor(150, 150, 150), Qt.SolidPattern))
            painter.drawEllipse(230, 710, 35, 35)

            if(GRIPPER3_STATE == 1):
                painter.setBrush(QBrush(QColor(57, 255, 20), Qt.SolidPattern))
            else:
                painter.setBrush(QBrush(QColor(150, 150, 150), Qt.SolidPattern))
            painter.drawEllipse(230, 760, 35, 35)

            if(GRIPPER4_STATE == 1):
                painter.setBrush(QBrush(QColor(57, 255, 20), Qt.SolidPattern))
            else:
                painter.setBrush(QBrush(QColor(150, 150, 150), Qt.SolidPattern))
            painter.drawEllipse(230, 810, 35, 35)

    #----------------------Graph----------------------------------------
            painter.setPen(QPen(QColor(150, 150, 150), 2))
            painter.setBrush(QBrush(Qt.transparent, Qt.SolidPattern))
            painter.drawRect(600, 330, 980, 550)
    app = QApplication(sys.argv)
    ex = App()
    app.exec_()
    
if __name__=='__main__':
                    
    thread = Thread(target =main)
    thread.start()
    GUI()