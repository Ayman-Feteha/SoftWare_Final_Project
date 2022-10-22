#! /usr/bin/python3

# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

"""
            *This class is created to take the reading of axis from PS5 controller and convert these reading into speed values that control the motion of the 
            thrusters.
            *these speed values should be ranged from -max_speed to +max_speed
            *the max speed should be controlled from a button in the controller

"""



class motion():

    def motorSpeed(self, axis, max_speed,buttons):


        self.forward = axis[0]      #the axis of pure forward motion, +ve value means moving forward, -ve value means moving backward
        self.rotate = axis[1]       #the axis of pure rotate motion, +values means rotating right, -ve value means rotating left
        self.slide = axis[2]        #the axis of pure sliding motion, +ve value means sliding to right, -ve value means sliding to left
        self.pitch = axis[3]        #the axis of pure pitch motion, +ve value means pitching down, -ve value means pitching up
        self.up = axis[4]           #the axis of pure up motion, +ve value means moving upwards, -ve value means moving downwards
        self.max_speed = max_speed  #the max speed used for speed values calculations to control the sensitivity of the joystick

        """
        the following calculated speeds are based on assumtions for propeller positioning, signs of the equation terms can be changed if the propellers positioning is changed

        """
        self.FR = (self.forward - self.rotate + self.slide) * self.max_speed        #the equation used to calculate the speed on Forward right thruster
        self.FL = (-self.forward - self.rotate + self.slide) * self.max_speed       #the equation used to calculate the speed on Forward left thruster
        self.TF = (self.pitch - self.up) * self.max_speed                           #the equation used to calculate the speed on the Front motor 
        self.TB = (self.pitch + self.up) * self.max_speed                           #the equation used to calculate the speed on the Back motor
        self.BR = (-self.forward + self.rotate + self.slide) * self.max_speed       #the equation used to calculate the speed on the Back right motor
        self.BL = (self.forward + self.rotate + self.slide) * self.max_speed        #the equation used to calculate the speed on the Back left motor

        self.speeds=[self.FR,self.FL,self.TF,self.TB,self.BR,self.BL]   
        

        x=[abs(max(self.speeds)),abs(min(self.speeds))]     #array of maximum and absolute minimum speeds that will be used to scale the speeds if an overlimit speed is given out of equations
        if(max(x)>self.max_speed):                          #checking if the absolute value of the motor with max or min speed (the speeder is taken to the condition by using the previous array) is higher than max speed given by controller.
            scale = self.max_speed / max(x)
            for i in range(len(self.speeds)):               #scaling all speeds of thrusters for safety without changing the direction of the motion
                self.speeds[i]=round(self.speeds[i]*scale)
        else:                                               
            for i in range(len(self.speeds)):
                self.speeds[i]=round(self.speeds[i])
        #print(self.speeds)
        self.output = self.encoder()
        
        self.output="#" + self.output + str(buttons)+ "#"
        
        return self.output
    def encoder(self):
        self.Chrspeeds=[]
        for i in range(len(self.speeds)):
            self.speeds[i] = (self.speeds[i] + 200)
            self.speeds[i] = round(self.speeds[i] / 2)
            self.Chrspeeds.append(chr(self.speeds[i]))
        

        #print(self.Chrspeeds)
        self.spd = ''.join(self.Chrspeeds)
        
        return self.spd

    def decoder(self,msg):
        self.speeds=[x for x in msg]
        #Check if the squence is sent correct 
        #if(len(self.speeds) == '' )
        if (self.speeds[0]=="#" and self.speeds[-1]=="#"):
            del self.speeds[0]
            self.speeds.pop()

            for i in range(len(self.speeds)):

                self.speeds[i] = int(ord(self.speeds[i]))
                self.speeds[i] = self.speeds[i] * 2
                self.speeds[i] = self.speeds[i] - 200


        print(self.speeds[0:6])
        print(bin(ord(self.speeds[7])-50))







