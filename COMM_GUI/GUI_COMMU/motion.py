#! /usr/bin/python3

# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.



class motion():

    def motorSpeed(self, axis, max_speed,buttons):


        self.forward = axis[0]
        self.rotate = axis[1]
        self.slide = axis[2]
        self.pitch = axis[3]
        self.up = axis[4]
        self.max_speed = max_speed

        self.FR = (self.forward - self.rotate + self.slide) * self.max_speed
        self.FL = (-self.forward - self.rotate + self.slide) * self.max_speed
        self.TF = (self.pitch - self.up) * self.max_speed
        self.TB = (self.pitch + self.up) * self.max_speed
        self.BR = (-self.forward + self.rotate + self.slide) * self.max_speed
        self.BL = (self.forward + self.rotate + self.slide) * self.max_speed

        self.speeds=[self.FR,self.FL,self.TF,self.TB,self.BR,self.BL]
        

        x=[abs(max(self.speeds)),abs(min(self.speeds))]
        if(max(x)>self.max_speed):
            scale = self.max_speed / max(x)
            for i in range(len(self.speeds)):
               self.speeds[i]=round(self.speeds[i]*scale)
        else:
            for i in range(len(self.speeds)):
                self.speeds[i]=round(self.speeds[i])
        #print(self.speeds)
        self.output = self.encoder()
        
        self.output="#" + self.output + str(buttons)+ "#"
        
        #return self.output
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







