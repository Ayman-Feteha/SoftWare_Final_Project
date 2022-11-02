#! /usr/bin/python3

# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

display_arr=[0,0,0,0,0,0,0,0,0,0,0]

class motion():

    def motorSpeed(self, axis, max_speed,buttons):


        self.forward = axis[0]
        self.rotate = axis[1]
        self.slide = axis[2]
        self.pitch = axis[3]
        self.up = axis[4]
        self.max_speed = max_speed

        #self.FR = (self.forward - self.rotate + self.slide) * self.max_speed
        #self.FL = (-self.forward - self.rotate + self.slide) * self.max_speed
        #self.TF = (self.pitch - self.up) * self.max_speed
        #self.TB = (self.pitch + self.up) * self.max_speed
        #self.BR = (-self.forward + self.rotate + self.slide) * self.max_speed
        #self.BL = (self.forward + self.rotate + self.slide) * self.max_speed

        self.BFR= (-self.forward-self.rotate-self.slide-self.pitch+self.up)*max_speed
        self.BFL= (self.forward-self.rotate-self.slide+self.pitch-self.up)*max_speed
        self.BBR= (self.forward+self.rotate-self.slide+self.pitch+self.up)*max_speed
        self.BBL= (-self.forward+self.rotate-self.slide-self.pitch-self.up)*max_speed
        self.TFR= (self.forward+self.rotate+self.slide-self.pitch-self.up)*max_speed
        self.TFL= (-self.forward+self.rotate+self.slide+self.pitch-self.up)*max_speed
        self.TBR= (-self.forward-self.rotate+self.slide+self.pitch+self.up)*max_speed
        self.TBL= (-self.forward+self.rotate-self.slide+self.pitch+self.up)*max_speed

        self.speeds=[self.BFR,self.BFL,self.BBR,self.BBL,self.TFR,self.TFL,self.TBR,self.TBL]
        

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
        global display_arr
        self.speeds2=[x for x in msg]
        #Check if the squence is sent correct 
        #if(len(self.speeds) == '' )
        if (self.speeds2[0]=="#" and self.speeds2[-1]=="#"):
            #del self.speeds2[0]
            #self.speeds2.pop()


            speeds_arr=[ord(x) for x in msg[1:7]]
            speeds_arr=[((x*2)-200)for x in speeds_arr[0:6]] #values to be passed to the gui to display speeds of thrusters
            #print(speeds_arr)                       #printing in terminal just to check

            states=bin(ord(msg[7])-50)[2:].zfill(5)
            ls=[int(x) for x in states[0:6]]
            display_arr=speeds_arr+ls
            print(display_arr)
        else:
            pass
        return(display_arr) 
        


        #This did not work so we switched to the two lines upove
            #for i in range(len(self.speeds)):

            #    self.speeds2[i] = int(ord(self.speeds2[i]))
            #    self.speeds2[i] = self.speeds2[i] * 2
            #    self.speeds2[i] = self.speeds2[i] - 200


        
        #print(bin(ord(self.speeds2[7])-50))







