#! /usr/bin/python3



display_arr=[0,0,0,0,0,0,0,0,0,0,0]

class motion():                         #This file is responsible for converting axis readings into speed signals

    def motorSpeed(self, axis, max_speed,buttons):          


        self.forward = axis[0]          #asigning axis readings to their functionalities
        self.rotate = axis[1]
        self.slide = axis[2]
        self.pitch = axis[3]
        self.up = axis[4]
        self.max_speed = max_speed      
                                        #calculating the speed of each motor depending on axis readings and max speed provided
        self.FR = (self.forward - self.rotate + self.slide) * self.max_speed
        self.FL = (-self.forward - self.rotate + self.slide) * self.max_speed
        self.TF = (self.pitch - self.up) * self.max_speed
        self.TB = (self.pitch + self.up) * self.max_speed
        self.BR = (-self.forward + self.rotate + self.slide) * self.max_speed
        self.BL = (self.forward + self.rotate + self.slide) * self.max_speed

        self.speeds=[self.FR,self.FL,self.TF,self.TB,self.BR,self.BL]
        
                                        #calculating scale and limiting the speed to be written to thrusters
        x=[abs(max(self.speeds)),abs(min(self.speeds))]
        if(max(x)>self.max_speed):
            scale = self.max_speed / max(x)
            for i in range(len(self.speeds)):
               self.speeds[i]=round(self.speeds[i]*scale)
        else:
            for i in range(len(self.speeds)):
                self.speeds[i]=round(self.speeds[i])
        #print(self.speeds)
        self.output = self.encoder()    #calling function that encodes those speed signals
                                        #adding '#' for checking whether the information is being recieved correctly
        self.output="#" + self.output + str(buttons)+ "#"
        
        return self.output
                                        #This function encodes the speeds to be sent
    def encoder(self):
        self.Chrspeeds=[]
        for i in range(len(self.speeds)):
                                        #shifting by '200' to eliminate negative values then dividing by 2
            self.speeds[i] = (self.speeds[i] + 200)
            self.speeds[i] = round(self.speeds[i] / 2)
                                        #appending each value to an array after converting the number to its corresponding ascii character
            self.Chrspeeds.append(chr(self.speeds[i]))
        

        #print(self.Chrspeeds)
        self.spd = ''.join(self.Chrspeeds)
        
        return self.spd

    def decoder(self,msg):              #the decoder function reverses the operations performed in the encoder part
        global display_arr
        self.speeds2=[x for x in msg]
        #Check if the squence is sent correct 
        #if(len(self.speeds) == '' )
        print(msg)
        if (self.speeds2[0]=="#" and self.speeds2[8]=="#"): #checking whether the message is received correctly
            #del self.speeds2[0]
            #self.speeds2.pop()
            


            speeds_arr=[ord(x) for x in msg[1:7]]
            speeds_arr=[((x*2)-200)for x in speeds_arr[0:6]] #values to be passed to the gui to display speeds of thrusters
            #print(speeds_arr)                               #printing in terminal just to check
            
            states=bin(ord(msg[7])-50)[2:].zfill(5)          #decoding light/gripper states values
            ls=[int(x) for x in states]
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







