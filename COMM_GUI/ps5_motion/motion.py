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

        self.speeds.append(buttons)
        self.output = self.encoder()
        return self.output
    def encoder(self):
        
        for i in range(len(self.speeds)):
            self.speeds[i] = round(self.speeds[i] + 200)
            self.speeds[i] = round(self.speeds[i] / 2)
            self.speeds[i] = chr(self.speeds[i])

        self.spd = ''.join(self.speeds)
        return self.spd

    def decoder(self):
        self.speeds=list(self.spd)
        for i in range(len(self.speeds)):
            self.speeds[i] = ord(self.speeds[i])
            self.speeds[i] = round(self.speeds[i] * 2)
            self.speeds[i] = round(self.speeds[i] - 200)


        print(self.speeds)







