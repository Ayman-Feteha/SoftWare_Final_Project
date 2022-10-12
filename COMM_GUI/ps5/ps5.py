import pygame
import math
buffer = [0]*11
check = True
pygame.init()
speed_count=0
speed=100

# Set the width and height of the screen [width,height]

#size = [500, 700]
#screen = pygame.display.set_mode(size)
#pygame.display.set_caption("ROV")

# Loop until the user clicks the close button.
done = False

# Initialize the joysticks
pygame.joystick.init()

def bin_dec(button_ls):
    temp=1
    dec=0
    for i in buttons_ls:
        dec=dec+i*temp
        temp*=2
        #print (dec)
    return dec

# -------- Main Program Loop -----------
while not done:
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
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            axis_RX = round(joystick.get_axis(2),3)
            axis_RY = round(joystick.get_axis(5),3)
            axis_LX = round(joystick.get_axis(0),3)
            axis_LY = round(joystick.get_axis(1),3)

            axis_UP = round(((joystick.get_axis(3)+1)/-2) + ((joystick.get_axis(4)+1)/2),3)
            btn_light = joystick.get_button(4)
            btn_speed = joystick.get_button(5)

            btn_gribber1 = joystick.get_button(0)
            btn_gribber2 = joystick.get_button(1)
            btn_gribber3 = joystick.get_button(2)
            btn_gribber4 = joystick.get_button(3)
            buttons_ls=[btn_light,btn_gribber1,btn_gribber2,btn_gribber3,btn_gribber4]
            buttons_dec=bin_dec(buttons_ls)
            ls=[axis_RX,axis_RY,axis_LX,axis_LY,axis_UP]

            if (btn_speed==1):
                speed+=50
                if speed>200:
                    speed=100
                
            for i in range(10): 
                    if abs(ls[i])<=0.05 :
                        ls[i]=0
            if ls != buffer:
                #print(ls)
                for i in range(10): 
                    buffer[i] = ls[i]

            
            



                

        except:
            print("Joy stick not connent")
            





# Close the window and quit.
# If you forget this line, the program will 'hang'
# on exit if running from IDLE.
pygame.quit()


        