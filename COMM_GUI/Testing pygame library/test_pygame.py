import pygame
import rospy
from std_msgs.msg import String
import sys

def publisher(data):
    rospy.init_node("keyboard input", anonymous=True)
    pub=rospy.Publisher("/topic1",String,queue_size=10)
    rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(data)

while True:
    if __name__=='__main__':
        pygame.init()
        display = pygame.display.set_mode((200, 200))
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        # checking if keydown event happened or not
            if event.type == pygame.KEYDOWN:    
        # checking if key "A" was pressed
                if event.key == pygame.K_a:
                    print("Key A has been pressed")
                    data="A"
                    publisher(data)

                if event.key == pygame.K_b:
                    print("Key B has been pressed")
                    data="B"
                    publisher(data)
    







