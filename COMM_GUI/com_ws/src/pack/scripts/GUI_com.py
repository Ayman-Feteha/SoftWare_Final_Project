#! /usr/bin/python3
import threading
from ps5 import *
from motion import *
from GUI_code import *
communication=commu()
app = QApplication(sys.argv)
ex = App()
app.exec_()
def commuThread():
    communication=commu()


def GUIThread():
    
    app = QApplication(sys.argv)
    ex = App()
    app.exec_()

if __name__ =="__main__":
    # creating thread
    t1 = threading.Thread(target=commuThread, args=())
    t2 = threading.Thread(target=GUIThread, args=())
 
    # starting thread 1
    t1.start()
    # starting thread 2
    t2.start()

     # wait until thread 1 is completely executed
    t1.join()
    # wait until thread 2 is completely executed
    t2.join()
    # both threads completely executed
    print("Done!")