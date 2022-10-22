# SuperFastPython.com
# example of running a function in another thread
from time import sleep
from threading import Thread
 
# a custom function that blocks for a moment
def task():
    # block for a moment
  
    # display a message
    while(1):
        print('This is from another thread')
        sleep(1) 
# create a thread
thread = Thread(target=task)
    # run the thread
thread.start()
while(1):
    
    # wait for the thread to finish
    print('Waiting for the thread...')
    #thread.join()
    sleep(1)
