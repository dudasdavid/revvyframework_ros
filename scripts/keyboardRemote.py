#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
import threading
import sys
import time

import curses

class killableThread(threading.Thread):
    def __init__(self, *args, **keywords):
        threading.Thread.__init__(self, *args, **keywords)
        self.killed = False

    def start(self):
        self.__run_backup = self.run
        self.run = self.__run
        threading.Thread.start(self)

    def __run(self):
        sys.settrace(self.globaltrace)
        self.__run_backup()
        self.run = self.__run_backup

    def globaltrace(self, frame, why, arg):
        if why == 'call':
            return self.localtrace
        else:
            return None

    def localtrace(self, frame, why, arg):
        if self.killed:
            if why == 'line':
                raise SystemExit()
        return self.localtrace

    def kill(self):
        self.killed = True

strBuffer = "Init"
timeStamp = 0

def keyboardInput():
    global strBuffer
    global timeStamp

    stdscr = curses.initscr()
    curses.cbreak()
    stdscr.keypad(1)

    stdscr.addstr(0, 0, "Hit 'q' to quit")
    stdscr.refresh()

    key = ''
    while key != ord('q'):
        key = stdscr.getch()
        stdscr.addch(20,25,key)
        stdscr.refresh()
        if key == curses.KEY_UP:
            stdscr.addstr(2, 0, "Up   ")
            strBuffer = "Up"
            timeStamp = time.time()
        elif key == curses.KEY_DOWN:
            stdscr.addstr(2, 0, "Down ")
            strBuffer = "Down"
            timeStamp = time.time()
        elif key == curses.KEY_LEFT:
            stdscr.addstr(2, 0, "Left ")
            strBuffer = "Left"
            timeStamp = time.time()
        elif key == curses.KEY_RIGHT:
            stdscr.addstr(2, 0, "Right")
            strBuffer = "Right"
            timeStamp = time.time()

    curses.endwin()

def talker():
    global strBuffer
    global timeStamp

    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        hello_str = strBuffer
        if (time.time()-timeStamp > 0.5):
            strBuffer = "None"
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':

    keyboardProvider = killableThread(target=keyboardInput)
    keyboardProvider.start()

    try:
        talker()
    except rospy.ROSInterruptException:
        pass