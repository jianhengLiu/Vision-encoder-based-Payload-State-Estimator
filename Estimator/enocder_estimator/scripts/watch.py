#!/usr/bin/env python3
import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import rospy
from std_msgs.msg import String
from wtr_can_motor.msg import motor_msgs
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time


points = np.zeros(300)
cnt = 1
plt.ion()
def callback(data):
    global cnt
    global points
    a = data.angle
    print(a)

    if(cnt<300):
        points[cnt] = a
        cnt = cnt + 1
    else:
        points[1:299] = points[2:300]
        points[299] = a
    plt.clf()

    plt.plot(points[0:299])
    plt.pause(0.00001)
    # time.sleep(0.0001)



def listener():
    rospy.init_node('watch', anonymous=True)
    rospy.Subscriber("/wtr/motor/0", motor_msgs, callback,queue_size=1,buff_size=115200)
    rospy.spin()


if __name__ == '__main__':
    listener()
    plt.ioff()


