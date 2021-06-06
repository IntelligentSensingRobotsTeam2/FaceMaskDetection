# -*- coding: utf-8 -*-
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist


import time 
import os,threading
import socket,signal
import fcntl, struct
from std_msgs.msg import String

DEBUG = True   ## print debug information 

PI = 3.1415926535897

def rotate(angle):
    #Starts a new node
    #rospy.init_node('robot_cleaner', anonymous=True)
    # velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    # Receiveing the user's input
    # print("Let's rotate your robot")
    # speed = input("Input your speed (degrees/sec):")
    # angle = input("Type your distance (degrees):")
    # clockwise = input("Clockwise?: ") #True or false
    speed = 50
    clockwise = (angle > 90 and angle <270)
    #Converting from angles to radians
    angular_speed = speed*2*PI/360
    relative_angle = angle*2*PI/360

    #We wont use linear components
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < relative_angle):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)
        # print('cur:',current_angle,'goal:',relative_angle)
    print('\nreach!: time taken:',(t1-t0))

    #Forcing our robot to stop
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)






def forward(dis):    
        # 定义在/cmd_vel Topic中发布Twist消息，控制机器人速度
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10) 
        rate = 50 
        # 设置更新频率为50HZ
        r = rospy.Rate(rate) 
        # 线速度
        linear_speed = 0.2 
        # 目标距离
        goal_distance = dis
        # 到达目标的时间
        linear_duration = goal_distance / linear_speed
         
        move_cmd = Twist()
            # Set the forward speed
        move_cmd.linear.x = linear_speed
            # 机器人向前运动，延时一定时间
        ticks = int(linear_duration * rate)
        for t in range(ticks):
            self.cmd_vel.publish(move_cmd)
            r.sleep()

        self.cmd_vel.publish(Twist())






def pump_start():
      pub = rospy.Publisher('pump_start', String, queue_size=10)
      hello_str = "1" 
      rospy.loginfo(hello_str)
      pub.publish(hello_str) 

def pump_close():
      pub = rospy.Publisher('pump_start', String, queue_size=10)
      hello_str = "0" 
      rospy.loginfo(hello_str)
      pub.publish(hello_str) 


def walk_start():
      pub = rospy.Publisher('robot_walk', String, queue_size=10)
      hello_str = "1" 
      rospy.loginfo(hello_str)
      pub.publish(hello_str) 

def walk_close():
      pub = rospy.Publisher('robot_walk', String, queue_size=10)
      hello_str = "0" 
      rospy.loginfo(hello_str)
      pub.publish(hello_str) 



def server_exit(signum, frame):
    print('\nServer Stopped by keyboard interupt.')
    exit()


## get IP of current PC. (python3 Linux OS)
def get_ip(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(s.fileno(), 0x8915, struct.pack('256s', bytes(ifname[:15],'utf-8')))[20:24])

def start_server():
    # ip = get_ip('ens33')
    ip = '10.17.98.14'
    if DEBUG:
        print('current ip:{}'.format(ip))
    PORT = 7000
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    address = (ip, PORT)  
    server_socket.bind(address)  

    while True:

        now = time.time() 
                        # 默认是阻塞的
        receive_data, client = server_socket.recvfrom(1024)
        decode_data = receive_data.decode('utf-8')
        if DEBUG:
            print(time.strftime('%Y-%m-%d %H:%M:%S',time.localtime(now)))
            
            print("recv from %s, data: %s\n" % (client, decode_data))
        
        
        cmd = decode_data.split(':')[0]
        value = int(decode_data.split(':')[1])
        if cmd == 'spray':
            if value == 1:
                pump_start()
                walk_start()
            elif value == 0:
                pump_close()
                walk_close()
        elif cmd == 'walk':
            if value == 1:
                walk_start()
            elif value == 0:
                walk_close()
        elif cmd == 'rotate':   
        #try:
        # Testing our functionS
            rotate(value)
        #except rospy.ROSInterruptException:
         #   pass

        elif cmd == 'forward': ## go straight
            forward(value)


if __name__ == '__main__':
    print('UDP server started. ctrl-c to stop.')
    signal.signal(signal.SIGINT, server_exit)
    signal.signal(signal.SIGTERM, server_exit)
    DEBUG = True
    rospy.init_node('robot_cleaner', anonymous=True)
    start_server()
