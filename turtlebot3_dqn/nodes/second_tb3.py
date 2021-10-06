#!/usr/bin/env python
#coding=utf-8

import rospy

import math
import tf
from geometry_msgs.msg import Twist, Pose , Point

if __name__ == '__main__':
    rospy.init_node('second_tb3')

    listener = tf.TransformListener() #TransformListener创建后就开始接受tf广播信息，最多可以缓存10s


    #Publisher 函数第一个参数是话题名称，第二个参数 数据类型，现在就是我们定义的msg 最后一个是缓冲区的大小
    turtle_vel = rospy.Publisher('tb3_1/cmd_vel', Twist, queue_size=5)

    rate = rospy.Rate(10.0) #循环执行，更新频率是10hz
    while not rospy.is_shutdown():
        try:
            #得到以robot2为坐标原点的robot1的姿态信息(平移和旋转)
            (trans, rot) = listener.lookupTransform('/tb3_1/odom', '/tb3_0/odom', rospy.Time()) #查看相对的tf,返回平移和旋转  turtle2跟着turtle1变换
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        angular = math.atan2(trans[1], trans[0]) #角度变换 计算出前往robot1的角速度 atan2(double y,double x) 返回的是原点至点(x,y)的方位角，即与 x 轴的夹角
        linear = math.sqrt(trans[0] ** 2 + trans[1] ** 2) #平移变换 计算出前往robot1的线速度
        msg = Twist()
        
        if linear>0.008: #如果robot1不动，但是数值有轻微漂移 就不让robot2动
            msg.linear.x = linear #*0.2    #平移变换
            msg.angular.z = angular #*0.1  #角度变换
        else:
            msg.linear.x = 0
            msg.angular.z = 0
        
        turtle_vel.publish(msg) #向/robot2/cmd_vel话题发布新坐标  (即tb3_1根据/tb3_0/cmd_vel的数据,来控制tb3_1移动)
        rate.sleep() #以固定频率执行
