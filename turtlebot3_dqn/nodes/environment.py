#!/usr/bin/env python

import rospy
import numpy as np
import math
from math import pi
import time
import Tkinter as tk
from PIL import ImageTk, Image
from geometry_msgs.msg import Twist, Point, Pose
from std_srvs.srv import Empty


np.random.seed(1)
PhotoImage = ImageTk.PhotoImage
UNIT = 50  # pixels
HEIGHT = 11  # grid height
WIDTH = 11  # grid width


class Env(tk.Tk):
    def __init__(self):
        tk.Tk.__init__(self)
        self.action_space = ['u', 'd', 'l', 'r']
        self.n_actions = len(self.action_space)
        self.title('Q Learning')
        self.geometry('{0}x{1}'.format(HEIGHT * UNIT, HEIGHT * UNIT))
        self.shapes = self.load_images()
        self.canvas = self._build_canvas()
        self.texts = []
	self.act = 0
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)



    def _build_canvas(self):
        canvas = tk.Canvas(self, bg='white',
                           height=HEIGHT * UNIT,
                           width=WIDTH * UNIT)
        # create grids
        for c in range(0, WIDTH * UNIT, UNIT):  # 0~400 by 80
            x0, y0, x1, y1 = c, 0, c, HEIGHT * UNIT
            canvas.create_line(x0, y0, x1, y1)
        for r in range(0, HEIGHT * UNIT, UNIT):  # 0~400 by 80
            x0, y0, x1, y1 = 0, r, HEIGHT * UNIT, r
            canvas.create_line(x0, y0, x1, y1)

        # add img to canvas
        self.rectangle = canvas.create_image(275, 275, image=self.shapes[0])
        self.triangle1 = canvas.create_image(175, 175, image=self.shapes[1])
        self.triangle2 = canvas.create_image(175, 375, image=self.shapes[1])
        self.triangle3 = canvas.create_image(375, 175, image=self.shapes[1])
        self.triangle4 = canvas.create_image(375, 375, image=self.shapes[1])
        self.circle = canvas.create_image(475, 75, image=self.shapes[2])

        # pack all
        canvas.pack()

        return canvas

    def load_images(self):
        rectangle = PhotoImage(
            Image.open("/home/wangqiang/catkin_ws/src/turtlebot3_machine_learning/turtlebot3_dqn/img/rectangle.png").resize((35, 35)))
        triangle = PhotoImage(
            Image.open("/home/wangqiang/catkin_ws/src/turtlebot3_machine_learning/turtlebot3_dqn/img/triangle.png").resize((35, 35)))
        circle = PhotoImage(
            Image.open("/home/wangqiang/catkin_ws/src/turtlebot3_machine_learning/turtlebot3_dqn/img/circle.png").resize((35, 35)))

        return rectangle, triangle, circle

    def text_value(self, row, col, contents, action, font='Helvetica', size=7,
                   style='normal', anchor="nw"):

        if action == 0:
            origin_x, origin_y = 3, 21
        elif action == 1:
            origin_x, origin_y = 37, 21
        elif action == 2:
            origin_x, origin_y = 21, 3
        else:
            origin_x, origin_y = 21, 33

        x = origin_y + (UNIT * col)
        y = origin_x + (UNIT * row)
        font = (font, str(size), style)
        text = self.canvas.create_text(x, y, fill="black", text=contents,
                                       font=font, anchor=anchor)
        return self.texts.append(text)

    def print_value_all(self, q_table):
        for i in self.texts:
            self.canvas.delete(i)
        del self.texts[::]
        for i in range(HEIGHT):
            for j in range(WIDTH):
                for action in range(0, 4):
                    state = [i, j]
                    if str(state) in q_table.keys():
                        temp = q_table[str(state)][action]
                        self.text_value(j, i, round(temp, 2), action)

    def coords_to_state(self, coords):
        x = int((coords[0] - 25) / 50)
        y = int((coords[1] - 25) / 50)
        return [x, y]

    def state_to_coords(self, state):
        x = int(state[0] * 50 + 25)
        y = int(state[1] * 50 + 25)
        return [x, y]

    def reset(self):
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")
        self.update()
        time.sleep(0.5)
        x, y = self.canvas.coords(self.rectangle)
        self.canvas.move(self.rectangle, UNIT / 2 - x + 250, UNIT / 2 - y + 250)
        self.render()
        # return observation
        return self.coords_to_state(self.canvas.coords(self.rectangle))


    def step(self, action):
        state = self.canvas.coords(self.rectangle)
        base_action = np.array([0, 0])
        self.render()
	
        
	vel_cmd = Twist()


        if action == 0:  # up
            if state[1] > UNIT:
		if self.act == 0: #zhi zou
	            vel_cmd.linear.x = 0.2
                    self.pub_cmd_vel.publish(vel_cmd)
		elif self.act == 1: #hou zhuan
		    vel_cmd.angular.z = pi/2
	    	    self.pub_cmd_vel.publish(vel_cmd)
	    	    time.sleep(2)
            	    vel_cmd.linear.x = 0.2
	            vel_cmd.angular.z = 0.0
            	    self.pub_cmd_vel.publish(vel_cmd)
		elif self.act ==2: #you zhuan
		    vel_cmd.angular.z = -pi/4
	    	    self.pub_cmd_vel.publish(vel_cmd)
	    	    time.sleep(2)
            	    vel_cmd.linear.x = 0.2
	            vel_cmd.angular.z = 0.0
            	    self.pub_cmd_vel.publish(vel_cmd)
		else:#zuo zhuan
		    vel_cmd.angular.z = pi/4
	    	    self.pub_cmd_vel.publish(vel_cmd)
	    	    time.sleep(2)
            	    vel_cmd.linear.x = 0.2
	            vel_cmd.angular.z = 0.0
            	    self.pub_cmd_vel.publish(vel_cmd)
		self.act = action
		base_action[1] -= UNIT
		time.sleep(2)
        elif action == 1:  # down
            if state[1] < (HEIGHT - 1) * UNIT:
		if self.act == 0: #hou zhuan
		    vel_cmd.angular.z = pi/2
	    	    self.pub_cmd_vel.publish(vel_cmd)
	    	    time.sleep(2)
            	    vel_cmd.linear.x = 0.2
	            vel_cmd.angular.z = 0.0
            	    self.pub_cmd_vel.publish(vel_cmd)
		elif  self.act == 1: # zhi zou
	            vel_cmd.linear.x = 0.2
                    self.pub_cmd_vel.publish(vel_cmd)
		elif  self.act ==2: #zuo zhuan
		    vel_cmd.angular.z = pi/4
	    	    self.pub_cmd_vel.publish(vel_cmd)
	    	    time.sleep(2)
            	    vel_cmd.linear.x = 0.2
	            vel_cmd.angular.z = 0.0
            	    self.pub_cmd_vel.publish(vel_cmd)
		else:#you zhuan
		    vel_cmd.angular.z = -pi/4
	    	    self.pub_cmd_vel.publish(vel_cmd)
	    	    time.sleep(2)
            	    vel_cmd.linear.x = 0.2
	            vel_cmd.angular.z = 0.0
            	    self.pub_cmd_vel.publish(vel_cmd)
		self.act = action
                base_action[1] += UNIT
		time.sleep(2)
        elif action == 2:  # left
            if state[0] > UNIT:
		if self.act == 0: #zuo zhuan
		    vel_cmd.angular.z = pi/4
	    	    self.pub_cmd_vel.publish(vel_cmd)
	    	    time.sleep(2)
            	    vel_cmd.linear.x = 0.2
	            vel_cmd.angular.z = 0.0
            	    self.pub_cmd_vel.publish(vel_cmd)
		elif self.act == 1: #you zhuan
		    vel_cmd.angular.z = -pi/4
	    	    self.pub_cmd_vel.publish(vel_cmd)
	    	    time.sleep(2)
            	    vel_cmd.linear.x = 0.2
	            vel_cmd.angular.z = 0.0
            	    self.pub_cmd_vel.publish(vel_cmd)
		elif self.act == 2: #zhi zou
	            vel_cmd.linear.x = 0.2
                    self.pub_cmd_vel.publish(vel_cmd)
		else:#hou zhuan
		    vel_cmd.angular.z = pi/2
	    	    self.pub_cmd_vel.publish(vel_cmd)
	    	    time.sleep(2)
            	    vel_cmd.linear.x = 0.2
	            vel_cmd.angular.z = 0.0
            	    self.pub_cmd_vel.publish(vel_cmd)
		self.act = action
                base_action[0] -= UNIT
		time.sleep(2)
        elif action == 3:  # right
            if state[0] < (WIDTH - 1) * UNIT:
		if self.act == 0: #you zhuan
		    vel_cmd.angular.z = -pi/4
	    	    self.pub_cmd_vel.publish(vel_cmd)
	    	    time.sleep(2)
            	    vel_cmd.linear.x = 0.2
	            vel_cmd.angular.z = 0.0
            	    self.pub_cmd_vel.publish(vel_cmd)
		elif self.act == 1: #zuo zhuan
		    vel_cmd.angular.z = pi/4
	    	    self.pub_cmd_vel.publish(vel_cmd)
	    	    time.sleep(2)
            	    vel_cmd.linear.x = 0.2
	            vel_cmd.angular.z = 0.0
            	    self.pub_cmd_vel.publish(vel_cmd)
		elif self.act == 2: #hou zhuan
		    vel_cmd.angular.z = pi/2
	    	    self.pub_cmd_vel.publish(vel_cmd)
	    	    time.sleep(2)
            	    vel_cmd.linear.x = 0.2
	            vel_cmd.angular.z = 0.0
            	    self.pub_cmd_vel.publish(vel_cmd)
		else:#zhi zou
	            vel_cmd.linear.x = 0.2
                    self.pub_cmd_vel.publish(vel_cmd)
		self.act = action
                base_action[0] += UNIT
		time.sleep(2)

	        
	# move agent
        self.canvas.move(self.rectangle, base_action[0], base_action[1])
        # move rectangle to top level of canvas
        self.canvas.tag_raise(self.rectangle)
        next_state = self.canvas.coords(self.rectangle)

        # reward function
        if next_state == self.canvas.coords(self.circle):
            reward = 100
            done = True
	    rospy.loginfo("Success!!!")
	    self.pub_cmd_vel.publish(Twist())
	    self.act = 0
        elif next_state in [self.canvas.coords(self.triangle1),
                            self.canvas.coords(self.triangle2),
                            self.canvas.coords(self.triangle3),
                            self.canvas.coords(self.triangle4)]:
            reward = -100
            done = True
	    rospy.loginfo("Collision!!!")
	    self.pub_cmd_vel.publish(Twist())
	    self.act = 0
        else:
            reward = 0
            done = False

        next_state = self.coords_to_state(next_state)
        return next_state, reward, done

    def render(self):
        time.sleep(0.03)
        self.update()
