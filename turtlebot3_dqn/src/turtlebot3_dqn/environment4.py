#!/usr/bin/env python

# change the navigation for turtlebot3,with position and oratation
import rospy
import numpy as np
import os
import time
import subprocess
import tf
import math
from math import radians, copysign, sqrt, pow, pi, atan2
import time
import Tkinter as tk
from PIL import ImageTk, Image
from geometry_msgs.msg import Twist, Point, Pose, Quaternion
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler

np.random.seed(1)
PhotoImage = ImageTk.PhotoImage
UNIT = 50  # pixels
HEIGHT = 21  # grid height
WIDTH = 21  # grid width
kp_distance = 1
ki_distance = 0.01
kd_distance = 0.5

kp_angle = 1
ki_angle = 0.03
kd_angle = 0.05


class Env(tk.Tk):
    def __init__(self):
        tk.Tk.__init__(self)
        self.action_space = ['u', 'd', 'l', 'r', 'l_u', 'r_d', 'l_d', 'r_u']
        self.n_actions = len(self.action_space)
        self.title('Q Learning')
        self.geometry('{0}x{1}'.format(HEIGHT * UNIT, HEIGHT * UNIT))
        self.shapes = self.load_images()
        self.canvas = self._build_canvas()
        self.texts = []
        self.act = 0
        self.total_x = 0
        self.total_y = 0
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        position = Point()
        move_cmd = Twist()
        r = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'

        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Execption")

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
        self.rectangle = canvas.create_image(525, 525, image=self.shapes[0])
        self.triangle1 = canvas.create_image(325, 325, image=self.shapes[1])
        self.triangle2 = canvas.create_image(325, 725, image=self.shapes[1])
        self.triangle3 = canvas.create_image(725, 325, image=self.shapes[1])
        self.triangle4 = canvas.create_image(725, 725, image=self.shapes[1])
        self.circle = canvas.create_image(875, 175, image=self.shapes[2])

        # pack all
        canvas.pack()

        return canvas

    def start_env(self):
        self.migong = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], ]
        self.x1, self.y1 = 11, 11
        self.end_game = 0
        return self.migong

    def load_images(self):
        rectangle = PhotoImage(
            Image.open(
                "/home/wangqiang/catkin_ws/src/turtlebot3_machine_learning/turtlebot3_dqn/img/rectangle.png").resize(
                (35, 35)))
        triangle = PhotoImage(
            Image.open(
                "/home/wangqiang/catkin_ws/src/turtlebot3_machine_learning/turtlebot3_dqn/img/triangle.png").resize(
                (35, 35)))
        circle = PhotoImage(
            Image.open(
                "/home/wangqiang/catkin_ws/src/turtlebot3_machine_learning/turtlebot3_dqn/img/circle.png").resize(
                (35, 35)))

        return rectangle, triangle, circle

    def text_value(self, row, col, contents, action, font='Helvetica', size=7,
                   style='normal', anchor="nw"):

        if action == 0:
            origin_x, origin_y = 3, 21
        elif action == 1:
            origin_x, origin_y = 37, 21
        elif action == 2:
            origin_x, origin_y = 21, 3
        elif action == 3:
            origin_x, origin_y = 21, 33
        elif action == 4:
            origin_x, origin_y = 3, 3
        elif action == 5:
            origin_x, origin_y = 37, 33
        elif action == 6:
            origin_x, origin_y = 37, 3
        else:
            origin_x, origin_y = 3, 33

        x = origin_y + (UNIT * col)
        y = origin_x + (UNIT * row)
        font = (font, str(size), style)
        text = self.canvas.create_text(x, y, fill="black", text=contents,
                                       font=font, anchor=anchor)
        return self.texts.append(text)

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
        self.canvas.move(self.rectangle, UNIT / 2 - x + 500, UNIT / 2 - y + 500)
        self.render()
        self.total_x = 0
        self.total_y = 0
        # return observation
        return self.coords_to_state(self.canvas.coords(self.rectangle))

    def control(self, x, y, z):

        (position, rotation) = self.get_odom()
        last_rotation = 0
        angular_speed = 1
        linear_speed = 1
        position = Point()
        move_cmd = Twist()
        r = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'

        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Execption")

        (goal_x, goal_y, goal_z) = (float(x), float(y), float(z))
        if goal_z > 180 or goal_z < -180:
            print("input wrong goal_z range")
            self.shutdown()
        goal_z = np.deg2rad(goal_z)

        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        distance = goal_distance

        while abs(rotation - goal_z) > 0.02:
            (position, rotation) = self.get_odom()
            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z - pi:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
            else:
                if rotation <= goal_z + pi and rotation > goal_z:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
            self.cmd_vel.publish(move_cmd)
            r.sleep()
        self.cmd_vel.publish(Twist())

        while distance > 0.04:
            (position, rotation) = self.get_odom()
            x_start = position.x
            y_start = position.y
            path_angle = atan2(goal_y - y_start, goal_x - x_start)

            if path_angle < -pi / 4 or path_angle > pi / 4:
                if goal_y < 0 and y_start < goal_y:
                    path_angle = -2 * pi + path_angle
                elif goal_y >= 0 and y_start > goal_y:
                    path_angle = 2 * pi + path_angle
            if last_rotation > pi - 0.1 and rotation <= 0:
                rotation = 2 * pi + rotation
            elif last_rotation < -pi + 0.1 and rotation > 0:
                rotation = -2 * pi + rotation

            move_cmd.angular.z = angular_speed * path_angle - rotation

            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))

            move_cmd.linear.x = min(linear_speed * distance, 0.1)

            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

            last_rotation = rotation
            self.cmd_vel.publish(move_cmd)
            r.sleep()

        (position, rotation) = self.get_odom()
        print("Current position and rotation are: ", (position.x, position.y, rotation))
        print("reached: ^_^")

        # rospy.loginfo("Stopping the robot ...")
        self.cmd_vel.publish(Twist())
        return

    def step(self, action):
        state = self.canvas.coords(self.rectangle)
        base_action = np.array([0, 0])
        self.render()
        unit_distance = 0.185

        if action == 0:  # up
            if state[1] > UNIT:
                self.total_x = self.total_x + unit_distance
                self.total_y = self.total_y
                goal_z = 0
                self.control(self.total_x, self.total_y, goal_z)
                base_action[1] -= UNIT
                self.migong[self.y1][self.x1] = 0
                self.migong[self.y1 - 1][self.x1] = 1
                self.y1 -= 1
        elif action == 1:  # down
            if state[1] < (HEIGHT - 1) * UNIT:
                self.total_x = self.total_x - unit_distance
                self.total_x = self.total_y
                goal_z = 180
                self.control(self.total_x, self.total_y, goal_z)
                base_action[1] += UNIT
                self.migong[self.y1][self.x1] = 0
                self.migong[self.y1 + 1][self.x1] = 1
                self.y1 += 1
        elif action == 2:  # left
            if state[0] > UNIT:
                self.total_x = self.total_x
                self.total_y = self.total_y + unit_distance
                goal_z = 90
                self.control(self.total_x, self.total_y, goal_z)
                base_action[0] -= UNIT
                self.migong[self.y1][self.x1] = 0
                self.migong[self.y1][self.x1 - 1] = 1
                self.x1 -= 1
        elif action == 3:  # right
            if state[0] < (WIDTH - 1) * UNIT:
                self.total_x = self.total_x
                self.total_y = self.total_y - unit_distance
                goal_z = -90
                self.control(self.total_x, self.total_y, goal_z)
                base_action[0] += UNIT
                self.migong[self.y1][self.x1] = 0
                self.migong[self.y1][self.x1 + 1] = 1
                self.x1 += 1
        elif action == 4:  # left_up
            if state[1] > UNIT and state[0] > UNIT:
                self.total_x = self.total_x + unit_distance
                self.total_y = self.total_y + unit_distance
                goal_z = 45
                self.control(self.total_x, self.total_y, goal_z)
                base_action[0] -= UNIT
                base_action[1] -= UNIT
                self.migong[self.y1][self.x1] = 0
                self.migong[self.y1 - 1][self.x1 - 1] = 1
                self.x1 -= 1
                self.y1 -= 1
        elif action == 5:  # right_down
            if state[1] < (HEIGHT - 1) * UNIT and state[0] < (WIDTH - 1) * UNIT:
                self.total_x = self.total_x - unit_distance
                self.total_y = self.total_y - unit_distance
                goal_z = -135
                self.control(self.total_x, self.total_y, goal_z)
                base_action[0] += UNIT
                base_action[1] += UNIT
                base_action[1] -= UNIT
                self.migong[self.y1][self.x1] = 0
                self.migong[self.y1 + 1][self.x1 + 1] = 1
                self.x1 += 1
                self.y1 += 1
        elif action == 6:  # left_down
            if state[0] > UNIT and state[0] < (WIDTH - 1) * UNIT:
                self.total_x = self.total_x - unit_distance
                self.total_y = self.total_y + unit_distance
                goal_z = 135
                self.control(self.total_x, self.total_y, goal_z)
                base_action[0] -= UNIT
                base_action[1] += UNIT
                base_action[1] -= UNIT
                self.migong[self.y1][self.x1] = 0
                self.migong[self.y1 + 1][self.x1 - 1] = 1
                self.x1 -= 1
                self.y1 += 1
        elif action == 7:  # right_up
            if state[0] < (WIDTH - 1) * UNIT and state[1] > UNIT:
                self.total_x = self.total_x + unit_distance
                self.total_y = self.total_y - unit_distance
                goal_z = -45
                self.control(self.total_x, self.total_y, goal_z)
                base_action[0] += UNIT
                base_action[1] -= UNIT
                base_action[1] -= UNIT
                self.migong[self.y1][self.x1] = 0
                self.migong[self.y1 - 1][self.x1 + 1] = 1
                self.x1 += 1
                self.y1 -= 1

        # vel_cmd.linear.x = 0.0
        # vel_cmd.angular.z = 0.0
        # self.pub_cmd_vel.publish(vel_cmd)
        # rospy.sleep(0.1)
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
        return self.migong, reward, done

    def render(self):
        time.sleep(0.03)
        self.update()

    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return (Point(*trans), rotation[2])

    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(0.1)
