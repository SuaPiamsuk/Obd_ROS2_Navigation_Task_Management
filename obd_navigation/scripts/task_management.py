#!/usr/bin/env python3

from obd_interfaces.srv import Goal       

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2

from robot_navigator import BasicNavigator, NavigationResult # Helper module

import yaml

import time
import math
import numpy as np

def quaternion_from_euler(ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q

class Task_management(Node):

    def __init__(self):
        super().__init__('task_management')
        self.srv = self.create_service(Goal, 'send_goal_nav2', self.srv_callback)        
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.navigator = BasicNavigator()

        #path to your YAML file
        file_path = '/home/sua/obd_ws/src/obd_navigation/config/waypoints.yaml'

        # Call the function to read the YAML file
        # self.yaml_data = self.read_yaml_file(file_path)

        with open(file_path, 'r') as file:
            yaml_data = file.read()
            self.yaml_data = yaml.safe_load(yaml_data)
        
        self.i = 0
        self.i_max = len(self.yaml_data['waypoint'])

        self.last_time = time.time() - self.yaml_data['wait_at_waypoint'] #now - offset (don't have to wait in first waypoints)

        self.x = None
        self.y = None
        self.theta = None

        self.srv_status = 0

        self.state = "navigate from file"
        self.send_goal_succeeded = 0

    
    
    def timer_callback(self):

        if self.state == "navigate from file":
            if self.send_goal_succeeded == 0:
                print((self.yaml_data['waypoint'][self.i]['point']))
                self.navigator.waitUntilNav2Active()

                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                goal_pose.pose.position.x = self.yaml_data['waypoint'][self.i]['point']['x']
                goal_pose.pose.position.y = self.yaml_data['waypoint'][self.i]['point']['y']
                goal_pose.pose.position.z = 0.0
                q = quaternion_from_euler(0, 0, self.yaml_data['waypoint'][self.i]['point']['theta'])
                goal_pose.pose.orientation.x = q[0]
                goal_pose.pose.orientation.y = q[1]
                goal_pose.pose.orientation.z = q[2]
                goal_pose.pose.orientation.w = q[3]

                self.navigator.goToPose(goal_pose)

                self.i = self.i + 1
                self.send_goal_succeeded = 1

            elif self.navigator.isNavComplete():
                self.last_time = time.time()
                self.send_goal_succeeded = 0
                self.state = "wait"
        
        elif self.state == "wait":
            if (time.time()-self.last_time >= self.yaml_data['wait_at_waypoint']) and self.i < self.i_max :
                self.state = "navigate from file"
                print("finished")

            elif (time.time()-self.last_time >= self.yaml_data['wait_at_waypoint']):
                self.state = "navigate from srv"
                print("finished")
            
            else:
                print(str(time.time()-self.last_time) + " seconds")
                
 
        elif self.state == "navigate from srv":
            if self.send_goal_succeeded == 0 and self.srv_status == 1:

                self.navigator.waitUntilNav2Active()

                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                goal_pose.pose.position.x = self.x
                goal_pose.pose.position.y = self.y
                goal_pose.pose.position.z = 0.0
                q = quaternion_from_euler(0, 0, self.theta)
                goal_pose.pose.orientation.x = q[0]
                goal_pose.pose.orientation.y = q[1]
                goal_pose.pose.orientation.z = q[2]
                goal_pose.pose.orientation.w = q[3]

                self.navigator.goToPose(goal_pose)

                self.i = self.i + 1
                self.send_goal_succeeded = 1
                self.srv_status = 0

            elif self.navigator.isNavComplete() and self.send_goal_succeeded == 1:
                self.last_time = time.time()
                self.send_goal_succeeded = 0
                self.state = "wait"
            

    def srv_callback(self, request, response):

        self.srv_status = 1

        self.x = request.x
        self.y = request.y
        self.theta = request.theta

        response.result = "send goal succeeded"

        return response


def main(args=None):
    rclpy.init(args=args)

    task_management = Task_management()

    rclpy.spin(task_management)

    rclpy.shutdown()

if __name__ == '__main__':
    main()










# import yaml

# def read_yaml_file(file_path):
#     with open(file_path, 'r') as file:
#         yaml_data = file.read()
#         data = yaml.safe_load(yaml_data)
#     return data

# # Provide the path to your YAML file
# file_path = '/home/sua/obd_ws/src/obd_navigation/config/waypoints.yaml'
# # file_path = '../../config/waypoints.yaml'

# # Call the function to read the YAML file
# yaml_data = read_yaml_file(file_path)

# # Now you can work with the YAML data
# # For example, printing the contents of the YAML file
# # print(yaml_data['waypoint_2']['x'])
# # print(len(yaml_data['waypoint_1']))
# # print((yaml_data['waypoint'][0]['point']['x']))
# print((yaml_data['wait_at_waypoint']))
# print("h"+"r")


# # เก็บใน buffer แค่อันล่าสุดพอ /
# # ต้องเพิ่มฟังชั่นแปลงมุม
# # ปรับ angular tolerance /
# # เปลี่ยนชื่อ srv /
# # แปลง code เป็น state machine