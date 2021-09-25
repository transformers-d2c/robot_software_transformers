import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import functools
from std_msgs.msg import Bool
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import json


def pose_equal(p1,p2):
    isEqual = (abs(p1.theta-p2.theta)<5)
    distance = ((p1.x-p2.x)**2 + (p1.y-p2.y)**2)**0.5
    isEqual = isEqual and (distance<15)
    return isEqual


class PathNode(Node):
    def __init__(self,num):
        super().__init__('path_node_'+str(num))
        self.group = MutuallyExclusiveCallbackGroup()
        time_period = 1/30
        self.time = self.create_timer(time_period, functools.partial(self.timer_callback,num=num))
        self.pose_sub = []
        if(num == 1):
            self.pose_sub.append(self.create_subscription(Pose2D, 'r_1/pose', self.r_1_pose_callback, 1))
            self.pub_r_1 = self.create_publisher(Pose2D, 'r_1/target', 1)
            self.pub_bool_r_1 = self.create_publisher(Bool, 'r_1/Bool', 1)
        elif(num == 2):
            self.pose_sub.append(self.create_subscription(Pose2D, 'r_2/pose', self.r_2_pose_callback, 1))
            self.pub_r_2 = self.create_publisher(Pose2D, 'r_2/target', 1)
            self.pub_bool_r_2 = self.create_publisher(Bool, 'r_2/Bool', 1)
        elif(num == 3):
            self.pose_sub.append(self.create_subscription(Pose2D, 'r_3/pose', self.r_3_pose_callback, 1))
            self.pub_r_3 = self.create_publisher(Pose2D, 'r_3/target', 1)
            self.pub_bool_r_3 = self.create_publisher(Bool, 'r_3/Bool', 1)
        elif(num == 4):
            self.pose_sub.append(self.create_subscription(Pose2D, 'r_4/pose', self.r_4_pose_callback, 1))
            self.pub_r_4 = self.create_publisher(Pose2D, 'r_4/target', 1)
            self.pub_bool_r_4 = self.create_publisher(Bool, 'r_4/Bool', 1)
        self.step = 0
        self.path = []
        with open('/robot_software_transformers-main/src/robot_software_transformers/robot_software_transformers/path.json','r') as f:
            data = json.load(f)
        for four_point in data:
            path_point = dict()
            t = Pose2D()
            t.x = four_point['r_'+str(num)]['x']
            t.y = four_point['r_'+str(num)]['y']
            if four_point['r_'+str(num)]['flip']:
                t.theta = 90.0
            else:
                t.theta = 0.0
            path_point[num] = t
            self.path.append(path_point) 
            self.pose = {'r_'+str(num):t}


        #Example path: [{'r_1':Pose2D(0,0,0),'r_2':Pose2D(0,0,0)},{'r_1':Pose2D(0,0,90),'r_2':Pose2D(0,0,0)}] 
        
    def timer_callback(self,num):
        if(num == 1):
            if self.step<len(self.path):
                self.pub_r_1.publish(self.path[self.step]['r_1'])
                if pose_equal(self.pose['r_1'],self.path[self.step]['r_1']):
                    self.step += 1
                temp = Bool()
                temp.data = True
                self.pub_bool_r_1.publish(temp)
            else:
                temp = Bool()
                temp.data = False
                self.pub_bool_r_1.publish(temp)
        elif(num == 2):
            if self.step<len(self.path[1]):
                self.pub_r_2.publish(self.path[self.step]['r_2'])
                if pose_equal(self.pose['r_2'],self.path[self.step]['r_2']):
                    self.step += 1
                temp = Bool()
                temp.data = True
                self.pub_bool_r_1.publish(temp)
            else:
                temp = Bool()
                temp.data = False
                self.pub_bool_r_1.publish(temp)
        elif(num == 3):
            if self.step<len(self.path):
                self.pub_r_3.publish(self.path[self.step]['r_3'])
                if pose_equal(self.pose['r_3'],self.path[self.step]['r_3']):
                    self.step += 1
                temp = Bool()
                temp.data = True
                self.pub_bool_r_1.publish(temp)
            else:
                temp = Bool()
                temp.data = False
                self.pub_bool_r_1.publish(temp)
        elif(num == 4):
            if self.step[3]<len(self.path[3]):
                self.pub_r_4.publish(self.path[self.step]['r_4'])
                if pose_equal(self.pose['r_4'],self.path[self.step]['r_4']):
                    self.step += 1
                temp = Bool()
                temp.data = True
                self.pub_bool_r_1.publish(temp)
            else:
                temp = Bool()
                temp.data = False
                self.pub_bool_r_1.publish(temp)

    def r_1_pose_callback(self, msg):
        self.pose['r_1'] = msg
    
    def r_2_pose_callback(self, msg):
        self.pose['r_2'] = msg

    def r_3_pose_callback(self, msg):
        self.pose['r_3'] = msg

    def r_4_pose_callback(self, msg):
        self.pose['r_4'] = msg


def main():
    rclpy.init()
    nodes = []
    try:
        executor = MultiThreadedExecutor(4)
        for i in range(4):
            path_node = PathNode(i+1)
            nodes.append(path_node)
            executor.add_node(path_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            for i in range(4):
                nodes[i].destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
