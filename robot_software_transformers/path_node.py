import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import functools
from std_msgs.msg import Bool
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import json



def pose_equal(p1,p2):
    """check if 2 poses are equal using sqrt(a^2+b^2) //formula ka naam kya ha bc"""
    isEqual = (abs(p1.theta-p2.theta)<5)
    distance = ((p1.x-p2.x)**2 + (p1.y-p2.y)**2)**0.5
    isEqual = isEqual and (distance<15)
    return distance<15 # returns a true if dist < 15 or a false


class PathNode(Node):
    def __init__(self,num):
        super().__init__('path_node'+str(num)) # making multiple nodes with different names since multiple nodes cant have same name
        time_period = 1/30
        self.time = self.create_timer(time_period, functools.partial(self.timer_callback,num=num)) # call the timer_callback in 30Hz
        self.pose_sub = [] # list to store the subbed objects
        if(num == 1):
            self.pose_sub.append(self.create_subscription(Pose2D, 'r_1/pose', self.r_1_pose_callback, 1)) # subb to r_1/pose
            self.pub_r_1 = self.create_publisher(Pose2D, 'r_1/target', 1) # create publisher of r_1/target
            self.pub_bool_r_1 = self.create_publisher(Bool, 'r_1/Bool', 1) # create publisher of r_1/bool
        elif(num == 2):
            self.pose_sub.append(self.create_subscription(Pose2D, 'r_2/pose', self.r_2_pose_callback, 1)) # subb to r_2/pose
            self.pub_r_2 = self.create_publisher(Pose2D, 'r_2/target', 1) # create publisher of r_2/target
            self.pub_bool_r_2 = self.create_publisher(Bool, 'r_2/Bool', 1) # create publisher of r_2/bool
        elif(num == 3):
            self.pose_sub.append(self.create_subscription(Pose2D, 'r_3/pose', self.r_3_pose_callback, 1)) # subb to r_3/pose
            self.pub_r_3 = self.create_publisher(Pose2D, 'r_3/target', 1) # create publisher of r_3/target
            self.pub_bool_r_3 = self.create_publisher(Bool, 'r_3/Bool', 1) # create publisher of r_3/bool
        elif(num == 4):
            self.pose_sub.append(self.create_subscription(Pose2D, 'r_4/pose', self.r_4_pose_callback, 1)) # subb to r_4/pose
            self.pub_r_4 = self.create_publisher(Pose2D, 'r_4/target', 1) # create publisher of r_4/target
            self.pub_bool_r_4 = self.create_publisher(Bool, 'r_4/Bool', 1) # create publisher of r_4/bool
        self.step = 0 # init steps with 0
        self.path = [] # empty list to store path
        with open('/robot_software_transformers-main/src/robot_software_transformers/robot_software_transformers/path.json','r') as f:
            data = json.load(f) # load the predefined path in data
            for four_point in data:
                """parsing of data in the json"""
                path_point = dict()
                for robot_id in four_point:
                    t = Pose2D()
                    t.x = four_point[robot_id]['x'] # assign x coordinate
                    t.y = four_point[robot_id]['y'] # assign y coordinate
                    if four_point[robot_id]['flip']: # for some reason we using theta with the flip bit because the angle we are calculating using some forumala //naam iska bhi bhul gaya bc
                        t.theta = 90.0
                    else:
                        t.theta = 0.0
                    path_point[robot_id] = t # place the t object with the values in the dictionary
                self.path.append(path_point) # append the dictionary in the list
        """initialize some values to t"""
        t = Pose2D()
        t.x = 12.0
        t.y = 30.0
        t.theta = 150.0
        self.pose = {'r_'+str(num):t}


        #Example path: [{'r_1':Pose2D(0,0,0),'r_2':Pose2D(0,0,0)},{'r_1':Pose2D(0,0,90),'r_2':Pose2D(0,0,0)}] 
        
    def timer_callback(self,num):
        if(num == 1):
            if self.step<len(self.path)-1:
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
            if self.step<len(self.path)-1:
                self.pub_r_2.publish(self.path[self.step]['r_2'])
                if pose_equal(self.pose['r_2'],self.path[self.step]['r_2']):
                    self.step += 1
                temp = Bool()
                temp.data = True
                self.pub_bool_r_2.publish(temp)
            else:
                temp = Bool()
                temp.data = False
                self.pub_bool_r_2.publish(temp)
        elif(num == 3):
            if self.step<len(self.path)-1:
                self.pub_r_3.publish(self.path[self.step]['r_3'])
                if pose_equal(self.pose['r_3'],self.path[self.step]['r_3']):
                    self.step += 1
                temp = Bool()
                temp.data = True
                self.pub_bool_r_3.publish(temp)
            else:
                temp = Bool()
                temp.data = False
                self.pub_bool_r_3.publish(temp)
        elif(num == 4):
            if self.step<len(self.path)-1:
                self.pub_r_4.publish(self.path[self.step]['r_4'])
                if pose_equal(self.pose['r_4'],self.path[self.step]['r_4']):
                    self.step += 1
                temp = Bool()
                temp.data = True
                self.pub_bool_r_4.publish(temp)
            else:
                temp = Bool()
                temp.data = False
                self.pub_bool_r_4.publish(temp)
        

    def r_1_pose_callback(self, msg):
        self.pose['r_1'] = msg
    
    def r_2_pose_callback(self, msg):
        self.pose['r_2'] = msg

    def r_3_pose_callback(self, msg):
        self.pose['r_3'] = msg

    def r_4_pose_callback(self, msg):
        self.pose['r_4'] = msg


def main():
    rclpy.init() # init rclpy
    nodes = [] # make a list to store nodes
    try:
        executor = MultiThreadedExecutor(4) # 4 multithreaded executors
        for i in range(4):
            path_node = PathNode(i+1) # objects of pathnode
            nodes.append(path_node) # append the objects in list
            executor.add_node(path_node) # add object as node in executor
        try:
            executor.spin() # spin the executor
        finally:
            executor.shutdown() # shutdown the executor
            for i in range(4):
                nodes[i].destroy_node() # destory the nodes
    finally:
        rclpy.shutdown() # shutdown rclpy

if __name__ == '__main__':
    main()
