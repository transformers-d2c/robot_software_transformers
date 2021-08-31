import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
import sys
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import json
import socket

class SocketNode(Node):

    def __init__(self,robot_id, robot_socket):
        super().__init__('socket_r_'+str(robot_id))
        self.group = ReentrantCallbackGroup()
        self.robot_socket = robot_socket
        self.timer = self.create_timer(1/30,self.timer_callback,self.group)
        self.pose_sub = self.create_subscription(Pose2D,'r_'+str(robot_id)+'/pose',self.pose_callback,1,callback_group=self.group)
        self.target_sub = self.create_subscription(Pose2D,'r_'+str(robot_id)+'/target',self.target_callback,1,callback_group=self.group)
        self.pose = Pose2D()
        self.target = Pose2D()
        self.flip = Bool()
        self.pose.x,self.pose.y,self.pose.theta = 0.0,0.0,0.0
        self.target.x,self.target.y,self.target.theta = 0.0,0.0,0.0
        self.flip.data = False

    def timer_callback(self):
        dict = {
            'pose':{
                'x':self.pose.x,
                'y':self.pose.y,
                'theta':self.pose.theta
            },
            'target':{
                'x':self.target.x,
                'y':self.target.y,
                'theta':self.target.theta
            },
            'flip':self.flip.data
        }
        message = json.dumps(dict,ensure_ascii=True)
        self.robot_socket.sendall(message.encode('ascii'))

    def pose_callback(self,msg):
        self.pose = msg

    def target_callback(self,msg):
        self.target = msg

def main():
    port = 5000
    if(len(sys.argv)>=2):
        port = int(sys.argv[1])
    rclpy.init()
    listener_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM,0)
    listener_socket.bind(('',port))
    listener_socket.listen(10)
    nodes = []
    ids = []
    addMore = 'y'
    while addMore=='y':
        (robot_socket, address) = listener_socket.accept()
        robot_id = int(robot_socket.recv(32).decode('ascii'))
        rnode = SocketNode(robot_id,robot_socket)
        nodes.append(rnode)
        ids.append(robot_id)
        print('Robots connected: ',end = '')
        print(str(ids))
        addMore = str(input('Add more robots(y/n)?: '))
    try:
        executor = MultiThreadedExecutor(4)
        for i in range(len(nodes)):
            executor.add_node(nodes[i])
        try:
            executor.spin()
        finally:
            executor.shutdown()
            for i in range(len(nodes)):
                nodes[i].destroy_node()
    finally:
        rclpy.shutdown()
    


        


if __name__=='__main__':
    main()