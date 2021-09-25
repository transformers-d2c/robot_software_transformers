import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
import functools

class CentralPathNode(Node):
    def __init__(self,robots):
        super().__init__('central_path_node') # making node called cpn
        self.time_period = 1/30
        """subscribing to all bool topics and target topics"""
        self.r_1_bool_sub = self.create_subscription(Bool, 'r_1/bool', self.r_1_callback, 1)
        self.r_2_bool_sub = self.create_subscription(Bool, 'r_2/bool', self.r_2_callback, 1)
        self.r_3_bool_sub = self.create_subscription(Bool, 'r_3/bool', self.r_3_callback, 1)
        self.r_4_bool_sub = self.create_subscription(Bool, 'r_4/bool', self.r_4_callback, 1)
        self.r_1_pose_sub = self.create_subscription(Pose2D, 'r_1/target', self.r_1_pose_callback, 1)
        self.r_2_pose_sub = self.create_subscription(Pose2D, 'r_2/target', self.r_2_pose_callback, 1)
        self.r_3_pose_sub = self.create_subscription(Pose2D, 'r_3/target', self.r_3_pose_callback, 1)
        self.r_4_pose_sub = self.create_subscription(Pose2D, 'r_4/target', self.r_4_pose_callback, 1)
        """making a publisher to publish in final_target topics"""
        self.r_1_pose_final = self.create_publisher(Pose2D, 'r_1/final_target', 1)
        self.r_2_pose_final = self.create_publisher(Pose2D, 'r_2/final_target', 1)
        self.r_3_pose_final = self.create_publisher(Pose2D, 'r_3/final_target', 1)
        self.r_4_pose_final = self.create_publisher(Pose2D, 'r_4/final_target', 1)
        
    def r_1_callback(self,msg):
        if(msg.data==True):
            """if the bool data is true meaning bot1 is active hence will work or else wont"""
            self.time = self.create_timer(self.time_period, functools.partial(self.timer_callback,content=self.r_1_pose_sub))
    
    def r_2_callback(self,msg):
        if(msg.data==True):
            """if the bool data is true meaning bot2 is active hence will work or else wont"""
            self.time = self.create_timer(self.time_period, functools.partial(self.timer_callback,content=self.r_2_pose_sub))
    
    def r_3_callback(self,msg):
        if(msg.data==True):
            """if the bool data is true meaning bot3 is active hence will work or else wont"""
            self.time = self.create_timer(self.time_period, functools.partial(self.timer_callback,content=self.r_3_pose_sub))
    
    def r_4_callback(self,msg):
        if(msg.data==True):
            """if the bool data is true meaning bot4 is active hence will work or else wont"""
            self.time = self.create_timer(self.time_period, functools.partial(self.timer_callback,content=self.r_4_pose_sub))
            
    def timer_callback(self, content):
        """This function will simply recieve the Pose2D object as content and then publish it in new topic final_target"""
        self.r_1_pose_final.publish(content)
        
def main():
    rclpy.init() # init rclpy
    central_path_node = CentralPathNode() # init object of CentralPathNode
    rclpy.spin(central_path_node) # spin the object
    rclpy.shutdown() # shutdown

if __name__ == '__main__':
    main()
