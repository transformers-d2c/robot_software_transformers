import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool as bool


class CentralPathNode(Node):
    def __init__(self,robots):
        super().__init__('central_path_node')
        self.time_period = 1/30
        #self.time = self.create_timer(time_period, self.timer_callback(num))
        self.r_1_bool_sub = self.create_subscription(bool, 'r_1/bool', self.r_1_callback, 1)
        self.r_2_bool_sub = self.create_subscription(bool, 'r_2/bool', self.r_2_callback, 1)
        self.r_3_bool_sub = self.create_subscription(bool, 'r_3/bool', self.r_3_callback, 1)
        self.r_4_bool_sub = self.create_subscription(bool, 'r_4/bool', self.r_4_callback, 1)
        self.r_1_pose_sub = self.create_subscription(Pose2D, 'r_1/target', self.r_1_pose_callback, 1)
        self.r_2_pose_sub = self.create_subscription(Pose2D, 'r_2/target', self.r_2_pose_callback, 1)
        self.r_3_pose_sub = self.create_subscription(Pose2D, 'r_3/target', self.r_3_pose_callback, 1)
        self.r_4_pose_sub = self.create_subscription(Pose2D, 'r_4/target', self.r_4_pose_callback, 1)
        self.r_1_pose_final = self.create_publisher(Pose2D, 'r_1/final_target', 1)
        self.r_2_pose_final = self.create_publisher(Pose2D, 'r_2/final_target', 1)
        self.r_3_pose_final = self.create_publisher(Pose2D, 'r_3/final_target', 1)
        self.r_4_pose_final = self.create_publisher(Pose2D, 'r_4/final_target', 1)
        
    def r_1_callback(self,msg):
        if(msg==True):
            self.time = self.create_timer(self.time_period, self.timer_callback('r_1',self.r_1_pose_sub))
    
    def r_2_callback(self,msg):
        if(msg==True):
            self.time = self.create_timer(self.time_period, self.timer_callback('r_2',self.r_2_pose_sub))
    
    def r_3_callback(self,msg):
        if(msg==True):
            
            self.time = self.create_timer(self.time_period, self.timer_callback('r_3',self.r_3_pose_sub))
    
    def r_4_callback(self,msg):
        if(msg==True):
            
            self.time = self.create_timer(self.time_period, self.timer_callback('r_4',self.r_4_pose_sub))
            
    def timer_callback(self, rID, content):
        self.r_1_pose_final.publish(content)
        
def main():
    rclpy.init()
    central_path_node = CentralPathNode()
    rclpy.spin(central_path_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
