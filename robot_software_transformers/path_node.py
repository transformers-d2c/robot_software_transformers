import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D




def pose_equal(p1,p2):
    isEqual = (abs(p1.theta-p2.theta)<5)
    distance = ((p1.x-p2.x)**2 + (p1.y-p2.y)**2)**0.5
    isEqual = isEqual and (distance<5)
    return isEqual


class PathNode(Node):
    def __init__(self):
        # super().__init__('r_'+str(robot_id))
        time_period = 1/30
        self.time = self.create_timer(time_period, self.timer_callback)
        self.pose_sub = []
        self.pose_sub.append(self.create_subscription(Pose2D, 'r_1/pose', self.r_1_pose_callback, 1))
        self.pose_sub.append(self.create_subscription(Pose2D, 'r_2/pose', self.r_2_pose_callback, 1))
        self.pose_sub.append(self.create_subscription(Pose2D, 'r_3/pose', self.r_3_pose_callback, 1))
        self.pose_sub.append(self.create_subscription(Pose2D, 'r_4/pose', self.r_4_pose_callback, 1))
        self.pub_r_1 = self.create_publisher(Pose2D, 'r_1/target', 1)
        self.pub_r_2 = self.create_publisher(Pose2D, 'r_2/target', 1)
        self.pub_r_3 = self.create_publisher(Pose2D, 'r_3/target', 1)
        self.pub_r_4 = self.create_publisher(Pose2D, 'r_4/target', 1)
        self.pose = dict()
        self.step = 0
        self.path = [] 
        #Example path: [{'r_1':Pose2D(0,0,0),'r_2':Pose2D(0,0,0)},{'r_1':Pose2D(0,0,90),'r_2':Pose2D(0,0,0)}] 
        
    def timer_callback(self):
        if pose_equal(self.pose['r_1'],self.path[self.step]['r_1']) and pose_equal(self.pose['r_2'],self.path[self.step]['r_2']) and pose_equal(self.pose['r_3'],self.path[self.step]['r_3']) and pose_equal(self.pose['r_4'],self.path[self.step]['r_4']):
            self.step += 1
        self.pub_r_1.publish(self.path[self.step]['r_1'])
        self.pub_r_2.publish(self.path[self.step]['r_2'])
        self.pub_r_3.publish(self.path[self.step]['r_3'])
        self.pub_r_4.publish(self.path[self.step]['r_4'])

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
    path_node = PathNode()
    rclpy.spin(path_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()