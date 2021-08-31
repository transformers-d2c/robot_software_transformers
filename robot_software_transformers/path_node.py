import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D

class PathNode(Node):
    def __init__(self):
        # super().__init__('r_'+str(robot_id))
        time_period = 1/30
        self.time = self.create_timer(time_period, self.timer_callback)
        self.pose_sub = []
        self.pose_sub.append({'r_1': self.create_subscription(Pose2D, 'r_1/pose', self.r_1_pose_callback, 1)})
        self.pose_sub.append({'r_2': self.create_subscription(Pose2D, 'r_2/pose', self.r_2_pose_callback, 1)})
        self.pose_sub.append({'r_3': self.create_subscription(Pose2D, 'r_3/pose', self.r_3_pose_callback, 1)})
        self.pose_sub.append({'r_4': self.create_subscription(Pose2D, 'r_4/pose', self.r_4_pose_callback, 1)})
        self.pub_r_1 = self.create_publisher(Pose2D, 'r_1/target', 1)
        self.pub_r_2 = self.create_publisher(Pose2D, 'r_2/target', 1)
        self.pub_r_3 = self.create_publisher(Pose2D, 'r_3/target', 1)
        self.pub_r_4 = self.create_publisher(Pose2D, 'r_4/target', 1)
        self.pose = []
        self.step = 0
        
    def timer_callback(self, msg):
        pass

    def r_1_pose_callback(self, msg):
        self.pose.append({'r_1': msg})
    
    def r_2_pose_callback(self, msg):
        self.pose.append({'r_2': msg})

    def r_3_pose_callback(self, msg):
        self.pose.append({'r_3': msg})

    def r_4_pose_callback(self, msg):
        self.pose.append({'r_4': msg})


def main():
    path_ls = [{'r1': 'pose', 'r2': 'pose'}, {'r1': 'pose', 'r2': 'pose'}, {'r1': 'pose', 'r2': 'pose'}]
    rclpy.init()
    path_node = PathNode()
    for i in range(len(path_node.pose_sub)):
        if path_ls[i] == path_node.pose_sub[i]:
            path_node.step += 1
    rclpy.shutdown()

if __name__ == '__main__':
    main()