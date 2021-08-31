import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D

class PathNode(Node):
    def __init__(self, n_robots, robot_id):
        super().__init__('r_'+str(robot_id))
        time_period = 1/30
        self.time = self.create_timer(time_period, self.timer_callback)
        self.n_robots = n_robots
        self.robot_id = robot_id
        self.pose_sub = []
        for i in range(n_robots):
            self.pose_sub.append(self.create_subscription(Pose2D, 'r_'+str(i+1), self.pose_callback, 1))
        self.pose = []
        
    def timer_callback(self, msg):
        pass

    def pose_callback(self, msg):
        self.pose.append(msg)


def main():
    path_ls = [{'r1': 'pose', 'r2': 'pose'}, {'r1': 'pose', 'r2': 'pose'}, {'r1': 'pose', 'r2': 'pose'}]
    rclpy.init()
    path_node = PathNode()