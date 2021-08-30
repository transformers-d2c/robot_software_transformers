import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import sys

class CentralNode(Node):

    def __init__(self, n_camera, robot_id):
        super().__init__('central_node')
        self.publisher = self.create_publisher(Pose2D, 'r_'+str(robot_id), 1)
        time_period = 1/30
        self.timer = self.create_timer(time_period, self.pub_callback)
        self.n_camera = n_camera
        self.robot_id = robot_id
        self.subs = []
        for i in range(n_camera):
            self.subs.append(self.create_subscription(Pose2D, 'c_'+str(i+1)+'/r_'+str(robot_id), self.subs_callback, 1))
        self.camera_pose = []
    
    def subs_callback(self, msg):
        self.camera_pose.append(msg)

    def pub_callback(self):
        pose_X = 0
        pose_Y = 0
        pose_theta = 0
        if len(self.camera_pose) > 0:
            for pose in self.camera_pose:
                pose_X += pose.x
                pose_Y += pose.y
                pose_theta += pose.theta
            pose_X = pose_X/len(self.camera_pose)
            pose_Y = pose_Y/len(self.camera_pose)
            pose_theta = pose_theta/len(self.camera_pose)
            self.camera_pose = []
            self.publisher.publish(Pose2D(pose_X, pose_Y, pose_theta))
    
    def send_to_node():
        """
        Store the data in DS and return it to be used for socket_handler.py
        """
        pass
    
def main():
    n_camera = int(sys.argv[1])
    robot_id = int(sys.argv[2])
    print(n_camera, robot_id)
    rclpy.init()
    central_node = CentralNode(n_camera, robot_id)
    rclpy.spin(central_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
