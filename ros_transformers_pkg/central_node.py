import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D


class CentralNode(Node):

    def __init__(self, nCamera, RobotID):
        super().__init__('central_node')
        self.publisher = self.create_publisher(Pose2D, 'r_'+str(RobotID), 1)
        time_period = 1/30
        self.timer = self.create_timer(time_period, self.pub_callback)
        self.nCamera = nCamera
        self.RobotID = RobotID
        self.subs = []
        for i in range(nCamera):
            self.subs.append(self.create_subscription(Pose2D, 'c_'+str(i+1)+'/r_'+str(RobotID), self.subsCallback, 1))
        self.camera_pose = []
    
    def subsCallback(self, msg):
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
    
def main():
    rclpy.init()
    central_node = CentralNode(3, 1)
    rclpy.spin(central_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
