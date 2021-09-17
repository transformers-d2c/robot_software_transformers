import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import sys
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from statistics import mode


def pose_tolerance(p1,p2):
    isEqual = (abs(p1.theta-p2.theta)<30000)
    distance = ((p1.x-p2.x)**2 + (p1.y-p2.y)**2)**0.5
    isEqual = isEqual and (distance<5000000)
    return isEqual
class CentralNode(Node):

    def __init__(self, n_camera, robot_id):
        super().__init__('r_'+str(robot_id))
        self.group = MutuallyExclusiveCallbackGroup()
        self.publisher = self.create_publisher(Pose2D, 'r_'+str(robot_id)+'/pose', 1)
        time_period = 1/30
        self.timer = self.create_timer(time_period, self.pub_callback, callback_group=self.group)
        self.n_camera = n_camera
        self.robot_id = robot_id
        self.subs = []
        for i in range(n_camera):
            self.subs.append(self.create_subscription(Pose2D, 'c_'+str(i+1)+'/r_'+str(robot_id), self.subs_callback, 1, callback_group=self.group))
        self.camera_pose = []
        self.old_pose = None
        self.old_poses_x = []
        self.old_poses_y = []
        self.old_poses_theta = []
    
    def subs_callback(self, msg):
        self.camera_pose.append(msg)

    def pub_callback(self):
        pose_holder = Pose2D()
        pose_holder.x = 0.0
        pose_holder.y = 0.0
        pose_holder.theta = 0.0
        cnt = 0
        if len(self.camera_pose) > 0:
            if self.old_pose is None:
                for pose in self.camera_pose:
                    pose_holder.x += pose.x
                    pose_holder.y += pose.y
                    pose_holder.theta += pose.theta
                    cnt = cnt + 1
            else:
                for pose in self.camera_pose:
                    if pose_tolerance(pose,self.old_pose):
                        pose_holder.x += pose.x
                        pose_holder.y += pose.y
                        pose_holder.theta += pose.theta
                        cnt = cnt + 1
            if cnt>0:
                pose_holder.x = pose_holder.x/cnt
                pose_holder.y = pose_holder.y/cnt
                pose_holder.theta = pose_holder.theta/cnt
                pose_holder.x = (pose_holder.x//5)*5
                pose_holder.y = (pose_holder.y//5)*5
                pose_holder.theta = (pose_holder.theta//5)*5
                self.old_pose = Pose2D()
                self.old_pose.x = pose_holder.x
                self.old_pose.y = pose_holder.y
                self.old_pose.theta = pose_holder.theta
                self.old_poses_x.append(pose_holder.x)
                self.old_poses_y.append(pose_holder.y)
                self.old_poses_theta.append(pose_holder.theta)
                self.old_poses_x = self.old_poses_x[max(-1*len(self.old_poses_x),-5):]
                self.old_poses_y = self.old_poses_y[max(-1*len(self.old_poses_y),-5):]
                self.old_poses_theta = self.old_poses_theta[max(-1*len(self.old_poses_theta),-5):]
                pose_holder = Pose2D()
                pose_holder.x = mode(self.old_poses_x)
                pose_holder.y = mode(self.old_poses_y)
                pose_holder.theta = mode(self.old_poses_theta)
                self.publisher.publish(pose_holder)
            self.camera_pose = []
    
def main():
    n_camera = int(sys.argv[1])
    n_robots = int(sys.argv[2])
    print(n_camera, n_robots)
    rclpy.init()
    nodes = []
    try:
        executor = MultiThreadedExecutor(4)
        for i in range(n_robots):
            cnode = CentralNode(n_camera, i+1)
            nodes.append(cnode)
            executor.add_node(cnode)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            for i in range(n_robots):
                nodes[i].destroy_node()
    finally:
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()
