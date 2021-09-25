import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import sys
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from statistics import mode
import functools

def pose_tolerance(p1,p2):
    isEqual = (abs(p1.theta-p2.theta)<30000)
    distance = ((p1.x-p2.x)**2 + (p1.y-p2.y)**2)**0.5
    isEqual = isEqual and (distance<5000000)
    return isEqual
class CentralNode(Node):

    def __init__(self, n_camera, robot_id): # passing the number of cams and robots
        super().__init__('r_'+str(robot_id)) # making robot nodes like r_1/r_2.....
        self.group = MutuallyExclusiveCallbackGroup() # preventing race conditions
        self.publisher = self.create_publisher(Pose2D, 'r_'+str(robot_id)+'/pose', 1) # create pub with topic r_1/pose, r_2/pose...
        time_period = 1/30
        self.timer = self.create_timer(time_period, self.pub_callback, callback_group=self.group) # create timer to call publisher_callback and callback group will define that the code belongs to the mutually exclusive callgroups
        self.n_camera = n_camera
        self.robot_id = robot_id
        self.subs = []
        self.camera_pose = []
        self.camera_available = dict()
        for i in range(n_camera):
            self.subs.append(self.create_subscription(Pose2D, 'c_'+str(i+1)+'/r_'+str(robot_id), functools.partial(self.subs_callback,camera_id = i), 1, callback_group=self.group)) # making robot nodes like r_1/r_2..... and functools.partial passes params
            self.camera_pose.append([]) # appending empty list to cam_pose list
            self.camera_available[i] = False
        self.old_pose = None
        self.old_poses_x = []
        self.old_poses_y = []
        self.old_poses_theta = []
    
    def subs_callback(self, msg, camera_id):
        self.camera_pose[camera_id].append(msg) # appending msg recieved from the topic in the list
        self.camera_available[camera_id] = True # and setting bool to true

    def pub_callback(self):
        pose_holder = Pose2D()
        pose_holder.x = 0.0 # init with 0
        pose_holder.y = 0.0 # init with 0
        pose_holder.theta = 0.0 # init with 0
        cnt = 0 # init with 0
        camera_id = -1 # init with -1
        for i in range(self.n_camera): # looping through number of cams
            if self.camera_available[i]: # check if cam is available
                camera_id = i
                break # temp break
        if camera_id==-1: # if no camera then just return
            return
        for i in range(self.n_camera):
            self.camera_available[i] = False # reason to make this false? redundant?
        if len(self.camera_pose[camera_id]) > 0: #if list not empty means cam exists
            if self.old_pose is None: # if just init meaning no old pose
                for pose in self.camera_pose[camera_id]:
                    pose_holder.x += pose.x # init x coordinate (adding x to 0 basically)
                    pose_holder.y += pose.y # init y coordinate (adding x to 0 basically)
                    pose_holder.theta += pose.theta # init theta (adding x to 0 basically)
                    cnt = cnt + 1 # number of block crossed + 1
            else:
                for pose in self.camera_pose[camera_id]:
                    if pose_tolerance(pose,self.old_pose): # check for tolerance (redundant hi hai btw)
                        pose_holder.x += pose.x # adding x to old x
                        pose_holder.y += pose.y # adding y to old y
                        pose_holder.theta += pose.theta # adding theta to old theta
                        cnt = cnt + 1 # number of block crossed + 1
            if cnt>0:
                pose_holder.x = pose_holder.x/cnt
                pose_holder.y = pose_holder.y/cnt
                pose_holder.theta = pose_holder.theta/cnt
                pose_holder.x = (pose_holder.x//3)*3
                pose_holder.y = (pose_holder.y//3)*3
                pose_holder.theta = (pose_holder.theta//3)*3
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
                pose_holder.x = mode(self.old_poses_x) # taking mode of old x poses and putting in x
                pose_holder.y = mode(self.old_poses_y) # taking mode of old y poses and putting in y
                pose_holder.theta = mode(self.old_poses_theta) # taking mode of old thetas and putting in theta
                self.publisher.publish(pose_holder)
            for i in range(self.n_camera):
                self.camera_pose[i] = [] # init a 2D list
    
def main():
    n_camera = int(sys.argv[1]) # getting the number of cameras
    n_robots = int(sys.argv[2]) # getting the number of robots
    print(n_camera, n_robots)
    rclpy.init() # init the rclpy
    nodes = []
    try:
        executor = MultiThreadedExecutor(4) # init a 4 multithreadedexecutor
        for i in range(n_robots):
            cnode = CentralNode(n_camera, i+1) # making object of centralnode
            nodes.append(cnode) # need to append check finally for the need
            executor.add_node(cnode) # add the node in the executor object so that when executor does work, all the nodes in executor will spin
        try:
            executor.spin() # spinning the exec object
        finally:
            executor.shutdown() # shutting down the exec object
            for i in range(n_robots):
                nodes[i].destroy_node() # destorying the nodes before the final shutdown of rclpy
    finally:
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()
