import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
import sys


from .camera import Camera 
class CameraNode(Node):

    def __init__(self, camera_url, calib_file, map_file, cameraID):
        super().__init__('c_'+str(cameraID)) # making a node named c_1/c_2.....
        self.cameraID = cameraID
        self.camera = Camera(camera_url)
        self.camera._load(calib_file) # loading the caliberation file into the camera object of camera.py
        self.camera.load_map(map_file) # loading the map file into the camera object of camera.py
        self.pubs = []
        self.camera.video.start() # starting the video caputure
        
        for i in range(4): # 4 because of 4 robots
            self.pubs.append(self.create_publisher(Pose2D,"c_"+str(self.cameraID) +"/r_"+ str(i+1), 1)) # making topics of /r_1/c_1, /r_1/c_2...


        timer_period = 1/30

        self.timer = self.create_timer(timer_period, self.timer_callback) # creating a timer object which will call timer_callback in 30 Hz

    def timer_callback(self):
        poses,ids = self.camera.cord_rel_to_marker(self.camera.get_frame(),11.8) # passing camera's frame [self.camera.get_frame()] and the size of aruco markers [11.8]     msg= Pose2D()
        for pose,id in zip(poses,ids): # pose is the pose of the robot and id is the ID of the robot
            id=id-201 # because aruco markers start with 200 for our specifications
            msg.x=pose[1][0][0] # getting x coordinate
            msg.y= pose[1][1][0] # getting y coordinate
            msg.theta= pose[0][2][0] # getting theta
            if id[0]<4: # checking if aruco markers are 201/202/203/204 
                self.pubs[id[0]].publish(msg)
        
def main():
    print(sys.argv) # printing arguments
    rclpy.init() # init the rclpy
    camera_node = CameraNode(sys.argv[1],sys.argv[2],sys.argv[3],sys.argv[4]) # making object of CameraNode
    rclpy.spin(camera_node) # spinning the object
    rclpy.shutdown()  # Shutdown the ROS client library for Python

if __name__ == '__main__':
    main()




