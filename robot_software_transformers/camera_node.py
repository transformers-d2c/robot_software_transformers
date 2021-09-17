import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
import sys


from .camera import Camera 
class CameraNode(Node):

    def __init__(self, camera_url, calib_file, map_file, cameraID):
        super().__init__('c_'+str(cameraID))
        self.cameraID = cameraID
        self.camera = Camera(camera_url)
        self.camera._load(calib_file)
        self.camera.load_map(map_file)
        self.pubs = []
        self.camera.video.start()
        
        for i in range(4):
            self.pubs.append(self.create_publisher(Pose2D,"c_"+str(self.cameraID) +"/r_"+ str(i+1), 1))


        timer_period = 0.03

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):


        poses,ids = self.camera.cord_rel_to_marker(self.camera.get_frame(),11.8)
        msg= Pose2D()
        for pose,id in zip(poses,ids):
            id=id-201
            msg.x=pose[1][0][0]
            msg.y= pose[1][1][0]
            msg.theta= pose[0][2][0]
            if id[0]<4:
                self.pubs[id[0]].publish(msg)
        
def main():
    # will have to initialize the camera library ?
    # Initialize the rclpy library
    print(sys.argv)
    rclpy.init()
    camera_node = CameraNode(sys.argv[1],sys.argv[2],sys.argv[3],sys.argv[4])
    rclpy.spin(camera_node)
    rclpy.shutdown()  # Shutdown the ROS client library for Python

if __name__ == '__main__':
    main()




