import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D


class CameraNode(Node):

    def __init__(self, camera_url, calib_file, map_file):
        super().__init__('minimal_publisher')

        self.camera = camera.Camera(camera_url)
        self.camera._load(calib_file)
        self.camera.load_map(map_file)

        self.camera.video.start()

        self.publishers= []
        for i in range(4):
            self.publishers.append(self.create_publisher(Pose2D, "r_"+ str(i+1), 1))


        timer_period = 0.03

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):


        poses,ids = self.camera.cord_rel_to_marker(self.camera.get_frame())
        msg= Pose2D()

        for pose,id in zip(poses,ids):
            id=id-201
            msg.x=pose[1][0]
            msg.y= pose[1][1]
            msg.theta= pose[0][2]

            self.publishers[id].publish(msg)

def main(args=None):
    # will have to initialize the camera library ?
    # Initialize the rclpy library

    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)

    camera_node.destroy_node()


    rclpy.shutdown()  # Shutdown the ROS client library for Python

if __name__ == '__main__':
    main()










def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
