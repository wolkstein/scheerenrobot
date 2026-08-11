import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import depthai as dai
import time

class Bridge(Node):

    def __init__(self):
        super().__init__('oak_d_lite')

        self.rate = 30 # 30hz
        self.create_pipeline()
        self.run()

    def create_pipeline(self):
        self.pipeline = dai.Pipeline()

        monoLeft  = self.pipeline.create(dai.node.MonoCamera)
        monoRight = self.pipeline.create(dai.node.MonoCamera)
        xoutLeft  = self.pipeline.create(dai.node.XLinkOut)
        xoutRight = self.pipeline.create(dai.node.XLinkOut)

        xoutLeft.setStreamName('left')
        xoutRight.setStreamName('right')

        monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        monoLeft.out.link(xoutLeft.input)
        monoRight.out.link(xoutRight.input)

        # ros publishers
        self.left_pub = self.create_publisher(Image, '/camera/left/image_raw', 1)
        self.right_pub = self.create_publisher(Image, '/camera/right/image_raw', 1)
        self.left_info_pub = self.create_publisher(CameraInfo, 'left/camera_info', 1)
        self.right_info_pub = self.create_publisher(CameraInfo, 'right/camera_info', 1)

        # bridge
        self.bridge = CvBridge()
        self.left_info = CameraInfo()
        self.right_info = CameraInfo()

        self.get_logger().info("Oak-D-Lite stereo pipeline created")

    def run(self):
        with dai.Device(self.pipeline) as device:
            self.get_logger().info("Device ready for use ...")
            qLeft  = device.getOutputQueue(name="left",  maxSize=1, blocking=False)
            qRight = device.getOutputQueue(name="right", maxSize=1, blocking=False)
            left_counter = 0
            right_counter = 0

            while True:
                start = time.time()
                inLeft  = qLeft.tryGet()
                inRight = qRight.tryGet()
                now = self.get_clock().now().to_msg()

                if inLeft is not None:
                    left_frame = inLeft.getCvFrame()
                    left_frame_ros = self.bridge.cv2_to_imgmsg(left_frame, 'mono8')
                    left_frame_ros.header.stamp = now
                    left_frame_ros.header.frame_id = f'{left_counter}'
                    self.left_info.header.stamp = now
                    self.left_pub.publish(left_frame_ros)
                    self.left_info_pub.publish(self.left_info)
                    left_counter += 1

                if inRight is not None:
                    right_frame = inRight.getCvFrame()
                    right_frame_ros = self.bridge.cv2_to_imgmsg(right_frame, 'mono8')
                    right_frame_ros.header.stamp = now
                    right_frame_ros.header.frame_id = f'{right_counter}'
                    self.right_info.header.stamp = now
                    self.right_pub.publish(right_frame_ros)
                    self.right_info_pub.publish(self.right_info)
                    right_counter += 1

                dt = time.time() - start
                if dt > (1 / self.rate) * 1.1:
                    self.get_logger().warn(f"Loop took too much time = {dt:.3f}s")
                self.get_logger().info("Working ...", throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    bridge = Bridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)