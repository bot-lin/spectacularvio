import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import depthai
import spectacularAI
import json
import threading

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.odom_publisher_ = self.create_publisher(Odometry, 'vio/odom', 10)
        # self.imu_publisher_ = self.create_publisher(Imu, 'vio/imu', 10)
        self.pipeline = depthai.Pipeline()
        self.vio_pipeline = spectacularAI.depthai.Pipeline(self.pipeline)
        self.main()
        # hz = 100
        # timer_period = 1.0 / hz  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

    def main(self):
        with depthai.Device(self.pipeline) as device, \
            self.vio_pipeline.startSession(device) as vio_session:
            while rclpy.ok():
                out = vio_session.waitForOutput()
                data = json.loads(out.asJson())
                odom_msg = Odometry()
                acceleration = data['acceleration']
                angularVelocity = data['angularVelocity']
                status = data['status']
                orientation = data['orientation']
                position = data['position']
                time = data['time']
                velocity = data['velocity']
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = "vio_odom"
                odom_msg.child_frame_id = "vio"
                odom_msg.pose.pose.position.x = position['x']
                odom_msg.pose.pose.position.y = position['y']
                odom_msg.pose.pose.position.z = position['z']
                odom_msg.pose.pose.orientation.x = orientation['x']
                odom_msg.pose.pose.orientation.y = orientation['y']
                odom_msg.pose.pose.orientation.z = orientation['z']
                odom_msg.pose.pose.orientation.w = orientation['w']
                odom_msg.twist.twist.linear.x = velocity['x']
                odom_msg.twist.twist.linear.y = velocity['y']
                odom_msg.twist.twist.linear.z = velocity['z']
                odom_msg.twist.twist.angular.x = angularVelocity['x']
                odom_msg.twist.twist.angular.y = angularVelocity['y']
                odom_msg.twist.twist.angular.z = angularVelocity['z']
                self.odom_publisher_.publish(odom_msg)
                # self.imu_publisher_.publish(imu_msg)
                # self.get_logger().info('------------------------------------------')
                # self.get_logger().info('acceleration: {}'.format(acceleration))
                # self.get_logger().info('angularVelocity: {}'.format(angularVelocity))
                # self.get_logger().info('status: {}'.format(status))
                # self.get_logger().info('orientation: {}'.format(orientation))
                # self.get_logger().info('position: {}'.format(position))
                # self.get_logger().info('time: {}'.format(time))
                # self.get_logger().info('velocity: {}'.format(velocity))



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    # rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
