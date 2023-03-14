import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import depthai
import spectacularAI
import json

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.odom_publisher_ = self.create_publisher(Odometry, 'vio/odom', 10)
        self.imu_publisher_ = self.create_publisher(Imu, 'vio/imu', 10)
        self.pipeline = depthai.Pipeline()
        self.vio_pipeline = spectacularAI.depthai.Pipeline(self.pipeline)

        hz = 100
        timer_period = 1.0 / hz  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        with depthai.Device(self.pipeline) as device, \
            self.vio_pipeline.startSession(device) as vio_session:
            out = vio_session.waitForOutput()
            data = json.loads(out.asJson())
            odom_msg = Odometry()
            imu_msg = Imu()
            acceleration = data['acceleration']
            angularVelocity = data['angularVelocity']
            status = data['status']
            orientation = data['orientation']
            position = data['position']
            time = data['time']
            velocity = data['velocity']
            self.get_logger().info('------------------------------------------')
            self.get_logger().info('acceleration: {}'.format(acceleration))
            self.get_logger().info('angularVelocity: {}'.format(angularVelocity))
            self.get_logger().info('status: {}'.format(status))
            self.get_logger().info('orientation: {}'.format(orientation))
            self.get_logger().info('position: {}'.format(position))
            self.get_logger().info('time: {}'.format(time))
            self.get_logger().info('velocity: {}'.format(velocity))
            self.odom_publisher_.publish(odom_msg)
            self.imu_publisher_.publish(imu_msg)



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
