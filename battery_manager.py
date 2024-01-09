import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class BatteryManager(Node):

    def __init__(self, threshold):
        super().__init__('battery_manager_node')
        self.threshold = threshold
        self.subscription = self.create_subscription(Float32, 'battery_voltage', self.manager, 10)

    def manager(self, msg):
        battery_voltage = msg.data
        self.get_logger().info('Received battery voltage: {}'.format(battery_voltage))
        
        if battery_voltage < self.threshold:
            self.get_logger().warn('Low battery voltage! {}'.format(battery_voltage))
        else:
            self.get_logger().warn('Battery level OK! {}'.format(battery_voltage))

def main(args=None):
    rclpy.init(args=args)
    threshold = 3.0  # Set your threshold value here
    battery_subscriber = BatteryManager(threshold)
    rclpy.spin(battery_subscriber)
    battery_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
