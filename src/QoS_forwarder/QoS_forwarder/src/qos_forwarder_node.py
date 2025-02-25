import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class QoSForwarderNode(Node):

    def __init__(self):
        super().__init__('qos_forwarder_node')

        # QoS profiles
        qos_profile_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_profile_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.subscriptionBatteryState = self.create_subscription(
            BatteryState,
            '/a200_1057/platform/bms/state',
            self.listener_callback_battery_state,
            qos_profile_sub
        )
        self.subscriptionEStopState = self.create_subscription(
            Bool,
            '/a200_1057/platform/emergency_stop',
            self.listener_callback_e_stop_state,
            qos_profile_sub
        )

        # Publishers
        self.publisherBatteryState = self.create_publisher(
            BatteryState,
            '/a200_1057/platform/bms/state_forwarded',
            qos_profile_pub
        )
        self.publisherEStopState = self.create_publisher(
            Bool,
            '/a200_1057/platform/emergency_stop_forwarded',
            qos_profile_pub
        )

    def listener_callback_battery_state(self, msg):
        self.get_logger().info(f'Battery State Listener heard: "{msg.data}"')
        self.publisherBatteryState.publish(msg)

    def listener_callback_e_stop_state(self, msg):
        self.get_logger().info(f'E Stop State Listener heard: "{msg.data}"')
        self.publisherEStopState.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = QoSForwarderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()