import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class QoSForwarderNode(Node):

    def __init__(self):
        super().__init__('qos_forwarder_node')

        # QoS profiles
        qos_profile_sub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_profile_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.subscription1 = self.create_subscription(
            String, # Change to appropriate message type
            'topic1',
            self.listener_callback1,
            qos_profile_sub
        )
        self.subscription2 = self.create_subscription(
            String, # Change to appropriate message type
            'topic2',
            self.listener_callback2,
            qos_profile_sub
        )

        # Publishers
        self.publisher1 = self.create_publisher(
            String,
            'new_topic1',
            qos_profile_pub
        )
        self.publisher2 = self.create_publisher(
            String,
            'new_topic2',
            qos_profile_pub
        )

    def listener_callback1(self, msg):
        self.get_logger().info(f'Listener1 heard: "{msg.data}"')
        self.publisher1.publish(msg)

    def listener_callback2(self, msg):
        self.get_logger().info(f'Listener2 heard: "{msg.data}"')
        self.publisher2.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = QoSForwarderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()