import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, String, Int32
from roboboat_2026.api.ivc.ivc_api import ASVServer, ASVClient
from roboboat_2026.util import deviceHelper
from threading import Thread, Lock


PARTNER_IP = '192.168.8.229'  # Use the IP of the Leader Jetson
PORT = 65432
class IVCNode(Node):
    def __init__(self):
        super().__init__('IVC_Node')

        self.boat_name = deviceHelper.boat
        self.ASV_conn = None
        if self.boat_name=="crusader":
            self.ASV_conn = ASVClient(server_ip=PARTNER_IP, port=PORT)
            while (not self.ASV_conn.connect()):
                self.get_logger().warn("Unable to connect to Barco Polo... retry in 1 second")
                time.sleep(1)
            self.get_logger().info("Successfully connected to Barco Polo")
        else:
            self.ASV_conn = ASVServer(port=PORT)
            self.ASV_conn.start()

        # Flags:
        self.running = True

        # Subscribe to message send queue
        self.sender_sub= self.create_subscription(
            String,
            '/ivc/send',
            self.send_callback,
            10
        )   # to send in cli: ros2 topic pub /ivc/send std_msgs/msg/String "{data: 'hello'}"

        # publish message received
        self.receiver_pub = self.create_publisher(
            String,
            '/ivc/receive',
            10
        )

        # LED control
        self.led_pub = self.create_publisher(
            Int32,
            '/led_state',
            10
        )

        self.lisener = self.create_timer(0.2, self.listening_loop) # Check new message at 5 Hz
        
        self.get_logger().info("Initialized IVC")

    def send_callback(self, msg):
        """Send message to target"""
        try:
            self.ASV_conn.send_string(msg.data)

            led_msg = Int32()
            led_msg.data = 1
            self.led_pub.publish(led_msg)

            self.get_logger().info(f"Send: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Exception in send callback {e}")

    def listening_loop(self):
        try:
            message = self.ASV_conn.get_next_message()
            if message is not None:
                msg = String()
                msg.data = message
                self.receiver_pub.publish(msg)
                self.get_logger().info(f"Received: {message}")

                led_msg = Int32()
                led_msg.data = 2
                self.led_pub.publish(led_msg)

        except Exception as e:
            self.get_logger().error(f"Exception in listening loop {e}")

    def destroy_node(self):
        self.get_logger().info("Stop IVC")
        self.ASV_conn.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = IVCNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C received")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



