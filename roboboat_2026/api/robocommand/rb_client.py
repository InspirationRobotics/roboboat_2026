from datetime import datetime, timezone
from google.protobuf.timestamp_pb2 import Timestamp
from roboboat_2026.api.robocommand.msgs.report_pb2 import *
from roboboat_2026.api.robocommand.report_body import HeartbeatMsg,GatePassMsg
import socket, struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from std_srvs.srv import Trigger

class RBClient(Node):
    def __init__(self):
        super().__init__('RBClient')
        # General Information
        self.team_id = "ASTA"
        self.vehicle_ids = {"Barco_Polo":123,"Crusader":123}
        
        # Track basic robot information
        self.pos = [] # lat,lon
        self.vel = float() # speed
        self.current_task = str() # current task
        
        # Create Subscribers and Service Servers
        # self.create_subscribers()
        # self.create_services()

        # send loop 
        self.msg_queue = []
        self.report_queue = [] 
        self.organizer = self.create_timer(1.0, self.organize_loop)  # 1Hz for heatbeat
        self.reporter  = self.create_timer(0.2, self.report_loop)  # 5Hz for all msgs
        self.loop_counter = 0

    def __send(self,msg):
        ts = Timestamp(); ts.FromDatetime(datetime.now(timezone.utc))
        msg.sent_at.CopyFrom(ts)

        # Serialize (binary protobuf) and send with 4-byte big-endian length prefix,
        wire = msg.SerializeToString()
        frame = struct.pack("!I", len(wire)) + wire + "test".encode()

        with socket.create_connection(("10.10.10.1", 50000)) as s:
            s.sendall(frame)

    def organize_loop(self): # 1 Hz
        try:
            self.report_queue = [HeartbeatMsg(state="UNKNOWN",lat= 32.00,lon= 31.00,speed=1.0,heading= 1.0,current_task="UNKNOWN")]
            while len(self.report_queue) <= 5 and len(self.msg_queue)>0:
                    self.report_queue.append(self.msg_queue.pop(0))
        except Exception as e:
            self.get_logger().error(f"Error in organize loop. report queue is {self.report_queue}, msg queue is {self.msg_queue}")
            self.get_logger().warn(f"{e}")
  
    def report_loop(self):
        try:
            if len(self.report_queue)>0:
                self.__send(self.report_queue.pop(0))
        except Exception as e:
            self.get_logger().error(f"Error in report loop, report queue is {self.report_queue}")
    
    """All subscribers and services"""
    def create_subscribers(self):
        self.sub_gps = self.create_subscription(
            Float32MultiArray,
            '/GPS',
            self.gps_callback,
            3
        )

        # TODO subscribe to other topics for velocity, robot state, current task

    def create_services(self):
        # TODO create services that contains str or float array ...etc 
        self.srv = self.create_service(
            Trigger,
            'toggle_water_pump',
            self.pump_callback
        )

    """Topic Callbacks"""
    def gps_callback(msg):
         pass
    
    """Service Callbacks"""
    


def main(args=None):
    rclpy.init(args=args)
    node = RBClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C received")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()