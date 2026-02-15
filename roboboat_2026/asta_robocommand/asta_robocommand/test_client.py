from datetime import datetime, timezone
from google.protobuf.timestamp_pb2 import Timestamp
from msgs.report_pb2 import *
import socket, struct

# Build message,
msg = Report(
    team_id="Team_Inspiration",
    vehicle_id="Barco_Polo",
    seq=42,
    gate_pass=GatePass(
        type=GateType.GATE_ENTRY,
        position=LatLng(latitude=69.6994736, longitude=-420.452767),
    ),
)
ts = Timestamp(); ts.FromDatetime(datetime.now(timezone.utc))
msg.sent_at.CopyFrom(ts)

# Serialize (binary protobuf) and send with 4-byte big-endian length prefix,
wire = msg.SerializeToString()
frame = struct.pack("!I", len(wire)) + wire + "test".encode()

with socket.create_connection(("10.10.10.1", 50000)) as s:
    s.sendall(frame)
