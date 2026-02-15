# This script contains functions to create report body for robocommand
from msgs.report_pb2 import *

def HeartbeatMsg(state:str,lat:float,lon:float,speed:float,heading:float,current_task:str):
    """
    Args:
        state (str): [MANUAL,AUTO,KILLED,UNKNOWN]
        lat (float): Latitude
        lon (float): Longitude
        speed (float): speed in m/s
        heading (float): heading
        current_task(str): [UNKNOWN,NONE,NAV_CHANNEL,SPEED_CHALLENGE,OBJECT_DELIVERY,DOCKING,SOUND_SIGNAL]

    Returns:
        Heartbeat message
    """
    states = ["UNKNOWN","KILLED","MANUAL","AUTO"]
    report_states = [RobotState.STATE_UNKNOWN,
                     RobotState.STATE_KILLED,
                     RobotState.STATE_MANUAL,
                     RobotState.STATE_AUTO
                     ]

    tasks =  ['UNKNOWN','NONE','NAV_CHANNEL','SPEED_CHALLENGE','OBJECT_DELIVERY','DOCKING','SOUND_SIGNAL']
    report_tasks = [TaskType.TASK_UNKNOWN,
                    TaskType.TASK_NONE,
                    TaskType.TASK_NAV_CHANNEL,
                    TaskType.TASK_SPEED_CHALLENGE,
                    TaskType.TASK_OBJECT_DELIVERY,
                    TaskType.TASK_DOCKING,
                    TaskType.TASK_SOUND_SIGNAL
                    ]

    # normalize (None -> "UNKNOWN", strip & uppercase)
    s_key = (state or "UNKNOWN").strip().upper()
    t_key = (current_task or "UNKNOWN").strip().upper()

    # find index or default to 0 (UNKNOWN)
    try:
        s_idx = states.index(s_key)
    except ValueError:
        s_idx = 0
    try:
        t_idx = tasks.index(t_key)
    except ValueError:
        t_idx = 0

    # always return a Heartbeat (with UNKNOWN defaults if necessary)
    return Heartbeat(
        state=report_states[s_idx],
        position=LatLng(latitude=float(lat or 0.0), longitude=float(lon or 0.0)),
        spd_mps=float(speed or 0.0),
        heading_deg=float(heading or 0.0),
        current_task=report_tasks[t_idx],
    )

def GatePassMsg(type, lat, lon):
    """
    Args:
        Type (String): ENTRY or EXIT
        lat (float): Latitude
        lon (float): Longitude

    Returns:
        GatePass message
    """
    if type == "ENTRY":    
        return GatePass(
            type=GateType.GATE_ENTRY,
            position=LatLng(latitude=lat, longitude=lon),
        )

    if type == "EXIT":    
        return GatePass(
            type=GateType.GATE_EXIT,
            position=LatLng(latitude=lat, longitude=lon),
        )

    return None

