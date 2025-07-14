import zmq
import struct
import time
import threading
import signal
import sys
#ARM &GRIPPER P2 leader
# === MSGID&MODEID ===
class MsgID:
    GET_STATE_REQ      = 0x01  #Request a single FrankaArmState
    GET_CONTROL_MODE_REQ   = 0x02  #Ask for active control mode
    SET_CONTROL_MODE_REQ  = 0x03 #Switch to desired mode
    GET_SUB_PORT_REQ   = 0x04 #Query PUB port number
    GRIPPER_COMMAND_REQ = 0x05  #Gripper command todo:add in client

    #Server → Client
    GET_STATE_RESP     = 0x51  #Respond to GET_STATE_REQ with FrankaArmState
    GET_CONTROL_MODE_RESP   = 0x52  #Respond to QUERY_STATE_REQ (1 byte: ControlMode)
    SET_CONTROL_MODE_RESP = 0x53  #Respond to START_CONTROL_REQ (1 byte: status,0 = OK)
    GET_SUB_PORT_RESP  = 0x54  #Respond to GET_SUB_PORT_REQ (2 bytes: port number)

    #Server → Client (error)
    ERROR              = 0xFF   # 1 byte error code
    #error details

class ModeID:
    HUMAN_MODE = 4
    IDLE = 5
    PD_TEST = 6

# === Server ADDR ===
ARM_SUB_ADDR = "tcp://141.3.53.152:52000"
ARM_SERVICE_ADDR =  "tcp://141.3.53.152:52001"      
GRIPPER_SUB_ADDR = "tcp://141.3.53.152:52003" 
GRIPPER_SERVICE_ADDR = "tcp://141.3.53.152:52004"      

context = zmq.Context()

# === 1. sub ===
def sub_thread(name, addr):
    socket = context.socket(zmq.SUB)
    socket.connect(addr)
    socket.setsockopt(zmq.SUBSCRIBE, b"")  # all messages
    while True:
        msg = socket.recv()
        print(f"[{name}] receive {len(msg)} byte")
        #todo:parse msg and print
        #if name == "ArmState": 


threading.Thread(target=sub_thread, args=("ArmState", ARM_SUB_ADDR), daemon=True).start()
threading.Thread(target=sub_thread, args=("GripperState", GRIPPER_SUB_ADDR), daemon=True).start()

# === 2. req ===
def send_service_request(addr, msg_id, payload):
    socket = context.socket(zmq.REQ)
    socket.connect(addr)
    header = struct.pack("BBH", msg_id, 0, len(payload))
    socket.send(header + payload)
    reply = socket.recv()
    print(f"[Service {addr}] receive resp {len(reply)} byte, ID={msg_id}")
    socket.close()
    return reply
#arm service
def send_arm_request(msg_id, payload=b""):
    return send_service_request(ARM_SERVICE_ADDR, msg_id, payload)
#gripper service
def send_gripper_request(msg_id, payload=b""):
    return send_service_request(GRIPPER_SERVICE_ADDR, msg_id, payload)
def signal_handler(sig, frame):
    print("\n[Client] keyboard Ctrl+C, exit...")
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


# === begin test ===
time.sleep(1)
#1.Gripper Move command = 1
print("\n--- Gripper Move ---")
send_gripper_request(MsgID.GRIPPER_COMMAND_REQ, b"\x01")
#2.set Arm ControlMode to HUMAN_MODE
#print("\n--- Set Arm ControlMode to Human_mode ---")
#send_arm_request(MsgID.SET_CONTROL_MODE_REQ, bytes([ModeID.HUMAN_MODE]))

# print("\n--- Set Arm ControlMode to JOINT_VELOCITY (Follower) ---")
# send_arm_request(MsgID.SET_CONTROL_MODE_REQ, bytes([ModeID.JOINT_VELOCITY]))

#3.get arm state
print("\n--- Get Arm State ---")
send_arm_request(MsgID.GET_STATE_REQ)

#4.get current control mode
print("\n--- Get Arm ControlMode ---")
send_arm_request(MsgID.GET_CONTROL_MODE_REQ)


# === keep running ===
while True:
    time.sleep(1)
