import zmq
import struct
import time
import threading
import signal
import sys
#P2 follower interact
# === MSGID & MODEID ===
class MsgID:
    GET_STATE_REQ = 0x01
    GET_CONTROL_MODE_REQ = 0x02
    SET_CONTROL_MODE_REQ = 0x03
    GET_SUB_PORT_REQ = 0x04
    GRIPPER_COMMAND_REQ = 0x05

    GET_STATE_RESP = 0x51
    GET_CONTROL_MODE_RESP = 0x52
    SET_CONTROL_MODE_RESP = 0x53
    GET_SUB_PORT_RESP = 0x54
    ERROR = 0xFF

class ModeID:
    HUMAN_MODE = 4
    IDLE = 5
    PD_TEST = 6

# === Server ADDR ===
ARM_SUB_ADDR = "tcp://141.3.53.152:51000"
ARM_SERVICE_ADDR = "tcp://141.3.53.152:51001"
GRIPPER_SUB_ADDR = "tcp://141.3.53.152:51003"
GRIPPER_SERVICE_ADDR = "tcp://141.3.53.152:51004"

context = zmq.Context()

# === sub_thread ===
def sub_thread(name, addr):
    socket = context.socket(zmq.SUB)
    socket.connect(addr)
    socket.setsockopt(zmq.SUBSCRIBE, b"")  # all messages
    while True:
        msg = socket.recv()
        # print(f"[{name}] received {len(msg)} bytes")
        #todo:add msg parsing and printing

threading.Thread(target=sub_thread, args=("ArmState", ARM_SUB_ADDR), daemon=True).start()
threading.Thread(target=sub_thread, args=("GripperState", GRIPPER_SUB_ADDR), daemon=True).start()

# === request ===
def send_service_request(addr, msg_id, payload):
    socket = context.socket(zmq.REQ)
    socket.connect(addr)
    header = struct.pack("BBH", msg_id, 0, len(payload))
    socket.send(header + payload)
    reply = socket.recv()
    print(f"[Service {addr}] received {len(reply)} bytes, ID={msg_id}")
    socket.close()
    return reply

def send_arm_request(msg_id, payload=b""):
    return send_service_request(ARM_SERVICE_ADDR, msg_id, payload)

def send_gripper_request(msg_id, payload=b""):
    return send_service_request(GRIPPER_SERVICE_ADDR, msg_id, payload)

# === interact logic ===
def handle_command(cmd: str):
    cmd = cmd.strip().lower()
    if cmd == "exit":
        print("[Client] Exiting...")
        sys.exit(0)

    elif cmd == "gripper_move":
        send_gripper_request(MsgID.GRIPPER_COMMAND_REQ, b"\x01")

    elif cmd == "set_mode pd_test":
        send_arm_request(MsgID.SET_CONTROL_MODE_REQ, bytes([ModeID.PD_TEST]))

    elif cmd == "set_mode idle":
        send_arm_request(MsgID.SET_CONTROL_MODE_REQ, bytes([ModeID.IDLE]))

    elif cmd == "get_arm_state":
        send_arm_request(MsgID.GET_STATE_REQ)

    elif cmd == "get_control_mode":
        send_arm_request(MsgID.GET_CONTROL_MODE_REQ)
    

    else:
        print("[Client] Unknown command.")

# === Ctrl+C ===
def signal_handler(sig, frame):
    print("\n[Client] Keyboard Ctrl+C, exit...")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# === main loop ===
print("=== Franka Client Ready ===")
print("Command List:")
print("  gripper_move")
print("  set_mode pd_test")
print("  set_mode idle")
print("  get_arm_state")
print("  get_control_mode")
print("  exit")
print("===========================\n")

while True:
    try:
        command = input("> ")
        handle_command(command)
    except KeyboardInterrupt:
        print("\n[Client] exiting...")
        break
