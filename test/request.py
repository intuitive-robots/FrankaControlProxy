import zmq
import json

# Create a context
context = zmq.Context()

# Create a SUB (subscriber) socket
client = context.socket(zmq.REQ)

# Connect to the publisher (replace with the correct IP and port)
client.connect("tcp://127.0.0.1:5556")
# Receive and print messages

pi = 3.1415926

# _ = input()
target_joint_pose = [0, -pi/4, 0, -3 * pi / 4, 0, pi / 2, pi / 4]
client.send_string(json.dumps(target_joint_pose))
message = client.recv()
print(message)
