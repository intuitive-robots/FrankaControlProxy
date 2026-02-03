import zmq
import json

# Create a context
context = zmq.Context()

# Create a SUB (subscriber) socket
subscriber = context.socket(zmq.SUB)

# Connect to the publisher (replace with the correct IP and port)
subscriber.connect("tcp://141.3.53.152:5555")
subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
# Receive and print messages
try:
    while True:
        print("waiting for message")
        message = subscriber.recv_string()  # Receive a message
        data = json.loads(message)
        print(f"Received message: {data}")
        
except KeyboardInterrupt:
    print("Subscriber interrupted.")
finally:
    subscriber.close()
    context.term()
