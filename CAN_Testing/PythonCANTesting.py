from can.interfaces.serial import SerialBus
from can.message import Message

# Create a CAN bus object
bus = SerialBus(channel='COM5', bustype='serial', baudrate=115200)


# Send a CAN message
message = Message(arbitration_id=0x123, data=[0x01, 0x02, 0x03])
bus.send(message)

# Receive a CAN message
received_message = bus.recv()
print(received_message)
