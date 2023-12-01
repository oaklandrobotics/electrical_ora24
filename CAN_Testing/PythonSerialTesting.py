import tkinter as tk
from can.interfaces.serial import SerialBus
from can.message import Message

class SerialGUICAN:
    def __init__(self, master):
        self.master = master
        master.title("Serial and CAN Control GUI")

        # Serial setup
        self.serial_port = None  # You can add your serial port configuration here

        # CAN setup
        self.can_bus = SerialBus(channel='COM5', bustype='serial', baudrate=115200)

        self.label_serial = tk.Label(master, text="Send 1 or 0 to Serial:")
        self.label_serial.pack()

        self.button_1 = tk.Button(master, text="Send 1 to Serial", command=lambda: self.send_to_serial(1))
        self.button_1.pack()

        self.button_0 = tk.Button(master, text="Send 0 to Serial", command=lambda: self.send_to_serial(0))
        self.button_0.pack()

        self.label_can = tk.Label(master, text="Send CAN Message:")
        self.label_can.pack()

        self.button_send_can = tk.Button(master, text="Send CAN Message", command=self.send_can_message)
        self.button_send_can.pack()

        self.quit_button = tk.Button(master, text="Quit", command=master.quit)
        self.quit_button.pack()

    def send_to_serial(self, value):
        try:
            # Add your serial sending logic here
            print(f"Sent {value} to serial.")
        except Exception as e:
            print(f"Error sending to serial: {e}")

    def send_can_message(self):
        try:
            # Send a CAN message
            message = Message(arbitration_id=0x123, data=[0x01, 0x02, 0x03])
            self.can_bus.send(message)
            print("Sent CAN message.")
        except Exception as e:
            print(f"Error sending CAN message: {e}")

if __name__ == "__main__":
    root = tk.Tk()
    gui = SerialGUICAN(root)
    root.mainloop()
