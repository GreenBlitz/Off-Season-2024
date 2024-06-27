import sys
import time
import tkinter as tk
from tkinter import PhotoImage

# if this library is not installed, dont install ntcore but pyntcore
import ntcore

IMAGE_PATH = "noam-battery-message.png"
WINDOW_NAME = "Battery Message"
TEAM_NUMBER = 4590  # GREENBLITZ 🐐🐐🐐🐐
CLIENT_NAME = "BatteryMessage"
CONNECTION_COOLDOWN_SECONDS = 0.1
CONNECTION_TIMEOUT_SECONDS = 30
TABLE_NAME = "Battery"
KEY_NAME = "is low"


def create_window():
    root = tk.Tk()
    root.title(WINDOW_NAME)
    return root


def load_image(image_path):
    return PhotoImage(file=image_path)


def create_image_label(root, image):
    """Create a label widget to display the image on the given root window."""
    label = tk.Label(root, image=image)
    label.pack()


def terminate_program(root):
    root.quit()


def show_message():
    """Set up and run the Tkinter event loop."""
    root = create_window()
    image = load_image(IMAGE_PATH)
    create_image_label(root, image)
    root.protocol("WM_DELETE_WINDOW", lambda: terminate_program(root))
    root.mainloop()


def get_network_table(IP: str):
    network_table_instance = ntcore.NetworkTableInstance.getDefault()

    print("Setting up NetworkTables client for team {}".format(TEAM_NUMBER))
    network_table_instance.startClient4(CLIENT_NAME)
    network_table_instance.setServer(IP)
    network_table_instance.startDSClient()

    print("Waiting for connection to NetworkTables server...")
    started_time = time.time()
    while not network_table_instance.isConnected():
        # terminate client and program if it takes to long to connect
        if time.time() - started_time > CONNECTION_TIMEOUT_SECONDS:
            close_client(network_table_instance)
            sys.exit()
        time.sleep(CONNECTION_COOLDOWN_SECONDS)

    print("Connected NetworkTables server")
    return network_table_instance


def close_client(network_table_instance: ntcore.NetworkTableInstance):
    network_table_instance.stopDSClient()
    network_table_instance.stopClient()


def start(IP: str):
    network_table = get_network_table(IP)
    # todo - add mult message
    while network_table.isConnected() and not network_table.getTable(TABLE_NAME).getBoolean(KEY_NAME, defaultValue=False):
        time.sleep(1)

    show_message()
    close_client(network_table)