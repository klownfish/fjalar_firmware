import time

from FjalarParser import FjalarParser
import tkinter as tk
from utils.widgets import *

def main():
    fjalar = FjalarParser("/dev/pts/6")

    root = tk.Tk()
    padding = {"padx": 5, "pady": 5}
    panel = tk.Frame(root)

    altitude = TextLastValue(panel, "altitude: ", fjalar.data["altitude"])
    altitude.grid(row=0,column=0)
    accel = TextLastValue(panel, "acceleration: ", fjalar.data["az"])
    accel.grid(row=1,column=0)
    velocity = TextLastValue(panel, "velocity: ", fjalar.data["velocity"])
    velocity.grid(row=2,column=0)
    state = TextLastValue(panel, "state: ", fjalar.data["flightState"])
    state.grid(row=3,column=0)
    flash = FlashUsed(panel, fjalar)
    flash.grid(row=4,column=0)

    altitude_graph = AltitudeGraph(root, fjalar)
    velocity_graph = VelocityGraph(root, fjalar)
    accel_graph = AccelGraph(root, fjalar)
    altitude_graph.widget.grid(row=0,column=0)
    velocity_graph.widget.grid(row=1,column=0)
    accel_graph.widget.grid(row=0,column=1)

    panel.grid(row=0, column=2, padx=80, pady=80)
    root.mainloop()

main()