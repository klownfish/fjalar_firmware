import time as trolled
from FjalarParser import FjalarParser
import tkinter as tk
from utils.widgets import *
import sys

def main():
    fjalar = FjalarParser(sys.argv[1])

    def dump_flash(path):
        f = open(path, "wb")
        max_index = fjalar.data["flashAddress"][1][-1]
        current_index = 0
        while True:
            to_read = min(32, max_index - current_index)
            if (to_read == 0):
                break
            buf = bytes(fjalar.read_flash(current_index, to_read))
            f.write(buf)
            current_index += len(buf)

    root = tk.Tk()
    padding = {"padx": 5, "pady": 5}
    data_panel = tk.Frame(root)
    control_panel = tk.Frame(root)


    altitude = TextLastValue(data_panel, "altitude: ", fjalar.data["altitude"])
    altitude.grid(row=0,column=0)
    accel = TextLastValue(data_panel, "acceleration: ", fjalar.data["az"])
    accel.grid(row=1,column=0)
    velocity = TextLastValue(data_panel, "velocity: ", fjalar.data["velocity"])
    velocity.grid(row=2,column=0)
    state = TextLastValue(data_panel, "state: ", fjalar.data["flightState"])
    state.grid(row=3,column=0)
    state = TextLastValue(data_panel, "sudo: ", fjalar.data["sudo"])
    state.grid(row=4,column=0)
    flash = FlashUsed(data_panel, fjalar)
    flash.grid(row=5,column=0)


    altitude_graph = AltitudeGraph(root, fjalar)
    velocity_graph = VelocityGraph(root, fjalar)
    accel_graph = AccelGraph(root, fjalar)
    altitude_graph.widget.grid(row=0,column=0)
    velocity_graph.widget.grid(row=1,column=0)
    accel_graph.widget.grid(row=0,column=1)


    toggle_sudo = tk.Button(control_panel, text="toggle sudo", command=fjalar.toggle_sudo)
    toggle_sudo.grid(row=0, column=0)
    clear_flash = tk.Button(control_panel, text="clear flash", command=fjalar.clear_flash)
    clear_flash.grid(row=1, column=0)
    enter_idle = tk.Button(control_panel, text="enter idle", command=fjalar.enter_idle)
    enter_idle.grid(row=2, column=0)
    get_ready = tk.Button(control_panel, text="get ready", command=fjalar.ready_up)
    get_ready.grid(row=3, column=0)
    # dump_flash = tk.ButtonFile(control_panel, text="dump flash", command=dump_flash)

    data_panel.grid(row=0, column=2, padx=80, pady=80)
    control_panel.grid(row=1, column=2, padx=80, pady=80)

    # trolled.sleep(3)
    # dump_flash("flash_test.bin")

    root.mainloop()

main()