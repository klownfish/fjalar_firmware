import tkinter as tk
from tkinter.filedialog import askopenfilename
from matplotlib import pyplot as plt
from matplotlib import animation
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from time import time

REFRESH = 5

VIEWRANGE = 120
##################
#Class to display TimeSeries
##################
#
#__init__(root, database, datalists)
#root - TKinter root window
#clock - the clock that will be used
#time_serise - a list with TimeSeries to display
class GenericGraph():
    def __init__(self, root, time_series, width = 7, height = 5):
        self.time_series = time_series
        self.fig = plt.Figure(figsize=(width, height), dpi=100, tight_layout=True)
        self.ax = self.fig.add_subplot(111)
        self.lines = []
        #create a line for every series
        for _ in time_series:
            line, = self.ax.plot([], [])
            self.lines.append(line)

        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.widget = self.canvas.get_tk_widget()
        self.ani = animation.FuncAnimation(
            self.fig, self.update, interval=REFRESH)

    def update(self, _):
        view_min = 0
        view_max = VIEWRANGE
        for i in range(len(self.time_series)):
            self.lines[i].set_data(self.time_series[i][0], self.time_series[i][1])
            if len(self.time_series[i][0])> 0:
                view_max = self.time_series[i][0][-1]
                view_min = view_max - VIEWRANGE
        self.ax.set_xlim(view_min, view_max)
        return self.lines,

###################
#class to display the latest value from a TimeSeries
###################
#
#__init__(self, root, text, value)
#root - tkinter root
#text - displayed before the value
#value - the TimeSeries that the value will be taken from
class TextLastValue(tk.Label):
    def __init__(self, root, text, value, **kwargs):
        self.text = text
        self.stringVar = tk.StringVar()
        self.stringVar.set(text)
        self.value = value
        self.root = root
        self.update()
        super().__init__(root, textvariable = self.stringVar)

    def update(self):
        self.root.after(REFRESH, self.update)
        if len(self.value[0]) == 0:
            return
        if type(self.value[1][-1]) == float:
            self.stringVar.set(self.text + '%.6f' % self.value[1][-1])
        else:
            self.stringVar.set(self.text + '%s' % self.value[1][-1])

class ButtonFile(tk.Button):
    def __init__(self, root, **kwargs):
        self.on_click = kwargs["command"]
        kwargs["command"] = self.on_click2
        super().__init__(root, **kwargs)

    def on_click2(self):
        path = askopenfilename()
        self.on_click(path)

class AltitudeGraph(GenericGraph):
    def __init__(self, root, gw):
        super().__init__(root, [gw.data["altitude"]])
        self.ax.set_ylim(-10, 4000)
        self.ax.set_title("altitude - m")

class VelocityGraph(GenericGraph):
    def __init__(self, root, gw):
        super().__init__(root, [gw.data["velocity"]])
        self.ax.set_ylim(-80, 400)
        self.ax.set_title("velocity - m/s")

class AccelGraph(GenericGraph):
    def __init__(self, root, gw):
        super().__init__(root, [gw.data["az"]])
        self.ax.set_ylim(-50, 200)
        self.ax.set_title("Acceleration - m/s2")

class FlashUsed(tk.Label):
    def __init__(self, root, gw, **kwargs):
        self.stringVar = tk.StringVar()
        self.stringVar.set("flash used:")
        self.root = root
        self.value = gw.data["flashAddress"]
        self.update()
        super().__init__(root, textvariable = self.stringVar)

    def update(self):
        self.root.after(REFRESH, self.update)
        if len(self.value[1]) == 0:
            return
        # 128MiBit
        self.stringVar.set('flash used: %.2f' % (100 * self.value[1][-1] /  16777216) + "%")