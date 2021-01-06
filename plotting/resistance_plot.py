#!/usr/bin/python3
import chart_studio.plotly as py
import plotly.graph_objs as go
import numpy as np
import json
import yaml
import time
import numpy as np
from math import pi

config = []
data = []
with open("output.txt") as f:
    for line in f.readlines():
        try: 
            a, b = line.split()
            data.append((float(a), float(b)))
        except ValueError:
            config.append(line)
            pass
config = yaml.load('\n'.join(config))["config"]
dt = 1./config["simulation"]["small_steps_per_second"]
m = config["probe"]["mass"]
A = pi*config["probe"]["radius"]**2
units_per_m = config["simulation"]["units_per_m"]
assert units_per_m == config["regolith"]["units_per_m"]

while data[0][1] < 0.000001:
    data.pop(0)
y_rescaled, dv_rescaled = zip(*data)
y = np.array(y_rescaled)/units_per_m
dv = np.array(dv_rescaled)/units_per_m
q = dv*m/dt/A
print(f"avrg q: {sum(q)/len(q)/1e6}")
moving_average_width = 100
q_avrg = np.convolve(q, np.ones(moving_average_width), 'valid')/moving_average_width
v_avrg = (max(y)-min(y))/(len(y)*dt)
print(f"average v: {v_avrg}")

fig = go.Figure(data=[go.Scatter(x=q, y=y), go.Scatter(x=q_avrg, y=y)])
fig.update_xaxes(range=[0, 20e6])
#fig.show()
