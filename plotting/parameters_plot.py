#!/usr/bin/python3
import chart_studio.plotly as py
import plotly.graph_objs as go
import numpy as np
import json
import yaml
import time
import numpy as np
from math import pi

params = [
        (("dt2000_1", "dt=0.5 ms (2 MHz)"), ("oldbaseline_1", "dt=0.4 ms (2.5 MHz)"), ("dt3000_1", "dt=3.3 ms (3 MHz)")),
        (("u80_1", "80 jednostek na metr"), ("oldbaseline_1", "100 jednostek na metr"), ("u150_1", "150 jednostek na metr")),
        (("m8_1", "8 kg"), ("oldbaseline_1", "10 kg"), ("m12_1", "12 kg")),
        (("i250_1", "250 iteracji"), ("oldbaseline_1", "300 iteracji"), ("i350_1", "350 iteracji")),
    ]
for (j, param_runs) in enumerate(params):
    plots = []
    for i, (run, name) in enumerate(param_runs):
        config = []
        data = []
        with open(f"runs/{run}/output.txt") as f:
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
        print()
        moving_average_width = 100
        q_avrg = sum(q)/len(q)/1e6
        v_avrg = (max(y)-min(y))/(len(y)*dt)
        print(f"name: {name}")
        print(f"average v: {v_avrg}")
        print(f"average q: {q_avrg}")
        moving_avrg_q_points = np.convolve(q, np.ones(moving_average_width), 'valid')/moving_average_width
        s = ('x', 'pentagon', 'star')
        plots.append(go.Scatter(x=[v_avrg], y=[q_avrg], mode='markers',
                                marker=dict(size=20, symbol=s[i]), name=name))
    fig = go.Figure(data=plots)
    fig.update_xaxes(range=[1.07, 1.08])
    fig.update_yaxes(range=[2.4, 3.25])
    fig.update_layout(legend=dict(
        yanchor="top",
        y=0.99,
        xanchor="left",
        x=0.01
        ),
        width=1.5*400,
        height=1.5*300,
        yaxis_title="Oprór [MPa]",
        xaxis_title="Średnia predkość sondy [m/s]")
    n = ["dt", "units", "mass", "iterations"]
    fig.write_image(f"stability_{n[j]}.pdf")
    fig.show()
