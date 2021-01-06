#!/usr/bin/python3
import chart_studio.plotly as py
import plotly.graph_objs as go
import numpy as np
import json
import time


def area_plot(data, labels):
    assert len(data) == len(labels)
    series_len = len(data[0])
    assert all(len(s) == series_len for s in data)

    previous_series = np.zeros(series_len);

    plots = []
    for label, series in zip(labels, data):

        if label:
            area_middle = (series/2. + previous_series)

            # fake plot for label
            plots.append(go.Scatter(
                x=np.arange(series_len),
                y=area_middle,
                mode="text",
                text=([''] * (series_len - 1) + [label]),
                textposition="middle left"))

            # actual area plot
            plots.append(go.Scatter(
                x=np.arange(series_len),
                y=series,
                stackgroup='one'))

        else:
            # transparent area plot
            plots.append(go.Scatter(
                x=np.arange(series_len),
                y=series,
                mode="none",
                fillcolor="rgba(0,0,0,0)",
                stackgroup='one'))

        previous_series += series
    return plots

data = []
for name in ("as", "dbvt"):
    json_data  = json.load(open(f"{name}.json"))
    times = []
    phases=[("start", 0)]
    for i, step_data in enumerate(json_data["data"]):
        phase_name = {
                0: "inicjalizacja",
                1: "aplikacja ciśnienia",
                2: "penetracja",
                3: "koniec"
                }[step_data["phase"]]
        if phases[-1][0] != phase_name:
            phases.append((phase_name, i))
        [internal_step_data] = [child for child in step_data["children"] if child["name"]=="Internal world step"]
        #step_names = [child["name"] for child in internal_step_data["children"]]
        steps_per_point = 0.01 / (1./2500)
        times.append(internal_step_data["total_time"]/steps_per_point) #for child in internal_step_data["children"]])
        #times[-1].append(internal_step_data["total_time"] - sum(times[-1]))
        #step_names.append("other")
        #print(dict(zip(step_names, times[-1])))
    phases = phases[1:]
    #data = np.array(list(zip(*times)))
    #plots = area_plot(data, step_names)
    data.append(times)
data[0] = data[0][:len(data[0])]
size = min(len(data[0]), len(data[1]))
data[0] = data[0][:size]
data[1] = data[1][:size]
print(sum(data[0])/size*65/6)
print(sum(data[1])/size*65/6)
fig = go.Figure(data=[go.Scatter(x=np.arange(size), y=data[0][:size], name="Axis Sweep"),
                      go.Scatter(x=np.arange(size), y=data[1][:size], name="DBVT")])
fig.update_layout(
    xaxis = dict(
        tickmode = 'array',
        tickvals = [i for p, i in phases],
        ticktext = ["  "+p for p, i in phases]
    )
)
fig.update_layout(legend=dict(
    yanchor="top",
    y=0.99,
    xanchor="left",
    x=0.01
    ),
    yaxis_title="Czas wykonania kroku symulacji [ms]",
    xaxis_title="Czas symulacji")
fig.update_xaxes(
    showgrid=True,
    ticklabelposition="outside right",
    ticks="outside",
    tickson="boundaries",
    ticklen=20
)
#fig.update_yaxes(range=[0, 4])
fig.write_image("broadphase.pdf")

fig.show()
