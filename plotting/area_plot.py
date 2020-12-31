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

json_data  = json.load(open("profiler_data.json"))
times = []
for step_data in json_data["data"]:
    #print(step_data)
    [internal_step_data] = [child for child in step_data["children"] if child["name"]=="Internal world step"]
    step_names = [child["name"] for child in internal_step_data["children"]]
    times.append([child["total_time"] for child in internal_step_data["children"]])

data = np.array(list(zip(*times)))

print(step_names)
print(data)
plots = area_plot(data, step_names)
fig = go.Figure(data=plots)
fig.show()
