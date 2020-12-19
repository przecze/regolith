import chart_studio.plotly as py
import plotly.graph_objs as go
import numpy as np


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

plots = area_plot(np.array([[1,2,3,4], [1,4,9,10]]), ['', 'broadphase'])
fig = go.Figure(data=plots)
fig.show()
