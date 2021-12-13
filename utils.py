""" utils.py 

Contains useful function that is used in the LabJack coding examples.
Plotly was chosen over Matplotlib due to its interactive graphs.

Author: Eduardo Nigro
    rev 0.0.1
    2021-12-13
"""
import numpy as np
import plotly.io as pio
import plotly.express as px
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# Setting plotting modified template as default
mytemplate = pio.templates['plotly_white']
mytemplate.layout['paper_bgcolor'] = 'rgb(250, 250, 250)'
pio.templates.default = mytemplate

def plot_line(x, y, xname='Time (s)', yname=None,
              axes='single', marker=False, legend=True):
    # Adjusting inputs
    if type(x) != list: x = [x]
    if type(y) != list: y = [y]
    if not yname:
        yname = ['y'+str(i) for i in range(len(y))]
    elif type(yname) != list:
        yname = [yname]
    # Setting legend display option
    if legend:
        if (len(yname) > 1) and (axes == 'single'):
            showlegend = True
        else:
            showlegend = False
            yname = yname * len(y)
    else:
        showlegend = False
    # Checking for single (with multiple curves)
    # or multiple axes with one curve per axes
    if axes == 'single':
        naxes = 1
        iaxes = [0] * len(y)
        colors = px.colors.qualitative.D3
    elif axes == 'multi':
        naxes = len(y)
        iaxes = range(0, len(y))
        colors = ['rgb(50, 100, 150)'] * len(y)
    # Checking for marker options
    if marker:
        mode = 'lines+markers'
    else:
        mode = 'lines'
    # Setting figure parameters
    m0 = 10
    margin = dict(l=6*m0, r=3*m0, t=3*m0, b=3*m0)
    wfig = 750
    hfig = 100+150*naxes
    # Plotting results
    fig = make_subplots(rows=naxes, cols=1)
    for i, xi, yi, ynamei, color in zip(iaxes, x, y, yname, colors):
        fig.add_trace(go.Scatter(
            x=xi,
            y=yi,
            name=ynamei,
            mode=mode,
            line=dict(
                width=1,
                color=color),
            marker=dict(
                size=2,
                color=color)
        ),
        row=i+1,
        col=1)
        fig.update_yaxes(title_text=ynamei, row=i+1, col=1)
        if xname.lower().find('angle') < 0:
            fig.update_xaxes(
                matches='x',
                row=i+1, col=1)
        else:
            fig.update_xaxes(
                tickmode='array',
                tickvals=np.arange(0,np.round(xi[-1])+360, 360),
                matches='x',
                row=i+1, col=1)

    fig.update_xaxes(title_text=xname, row=i+1, col=1)
    fig.update_layout(
        margin=margin,
        width=wfig,
        height=hfig,
        showlegend=showlegend)
    fig.show()
