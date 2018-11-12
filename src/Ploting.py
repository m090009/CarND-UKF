nis_laser_file = open("../NIS_data/NIS_laser_vals.txt", "r") 
nis_laser_vals = nis_laser_file.readlines()
nis_radar_file = open("../NIS_data/NIS_radar_vals.txt", "r") 
nis_radar_vals = nis_radar_file.readlines()

import plotly.offline as py
from plotly.graph_objs import *

t = list(range(0,len(nis_laser_vals)))
## we want: 'p1', 'p1est','p1meas', 'p2', 'p2est','p2meas', 'v', 'vest', 'yaw', 'yawest', 'yawrate', 'yawrateest', 'acc', 'yawacc'
#['p1est','p2est','vest','yawest','yawrateest','p1meas','p2meas','p1','p2','v','yaw', 'yawrate','v1_gt','v2_gt']


trace2 = Scatter(
    x=t,
    y=nis_laser_vals,
    xaxis='x2',
    yaxis='y2',
    name = 'NIS laser',
    #mode = 'markers'
)


trace1= Scatter(
    x=[t[0], t[-1]],
    y=[5.991, 5.991],
    xaxis='x2',
    yaxis='y2',
    name = '95 %',
    #mode = 'markers'
)



data = [trace1, trace2]

layout = Layout(
    xaxis2=dict(
   
        anchor='x2',
        title='k'
    ),
    yaxis2=dict(
    
        anchor='y2',
        #title='py'
    )
)

fig = Figure(data=data, layout=layout)
py.iplot(fig, filename= 'EKF')