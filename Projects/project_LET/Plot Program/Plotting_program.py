import serial
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import numpy as np

#pip install [module]

plt.ion()

ser = serial.Serial(
    port='/COM8',
    baudrate=115200
)

delay = 10

Enc_plot = [0]
Run_time_plot = [0]

graph = plt.plot(Run_time_plot, Enc_plot, color = 'g')[0]
plt.ylim(-360,360)
plt.pause(1)

'''
def update(frame):
    global graph

    # creating a new graph or updating the graph
    graph.set_xdata(Run_time_plot)
    graph.set_ydata(Enc_plot)
    plt.xlim(Run_time_plot[0], Run_time_plot[-1])
'''

while(True):
    value = ser.readline()
    StringValue = str(value,'UTF-8')

    Run_Time = StringValue.split("Run Time(s): ")[1].split("Deg:")[0].replace(" ", "")
    Encoder = StringValue.split("Deg: ")[1].split("Motor")[0].replace(" ", "")

    Enc_plot.append((float(Encoder))/6.66667)
    Run_time_plot.append(float(Run_Time))

    #print((float(Encoder))/6.66667)

    #replace old frame every second
    if delay == 0:
        delay = 10
        graph.remove()
    
        graph = plt.plot(Run_time_plot, Enc_plot, color = 'g')[0]
        plt.xlim(Run_time_plot[0], Run_time_plot[-1])
    
        # short pause
        plt.pause(0.25)

    delay -= 1

    print(StringValue)

ser.close()

