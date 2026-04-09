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

delay = 20

rotations = 0

Enc_plot        = [0]
Motor_plot      = [0]
Contr_plot      = [0]
Run_time_plot   = [0]

fig, graph = plt.subplots(2, 2, figsize=(12, 5))
fig.suptitle('Control System Monitoring')
graph[0, 0].plot(Run_time_plot, Enc_plot, 'tab:green')
graph[0, 0].set_title('Encoder Degree')
graph[0, 1].plot(Run_time_plot, Motor_plot, 'tab:orange')
graph[0, 1].set_title('Motor Degree')
graph[1, 0].plot(Run_time_plot, Contr_plot, 'tab:red')
graph[1, 0].set_title('Target Degree')

for plot in graph.flat:
    plot.set(xlabel='time(s)', ylabel='Degree')

for plot in graph.flat:
    plot.label_outer()

plt.ylim(-360,360)
plt.pause(1)


while(True):
    value = ser.readline()
    StringValue = str(value,'UTF-8')

    #Only parse the values for plotting if valid ID
    if "#-" in StringValue and "-#" in StringValue:
        extracted = StringValue.split("#-")[1].split("-#")[0]
        if extracted == "42":
            print(StringValue)

            #parse values from print
            Run_Time = StringValue.split("Run Time(s): ")[1].split("Deg:")[0].replace(" ", "")
            Encoder = StringValue.split("Deg: ")[1].split("Motor")[0].replace(" ", "")
            Motor = StringValue.split("Motor Deg: ")[1].split("Target")[0].replace(" ", "")
            Control = StringValue.split("Target Deg:")[1].split("End")[0].replace(" ", "")

            # Keep rotation within 360 degrees
            if ((float(Encoder))/6.66667) <= -360:
                rotations -= 1
            elif ((float(Encoder))/6.66667) >= 360:
                rotations += 1
            print("Rotations: ", rotations)

            #append values to plots
            Enc_plot.append(360*rotations - ((float(Encoder))/6.66667))
            Run_time_plot.append(float(Run_Time))
            Motor_plot.append(float(Motor))
            Contr_plot.append((float(Control)/8.88889)%360)

            #replace old frame every 2 seconds
            if delay == 0:
                delay = 20
                #fig.delaxes(graph[0, 0])
            
                graph[0, 0].plot(Run_time_plot, Enc_plot, 'tab:green')
                graph[0, 0].set_title('Encoder Degree')
                graph[0, 1].plot(Run_time_plot, Motor_plot, 'tab:orange')
                graph[0, 1].set_title('Motor Degree')
                graph[1, 0].plot(Run_time_plot, Contr_plot, 'tab:red')
                graph[1, 0].set_title('Target Degree')  

                for plot in graph.flat:
                    plot.set(xlabel='time(s)', ylabel='Degree')

                for plot in graph.flat:
                    plot.label_outer()     

                plt.xlim(Run_time_plot[0], Run_time_plot[-1])
            
                # short pause
                plt.pause(0.25)

            delay -= 1
        else:
            print("Invalid String Input")

ser.close()

