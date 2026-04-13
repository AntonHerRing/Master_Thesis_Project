import serial
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import numpy as np
import time
from datetime import datetime

from tkinter import Tk
from tkinter.filedialog import askopenfilename

#pip install [module]

plt.ion()

# Innit Variables
filePath = "D:\\Dokument\\ZRasberryPiTest\\ES-Lab-Kit\\Software\\Projects\\project_LET\\Plot Program\\Plot_logs\\"

Enc_plot        = [0]
Motor_plot      = [0]
Contr_plot      = [0]
Run_time_plot   = [0]

# Function handling live recording of variables
def Record_Graph():
    # Ping COM9 to see if available
    while True:
        try:
            ser = serial.Serial(port='/COM9', baudrate=115200) #/COM9
            break
        except serial.serialutil.SerialException:
            print("No Connection found") 
        time.sleep(1)

    print("Connected to COM9")

    # Generate file for logging with date and time
    CurrDateTime = str(datetime.now().strftime("%Y-%m-%d %H-%M-%S"))
    fileName = filePath + "log-" + CurrDateTime + ".txt"
    #print(fileName)
    file = open(fileName, 'w')

    # create initial plot
    fig, graph = plt.subplots(2, 2, figsize=(12, 5))
    fig.suptitle('Control System Monitoring')
    graph[0, 0].plot(Run_time_plot, Enc_plot, 'tab:green')
    graph[0, 0].set_title('Encoder Degree')
    graph[0, 1].plot(Run_time_plot, Motor_plot, 'tab:orange')
    graph[0, 1].set_title('Motor Degree')
    graph[1, 0].plot(Run_time_plot, Contr_plot, 'tab:red')
    graph[1, 0].set_title('Target Degree')
    fig.delaxes(graph[1, 1])

    # set plot labels
    for plot in graph.flat:
        plot.set(xlabel='time(s)', ylabel='Degree')

    for plot in graph.flat[:1]:
        plot.label_outer()

    plt.ylim(-360,360)
    #plt.show(block=True)
    plt.pause(1)

    delay = 20
    rotations = 0

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

                #append values to plots
                Enc_plot.append(360*rotations - ((float(Encoder))/6.66667))
                Run_time_plot.append(float(Run_Time))
                Motor_plot.append(float(Motor))
                Contr_plot.append((float(Control)/8.88889)%360)

                # Logg data in the file
                #file = open(fileName, 'a')
                with open(fileName, 'a') as log_file:
                    print("#StartRunTime#" + Run_Time + "#EndRunTime#", file=log_file)
                    print("#StartEnc#" + str(360*rotations - ((float(Encoder))/6.66667)) + "#EndEnc#", file=log_file)
                    print("#StartMotor#" + Motor + "#EndMotor#", file=log_file)
                    print("#StartContr#" + str((float(Control)/8.88889)%360) + "#EndContr#", file=log_file)

                #replace old frame every 2 seconds
                if delay == 0:
                    delay = 20
                    #fig.delaxes(graph[0, 0])

                    #Dynamically update the plots
                    graph[0, 0].plot(Run_time_plot, Enc_plot, 'tab:green')
                    graph[0, 0].set_title('Encoder Degree')
                    graph[0, 1].plot(Run_time_plot, Motor_plot, 'tab:orange')
                    graph[0, 1].set_title('Motor Degree')
                    graph[1, 0].plot(Run_time_plot, Contr_plot, 'tab:red')
                    graph[1, 0].set_title('Target Degree')
                    #fig.delaxes(graph[1, 1])

                    # set plot labels
                    for plot in graph.flat:
                        plot.set(xlabel='time(s)', ylabel='Degree')

                    for plot in graph.flat[:1]:
                        plot.label_outer()     

                    plt.xlim(Run_time_plot[0], Run_time_plot[-1])
                    #plt.show(block=True)
                    # short pause
                    plt.pause(0.25)

                delay -= 1
            else:
                print("Error! Invalid String Input! Expected '42' but got'", extracted,"'")

    ser.close()
    file.close()

#function which initiate the log laoding function 
def log_handler():
    # open log
    Tk().withdraw() # we don't want a full GUI, so keep the root window from appearing
    logname = askopenfilename(initialdir=filePath) # show an "Open" dialog box and return the path to the selected file
    if logname:
        print("Loading file:" + logname)
        load_log(logname)

# Functions that loads and displays the log as a graph
def load_log(logname):
    with open(logname, 'r') as file:
        data = file.read()

        # parse Run time variables
        temp_parse = data.split("#StartRunTime#")
        for var in temp_parse[1:]:
            Run_Time = var.split("#EndRunTime#")[0].replace(" ", "")
            Run_time_plot.append(float(Run_Time))

        # parse Encoder variables
        temp_parse = data.split("#StartEnc#")
        for var in temp_parse[1:]:
            Encoder = var.split("#EndEnc#")[0].replace(" ", "")
            Enc_plot.append(float(Encoder))

        # parse Motor variables
        temp_parse = data.split("#StartMotor#")
        for var in temp_parse[1:]:
            Motor = var.split("#EndMotor#")[0].replace(" ", "")
            Motor_plot.append(float(Motor))

        # parse Control variables
        temp_parse = data.split("#StartContr#")
        for var in temp_parse[1:]:
            Control = var.split("#EndContr#")[0].replace(" ", "")
            Contr_plot.append(float(Control))

        # Load in plot values
        fig, graph = plt.subplots(2, 2, figsize=(12, 5))
        fig.suptitle('Control System Monitoring')
        graph[0, 0].plot(Run_time_plot, Enc_plot, 'tab:green')
        graph[0, 0].set_title('Encoder Degree')
        graph[0, 1].plot(Run_time_plot, Motor_plot, 'tab:orange')
        graph[0, 1].set_title('Motor Degree')
        graph[1, 0].plot(Run_time_plot, Contr_plot, 'tab:red')
        graph[1, 0].set_title('Target Degree')
        fig.delaxes(graph[1, 1])

        # set plot labels
        for plot in graph.flat:
            plot.set(xlabel='time(s)', ylabel='Degree')

        for plot in graph.flat[:1]:
            plot.label_outer()

        plt.ylim(-360,360)
        plt.show(block=True)
        plt.pause(1)
        while True: pass

#Input to change state
State_input = int(input("(1): Record Graph\t(2): Load Graph\n"))

# Main function loop
while True:
    if State_input == 1:
        Record_Graph()
    elif State_input == 2:
        log_handler()
    else:
        print("Error: Incorrect Option! Expected 1 or 2. Got: " + State_input)
        State_input = int(input("(1): Record Graph\t(2): Load Graph\n"))


