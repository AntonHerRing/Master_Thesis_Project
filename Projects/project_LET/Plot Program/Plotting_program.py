import serial
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import numpy as np
import time
from datetime import datetime
import statistics
import math

import sys

from tkinter import Tk
from tkinter.filedialog import askopenfilename, askopenfilenames

import tkinter as tk
from tkinter import ttk

#pip install [module]

plt.ion()

# Innit Variables
#filePath = "D:\\Dokument\\ZRasberryPiTest\\ES-Lab-Kit\\Software\\Projects\\project_LET\\Plot Program\\Plot_logs\\"
filePath = "D:\\Dokument\\ZRasberryPiTest\\ES-Lab-Kit\\Software\\Projects\\project_LET\\Plot Program\\Plot_logs\\LET_120s_rand_dummy\\"
#filePath = "D:\\Dokument\\ZRasberryPiTest\\ES-Lab-Kit\\Software\\Projects\\project_LET\\Plot Program\\Plot_logs\\LET_120s_Control\\"

Enc_plot        = []
Motor_plot      = []
Contr_plot      = []
Run_time_plot   = []

#button choice when starting
State_input = 0

def Ping_Comm():
    while True:
        try:
            ser = serial.Serial(port='/COM9', baudrate=115200) #/COM9
            break
        except serial.serialutil.SerialException:
            print("No Connection found") 
        time.sleep(1)

    print("Connected to COM9")
    return ser

# Function handling live recording of variables
def Record_Graph(ser, time):
    # Ping COM9 to see if available
    '''
    while True:
        try:
            ser = serial.Serial(port='/COM9', baudrate=115200) #/COM9
            break
        except serial.serialutil.SerialException:
            print("No Connection found") 
        time.sleep(1)

    print("Connected to COM9")
    '''
    Run_Time = 0
    Start_time = 0

    # Read Run time at start of recoring
    value = ser.readline()
    StringValue = str(value,'UTF-8')
    while (True):
        if "#-" in StringValue and "-#" in StringValue:
            extracted = StringValue.split("#-")[1].split("-#")[0]
            if extracted == "42" and StringValue.find("#-42-#", 7, len(StringValue)) == -1:
                if StringValue.find("Run Time(s): ") != -1 and StringValue.find("Deg:") != -1:
                    Start_time = StringValue.split("Run Time(s): ")[1].split("Deg:")[0].replace(" ", "")
                if StringValue.find("Deg: ") != -1 and StringValue.find("Motor") != -1:
                    Encoder = StringValue.split("Deg: ")[1].split("Motor")[0].replace(" ", "")
                if StringValue.find("Motor Deg: ") != -1 and StringValue.find("Target") != -1:
                    Motor = StringValue.split("Motor Deg: ")[1].split("Target")[0].replace(" ", "")
                if StringValue.find("Target Deg:") != -1 and StringValue.find("End") != -1:
                    Control = StringValue.split("Target Deg:")[1].split("End")[0].replace(" ", "")
                break
    Run_Time = Start_time
    Run_time_plot.append(float(Run_Time))
    Enc_plot.append(float(Encoder))
    Motor_plot.append(float(Motor))
    Contr_plot.append(float(Control))

    print("First Data Saved")

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

    #plt.ylim(-360,360)
    #plt.show(block=True)
    plt.pause(1)

    delay = 20
    rotations = 0

    # Enter main recording loop
    while((float(Run_Time) - float(Start_time)) <= float(time)):
        value = ser.readline()
        StringValue = str(value,'UTF-8')
        print(StringValue)

        #Only parse the values for plotting if valid ID
        if "#-" in StringValue and "-#" in StringValue:
            extracted = StringValue.split("#-")[1].split("-#")[0]
            if extracted == "42" and StringValue.find("#-42-#", 7, len(StringValue)) == -1:
                #print(StringValue)

                #parse values from print
                if StringValue.find("Run Time(s): ") != -1 and StringValue.find("Deg:") != -1:
                    Run_Time = StringValue.split("Run Time(s): ")[1].split("Deg:")[0].replace(" ", "")
                if StringValue.find("Deg: ") != -1 and StringValue.find("Motor") != -1:
                    Encoder = StringValue.split("Deg: ")[1].split("Motor")[0].replace(" ", "")
                if StringValue.find("Motor Deg: ") != -1 and StringValue.find("Target") != -1:
                    Motor = StringValue.split("Motor Deg: ")[1].split("Target")[0].replace(" ", "")
                if StringValue.find("Target Deg:") != -1 and StringValue.find("End") != -1:
                    Control = StringValue.split("Target Deg:")[1].split("End")[0].replace(" ", "")

                # Keep rotation within 360 degrees
                #if (float(Encoder)) <= -360:
                #    rotations -= 1
                #elif (float(Encoder)) >= 360:
                #    rotations += 1

                #append values to plots
                Enc_plot.append(float(Encoder))
                Run_time_plot.append(float(Run_Time))
                Motor_plot.append(float(Motor))
                Contr_plot.append(float(Control))

                # Logg data in the file
                #file = open(fileName, 'a')
                with open(fileName, 'a') as log_file:
                    print("#StartRunTime#" + Run_Time + "#EndRunTime#", file=log_file)
                    print("#StartEnc#" + str(Encoder) + "#EndEnc#", file=log_file)
                    print("#StartMotor#" + Motor + "#EndMotor#", file=log_file)
                    print("#StartContr#" + str(Control) + "#EndContr#", file=log_file)

                #replace old frame every 2 seconds
                if delay == 0:
                    delay = 20
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
                print("Warning: Skipped Overwridden String")
    print("End Run Time: " + str((float(Run_Time) - float(Start_time))))

    # Get True Statistics values
    while (True):
        value = ser.readline()
        StringValue = str(value,'UTF-8')
        print(StringValue)
        if "##" in StringValue and "##" in StringValue:
            extracted = StringValue.split("##")[1].split("##")[0]
            if extracted == "32" and StringValue.find("##32##", 7, len(StringValue)) == -1:
                #parse values from print
                Mean = 0
                Variance = 0

                if StringValue.find("Mean: ") != -1 and StringValue.find("Variance:") != -1:
                    Mean = StringValue.split("Mean: ")[1].split("Variance:")[0].replace(" ", "")
                if StringValue.find("Variance: ") != -1 and StringValue.find("Standard Deviation:") != -1:
                    Variance = StringValue.split("Variance: ")[1].split("Standard Deviation:")[0].replace(" ", "")
                
                stnd_dev = math.sqrt(float(Variance))

                print("--True Values--")
                print("Average Value: " + str(Mean))

                #Calculate the other one
                print("variance: " + str(Variance))

                #Calculate Stand dev
                print("Standard Deviation: " + str(stnd_dev))

                with open(fileName, 'a') as log_file:
                    print("#StartMean#" + str(Mean) + "#EndMean#", file=log_file)
                    print("#StartVariance#" + str(Variance) + "#EndVariance#", file=log_file)
                    print("#StartStandardDeviation#" + str(stnd_dev) + "#EndStandardDeviation#", file=log_file)
                break
    ser.close()
    file.close()
    print("--Estimated Values--")            
    Analyze_data(Enc_plot)
    #plt.boxplot(Enc_plot)

def ask_record_time():
    window = tk.Tk()
    window.geometry('400x200')
    window.title('Input Record Time')

    time = tk.StringVar()

    #nested button function
    def read_button():
        window.destroy()

    time_label = ttk.Label(window, text='Record time in (s):')
    time_label.pack()

    time_input = ttk.Entry(window, textvariable=time)
    time_input.pack()

    affirm_choice = tk.Button(window, text="OK", width=25, command=lambda: read_button())
    affirm_choice.pack()

    window.mainloop()

    print("Input was: " + str(time.get()))

    return time.get()

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
        first_time = True

        # parse Run time variables
        temp_parse = data.split("#StartRunTime#")
        for var in temp_parse[1:]:
            Run_Time = var.split("#EndRunTime#")[0].replace(" ", "")
            Run_time_plot.append(float(Run_Time))
            if first_time == True:
                first_time = False
                Run_time_plot.append(float(Run_Time))
        first_time = True

        # parse Encoder variables
        temp_parse = data.split("#StartEnc#")
        for var in temp_parse[1:]:
            Encoder = var.split("#EndEnc#")[0].replace(" ", "")
            Enc_plot.append(float(Encoder))
            if first_time == True:
                first_time = False
                Enc_plot.append(float(Encoder))
        first_time = True

        # parse Motor variables
        temp_parse = data.split("#StartMotor#")
        for var in temp_parse[1:]:
            Motor = var.split("#EndMotor#")[0].replace(" ", "")
            Motor_plot.append(float(Motor))
            if first_time == True:
                first_time = False
                Motor_plot.append(float(Motor))
        first_time = True

        # parse Control variables
        temp_parse = data.split("#StartContr#")
        for var in temp_parse[1:]:
            Control = var.split("#EndContr#")[0].replace(" ", "")
            Contr_plot.append(float(Control))
            if first_time == True:
                first_time = False
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

        print("--True Values--")
        # parse Mean
        if data.find("#StartMean#") != -1 and data.find("#EndMean#") != -1:
            Mean = data.split("#StartMean#")[1].split("#EndMean#")[0].replace(" ", "")
            print("Mean: " + str(Mean))

        # parse Variance
        if data.find("#StartVariance#") != -1 and data.find("#EndVariance#") != -1:
            Variance = data.split("#StartVariance#")[1].split("#EndVariance#")[0].replace(" ", "")
            print("Variance: " + str(Variance))

        # parse Stand Deviation 
        if data.find("#StartStandardDeviation#") != -1 and data.find("#EndStandardDeviation#") != -1:
            stnd_dev = data.split("#StartStandardDeviation#")[1].split("#EndStandardDeviation#")[0].replace(" ", "")
            print("stnd_dev: " + str(stnd_dev))
                    
        print("--Estimated Values--")
        Analyze_data(Enc_plot)

        # plt.ylim(-360,360)
        plt.show(block=True)
        plt.pause(1)

        plt.boxplot(Enc_plot)

        #Analyze_data(Enc_plot)
        #while True: pass

def log_mult():
    # open log

    Tk().withdraw() # we don't want a full GUI, so keep the root window from appearing
    logtuple = askopenfilenames(initialdir=filePath) # show an "Open" dialog box and return the path to the selected file
    
    load_avr(logtuple)

# Functions that loads multiple logs and displays their average 
def load_avr(logtuple):
    for log in logtuple:
        with open(log, 'r') as file:
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
            #while True: pass

# Window pop up for choice selection
def select_function():
    window = tk.Tk()
    window.geometry('400x200')
    window.title("Plotting Options:")

    #nested button function
    def read_button(choice):
        global State_input
        State_input = choice

        print(State_input)
        window.destroy()

    #button1 = tk.Button(window, text="(1))", width=25, command=window.destroy)
    button1 = tk.Button(window, text="Record Graph", width=25, command=lambda: read_button(1))
    button2 = tk.Button(window, text="Load Single Graph", width=25, command=lambda: read_button(2))
    #button3 = tk.Button(window, text="Load Average Graph", width=25, command=lambda: read_button(3))
    button1.pack()
    button2.pack()
    #button3.pack()

    window.mainloop()

def loading_screen(time):
    def process_to_load():
        progress.start()

        progress['value'] = 0
        window.update_idletasks()

        ser = Ping_Comm()

        progress['value'] = 100
        window.update_idletasks()  

        progress.stop()
        window.destroy()
        Record_Graph(ser, time)

    window = tk.Tk()
    window.title("Pinging Comms..")

    # Create a progressbar widget
    progress = ttk.Progressbar(window, orient="horizontal", length=300, mode="determinate")
    progress.pack(pady=20)

    # Button to start progress
    window.after(50, process_to_load)
    window.mainloop()

def Analyze_data(data):
    #Calculate Average
    Avr = statistics.mean(data)
    print("Average Value: " + str(Avr))

    #Calculate the other one
    variance = statistics.variance(data)
    print("variance: " + str(variance))

    #Calculate Stand dev
    stnd_dev = statistics.stdev(data)
    print("Standard Deviation: " + str(stnd_dev))

    #Extract max/min values.
    print("Max Value: " + str(max(data)))
    print("Min Value: " + str(min(data)))

    #Print Box plot
    #plt.boxplot(data)

#Input to change state
#State_input = int(input("(1): Record Graph\t(2): Load Graph\n"))

# Main function loop
while True:
    State_input = 0
    select_function()
    print("Test")
    if State_input == 0:
        break
    elif State_input == 1:
        loading_screen(ask_record_time())
    elif State_input == 2:
        log_handler()
    elif State_input == 3:
        log_mult()
    else:
        print("Error: Incorrect Option! Expected 1, 2, or 3. Got: " + str(State_input))

    sys.exit()


