# -*- coding: utf-8 -*-
"""
Created on Fri May 21 12:05:14 2021
https://www.open-electronics.org/guest_projects/real-time-data-plotting-of-iot-sensor-using-python/
@author: Saintyves
added:
    JSON Parsing
    More parameters
    send basic control instruction
"""

import time
import math
from collections import deque , defaultdict
import matplotlib.animation as animation
from matplotlib import pyplot as plt
import threading
#from random import randint
from statistics import *
import psutil
import socket
import json
import numpy as np

#localIP     = "10.0.0.3" #local wifi 
localIP     = "192.168.0.100"
localPort   = 9000
#bufferSize  = 1024
bufferSize  = 1024

msgFromServer       = "Hello UDP Client"
#bytesToSend         = str.encode(msgFromServer)
broadcastIP           = "192.168.0.255"
# Create a datagram socket
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)


# Bind to address and ip

UDPServerSocket.bind((localIP, localPort))
#UDPServerSocket.bind(('', localPort))
print("UDP server up and listening")

class DataPlot3:
    def __init__(self, max_entries = 700):
        self.axis_x = deque(maxlen=max_entries)
        self.axis_y1 = deque(maxlen=max_entries)
        self.axis_y2 = deque(maxlen=max_entries)
        self.axis_y3 = deque(maxlen=max_entries)

        self.max_entries = max_entries

        self.buf1=deque(maxlen=5)
        self.buf2=deque(maxlen=5)
        self.buf3=deque(maxlen=5)

     
    def add(self, x, y1,y2,y3):

        self.axis_x.append(x)
        self.axis_y1.append(y1)
        self.axis_y2.append(y2)
        self.axis_y3.append(y3)

        
class DataPlot:
    def __init__(self, max_entries = 700):
        self.axis_x = deque(maxlen=max_entries)
        self.axis_y = deque(maxlen=max_entries)

        self.max_entries = max_entries

        self.buf1=deque(maxlen=5)

     
    def add(self, x, y):

        self.axis_x.append(x)
        self.axis_y.append(y)

class RealtimePlot:
    def __init__(self, axes):
     
        self.axes = axes
        self.lineplot, = axes.plot([], [], "r-")
        self.axes.tick_params(labelright=True)

    def plot(self, dataPlot,ymin,ymax):
        self.lineplot.set_data(dataPlot.axis_x, dataPlot.axis_y)

        self.axes.set_xlim((min(dataPlot.axis_x)), (min(dataPlot.axis_x))+180)
        # ymin = min([min(dataPlot.axis_y), min(dataPlot.axis_y2)])-10
        # ymax = max([max(dataPlot.axis_y), max(dataPlot.axis_y2)])+10
        #ymin = -10
        #ymax = 1100
        self.axes.set_ylim(ymin,ymax)
        self.axes.relim();
        #self.axes.scatter(len(cpu)-1, cpu[-1])
        
class RealtimePlot3:
    def __init__(self, axes,legend_1,legend_2,legend_3):
     
        self.axes = axes

        self.lineplot1, = axes.plot([], [], "r-", label=legend_1)
        self.lineplot2, = axes.plot([], [], "g-", label=legend_2)
        self.lineplot3, = axes.plot([], [], "b-", label=legend_3)
        axes.legend()
        #axes.legend(loc='upper center', shadow=True, fontsize='x-large')
        self.axes.tick_params(labelright=True)
        
    def plot(self, dataPlot,ymin,ymax):
        
        self.lineplot1.set_data(dataPlot.axis_x, dataPlot.axis_y1)
        self.lineplot2.set_data(dataPlot.axis_x, dataPlot.axis_y2)
        self.lineplot3.set_data(dataPlot.axis_x, dataPlot.axis_y3)
         
        self.axes.set_xlim(min(dataPlot.axis_x), min(dataPlot.axis_x)+180)
        #self.axes.set_xticks(np.arange(min(dataPlot.axis_x), min(dataPlot.axis_x)+(dataPlot.max_entries*100)+100, 1000))
        self.axes.set_ylim(ymin,ymax)
        self.axes.relim();
        #self.axes.scatter(len(cpu)-1, cpu[-1])
        
    # def label_line(self,dataPlot):
    #     self.axes.cla()
    #     self.axes.text(max(dataPlot.axis_x), dataPlot.axis_y2[-1]+2, "{}".format(dataPlot.axis_y2[-1]),color="g")
    #     #self.axes.text(len(cpu)-1, cpu[-1]+2, "{}%".format(cpu[-1]))
        
def main():
    fig = plt.figure(figsize=(12,10)) #figsize=(12,6), facecolor='#DEDEDE'
    #plt.title('Plotting Data')
    
    data_time=DataPlot();
   
    # fig, axes = plt.subplots()
    axes1 = fig.add_subplot(311)
    axes1.title.set_text('Angular Position')
    data_Angle = DataPlot();
    dataPlotting_Angle = RealtimePlot(axes1)
    
    axes2 = fig.add_subplot(312)
    axes2.title.set_text('Linear Acceleration')
    data_Acc = DataPlot3();  
    dataPlotting_Accel= RealtimePlot3(axes2,"x","y","sqrt(x^2+y^2)")
    
    axes3 = fig.add_subplot(313)
    axes3.title.set_text('Gyroscope')
    data_Gyr = DataPlot3();
    dataPlotting_Gyr= RealtimePlot3(axes3,"u","v","w")

    plt.subplots_adjust(left=0.1,bottom=0.1,right=0.9,top=0.9,wspace=0.4,hspace=0.4) #see https://www.geeksforgeeks.org/how-to-set-the-spacing-between-subplots-in-matplotlib-in-python/
    #plt.subplot_tool()

    try:
        a=20  #plot refreshed every "a" UDP packet received 
        count=0
        while True:
            count+=1          
            bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
            BufferRCV = bytesAddressPair[0].decode()
            dataJSON = json.loads(BufferRCV)                  
            #data_time.add(count, int(dataJSON["t"])) 
            
            if count % a==0: #plot refreshed every "a" UDP packet received 
                Angle=int(dataJSON["A"])
                data_Angle.add(float(dataJSON["t"])/1000, Angle)            
                dataPlotting_Angle.plot(data_Angle,-10,1100)
                
                # data_Acc.add(float(dataJSON["t"])/1000,#count/a,
                #               float(dataJSON["x"]),
                #               float(dataJSON["y"]),
                #               float(dataJSON["z"])) 
                data_Acc.add(float(dataJSON["t"])/1000,#count/a,
                              float(dataJSON["x"]),
                              float(dataJSON["y"]),
                              np.sqrt((float(dataJSON["x"])*float(dataJSON["x"]))+(float(dataJSON["y"])*float(dataJSON["y"])) ))        
                dataPlotting_Accel.plot(data_Acc,-25,25)
                #dataPlotting_Accel.label_line(data_Acc)
                
                data_Gyr.add(float(dataJSON["t"])/1000,#count/a,
                              float(dataJSON["u"]),
                              float(dataJSON["v"]),
                              float(dataJSON["w"]))        
                dataPlotting_Gyr.plot(data_Gyr,-7,7)
                      
                plt.pause(0.0001)
    except KeyboardInterrupt:
        print('nnKeyboard exception received. Exiting.')       
        #print(data_time.axis_x)
        #print(data_time.axis_y)          
        plt.close()
        #ser.close()
        #exit()

if __name__ == "__main__": main()