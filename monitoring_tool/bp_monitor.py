"""
Created on Thu Jul 21 14:15:11 2016

@author: Josh

This is an implementation of the math in Sharon Portney's manuscript.

Notes:
1. The physical range of Hct with measurement error is: -0.05 < Hct < 1.05
2. The physical range of sO2 with measurement error is: -0.1 < sO2 < 1.1

These uncertainties were determined by using measuremenents with up to 5% error
in a Monte Carlo simulation.

There is an overlap in the solution space for certain {R1, R2} vals. When 
multiple solutions arise, simply choosing the one that has a real part within 
the acceptable range is sufficient.

A normal fetal sO2 is 0.85, so in some cases it may be necessary to rule out 
solutions with sO2 > 0.95
"""

# std lib imports
from __future__ import division
import os
import traceback
import types
import math
# anaconda module imports
from PyQt4 import QtGui, QtCore
import numpy as np
import csv
from inspect import getsourcefile
from functools import wraps
import pyqtgraph as pg

import serial
import serial.tools.list_ports

def QTSlotExceptionRationalizer(*args):
    """
    for strange, mostly undocumented reasons, the default behaviour of PyQt
    is to silence all exceptions, which makes the debugging process roughly 
    equivilant in difficulty to trying to speak portugeuse when you don't know 
    any portugeuse. Inserting this decorator on every function is a kludgy way
    to make PyQt catch exceptions as you would except.
    
    DO NOT WRAP ANY METHODS OTHER THAN SLOT HANDLERS WITH THIS DECORATOR, IT
    BREAKS FUNCTION RETURNS!!!
    """
    if len(args) == 0 or isinstance(args[0], types.FunctionType):
        args = []
    @QtCore.pyqtSlot(*args)
    def slotdecorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            try:
                func(*args)
            except:
                print "Uncaught Exception in slot"
                traceback.print_exc()
        return wrapper

    return slotdecorator

class MainWindow(QtGui.QWidget):
    @QTSlotExceptionRationalizer("bool")
    def __init__(self):
        # draw the interface
        self.init_gui()
        self.teensy = Teensy()
        self.update_serial_port_selector()
        self.monitor_running = False
        self.plot_xrange = (0, 50)
        self.plot_yrange = (-300, 300)

    @QTSlotExceptionRationalizer("bool")
    def init_gui(self):
        QtGui.QWidget.__init__(self, parent=None)
        # button for opening dicom directory
        self.trigger_plot = pg.PlotWidget()
        self.trigger_plot.plot([1,2,3,4])
        self.trigger_plot.setXRange(0, 5, padding=0)
        
        self.serial_port_selector = QtGui.QComboBox() 
        
        self.vertical_range_selector = QtGui.QComboBox()
        self.vertical_range_selector.addItems(['+/- 300mmHg', '0 - +150 mmHg'])
        
        self.timescale_selector = QtGui.QComboBox()
        self.timescale_selector.addItems(['5s', '10s', '20s', '40s'])
        
        self.btn_start = QtGui.QPushButton('Start Monitor')
        
        self.btn_rescan = QtGui.QPushButton('Rescan Serial Ports')
        
        self.mean_HR_window = QtGui.QLabel('Mean HR over timescale = 0 BPM')
        self.mean_BP_window = QtGui.QLabel('Mean BP over timescale = 0 mmHg')
        
        layout_mid = QtGui.QHBoxLayout()
        layout_mid.addWidget(self.trigger_plot)
        
        layout_bottom = QtGui.QHBoxLayout()
        layout_bottom.addWidget(QtGui.QLabel('Serial Port:'))
        layout_bottom.addWidget(self.serial_port_selector)
        layout_bottom.addWidget(self.btn_rescan)
        layout_bottom.addWidget(QtGui.QLabel('BP Range:'))
        layout_bottom.addWidget(self.vertical_range_selector)
        layout_bottom.addWidget(QtGui.QLabel('Timescale Range:'))
        layout_bottom.addWidget(self.timescale_selector)
        layout_bottom.addWidget(self.btn_start)  
        
        layout_stats = QtGui.QGridLayout()
        layout_stats.addWidget(self.mean_HR_window, 1, 1)
        layout_stats.addWidget(self.mean_BP_window, 1, 2)
        
        layout_main = QtGui.QVBoxLayout()
        layout_main.addLayout(layout_mid)
        layout_main.addLayout(layout_bottom)
        layout_main.addLayout(layout_stats)
        self.setLayout(layout_main)
        
        self.btn_rescan.pressed.connect(self.update_serial_port_selector) 
        self.btn_start.pressed.connect(self.start_monitor)
    
    @QTSlotExceptionRationalizer("bool")
    def update_serial_port_selector(self):
        self.serial_ports = self.teensy.scan_serial_ports()
        
        self.serial_port_selector.clear()
        for description, port in self.serial_ports.iteritems():
            self.serial_port_selector.addItem(description)
            
    def get_selected_serial_port(self):
        return self.serial_ports[self.serial_port_selector.currentText()]
        
    @QTSlotExceptionRationalizer("bool")
    def update_plot(self):
        sample_values, triggers = self.teensy.get_sensor_values()
        bp_values = 
    
    def change_timescale(self):
        pass
    
    def change_vertical_scale(self):
        pass
        
    @QTSlotExceptionRationalizer("bool")
    def start_monitor(self):
        if not self.monitor_running:
            self.teensy.serial_port = self.get_selected_serial_port()
            self.teensy.start()
            self.timer = QtCore.QTimer(self)
            self.connect(self.timer, QtCore.SIGNAL("timeout()"), self.update_plot)
            self.timer.start(16)
            self.monitor_running = True
        else:
            self.timer.stop()
            self.teensy.stop()
            self.monitor_running = False
        
class Teensy(object):
    """Teensy is the microcontroller that drives the BP triggering unit. While
    it runs it it sends its current sensor levels out over serial. This serial
    communication is strictly one-way.

    The teensy has two codes it sends continuously:
        Trigger sent -> 100000
        Sensor Value -> [0, 65536] value read by the 16 bit ADC every 16 ms
    """
    SERIAL_BAUDRATE = 115200
    
    def __init__(self):
        self.ser = serial.Serial()
        self.serial_port = None
        
    def scan_serial_ports(self):
        self.port_options = {}
        # populate the dropdown menu of serial ports on the form
        for port, description, details in serial.tools.list_ports.comports():
            port = str(port)
            self.port_options[description] = port
        return self.port_options

    def start(self):
        """begin Serial communications with the teensy"""
        if self.ser.isOpen():
            self.ser.close()
        # must be the same baudrate as the one used in Serial.begin in the microcontroller program
        self.ser.baudrate = self.SERIAL_BAUDRATE
        self.ser.timeout = 0# 1 second just in case readline() ever hangs
        self.ser.port = self.serial_port
        self.ser.open()

    def stop(self):
        """end Serial communications with the teensy, freeing the serial port for other uses like reprogramming"""
        if self.ser.isOpen():
            self.ser.close()

    def get_sensor_values(self):
        """read one line of data over serial and parse it"""
        sample_values, triggers = [], [] 
        
        while True:        
            serial_line = self.ser.readline()
            split_values = serial_line.split()
            
            if not len(split_values) > 1: # buffer exhausted
                return sample_values, triggers
                
            sampleval, trigger = split_values
            sampleval = int(sampleval)
            trigger = int(trigger)
            sample_values.append(sampleval)
            triggers.append(trigger)
            
def adc_count_to_bp(adc_count_str):
    VREF = 4.096 # volts
    ADC_COUNTS_MIDPOINT = 0x8000    
    ADC_VOLTAGE_FSR = 3*VREF    
    AMP_GAIN = 727.4706
    CABLE_DRIVER_GAIN = 2
    V_SENSOR_TO_MMHG = 0.00003
    
    adc_count = int(adc_count_str)
    bipolar_adc_count = adc_count - ADC_COUNTS_MIDPOINT
    voltage = (float(bipolar_adc_count) / ADC_COUNTS_MIDPOINT) * ADC_VOLTAGE_FSR
    return voltage / (V_SENSOR_TO_MMHG*AMP_GAIN*CABLE_DRIVER_GAIN)

def main():
    w = 850; h = 370
    app = QtGui.QApplication.instance() or QtGui.QApplication([])
    win = MainWindow()
    win.setWindowTitle('Gating Monitor')
    win.resize(w, h)
    win.show()
    app.exec_()
    return win

if __name__ == '__main__':
    win = main()