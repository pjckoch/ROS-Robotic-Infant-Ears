#!/usr/bin/env python

"""
The following is a summary of the licenses involved in this project.
Please also refer to the LICENSE folder in this github repository
for full licensing information.

LICENSE SUMMARY:
------------------------------------------
           The MIT License (MIT)

applies to:
- qtSubAndPlot class, Copyright (c) 2016 Scott W Harden
- ui_main, Copyright (c) 2016 Scott W Harden
- PyQtGraph, Copyright (c) 2012 University of North Carolina at Chapel Hill
------------------------------------------
               BSD License

applies to:
- NumPy, Copyright (c) 2005-2018, NumPy Developers.
- rospy, Copyright (c) 2008, Willow Garage, Inc.
------------------------------------------
              GNU GPL License

applies to:
- PyQt4, Copyright (C) 2011 Riverbank Computing Limited
         Note: Redistribution possible under compatible licenses
               (see https://www.gnu.org/licenses/license-list.en.html)
------------------------------------------
"""

import rospy
from PyQt4 import QtGui,QtCore
import sys
import ui_main
import numpy as np
import pyqtgraph
from audio_proc.msg import FFTData


class qtSubAndPlot(QtGui.QMainWindow, ui_main.Ui_MainWindow):
    def __init__(self, parent=None):
        rospy.init_node('plotter')
        pyqtgraph.setConfigOption('background', 'w') #before loading widget
        super(qtSubAndPlot,self).__init__(parent)
        self.setupUi(self)
        self.grFFT.plotItem.showGrid(True, True, 0.5)
        self.grPCM.plotItem.showGrid(True, True, 0.5)
        self.maxFFT = 0
        self.maxPCM = 0
        self.chunk = 0
        self.fft = None
        self.audiowave = None
        self.freqs = None

        self.sample_rate=rospy.get_param("~sample_rate", 48000)

        rospy.Subscriber("fftData_throttle", FFTData, self.monitoringCallback)
        self.connect(self, QtCore.SIGNAL("changeUI(PyQt_PyObject)"),
                     self.updatePlot)

    def updatePlot(self, msg):
        """
        updates the plot everytime it receives a signal from the subscriber
        """
        self.fft = np.asarray(msg.fft,dtype=np.float32)
        self.freqs = np.asarray(msg.freqs,dtype=np.float32)
        self.audiowave = np.asarray(msg.wavedata,dtype=np.int32)
        self.chunk = len(self.audiowave)
        if not self.audiowave is None and not self.fft is None:
            pcmMax=np.max(np.abs(self.audiowave))
            if pcmMax>self.maxPCM:
                self.maxPCM=pcmMax
                self.grPCM.plotItem.setRange(yRange=[-pcmMax,pcmMax])
            if np.max(self.fft)>self.maxFFT:
                self.maxFFT=np.max(np.abs(self.fft))
                self.grFFT.plotItem.setRange(yRange=[0,1])
            self.pbLevel.setValue(1000*pcmMax/self.maxPCM)
            pen=pyqtgraph.mkPen(color='b')
            self.grPCM.plot(np.arange(self.chunk)/float(self.sample_rate), self.audiowave,pen=pen,clear=True)
            pen=pyqtgraph.mkPen(color='r')
            self.grFFT.plot(self.freqs,self.fft/self.maxFFT,pen=pen,clear=True)     
    
    def monitoringCallback(self,msg):
        self.emit(QtCore.SIGNAL("changeUI(PyQt_PyObject)"),msg)


if __name__=="__main__":
    app = QtGui.QApplication(sys.argv)
    form = qtSubAndPlot()
    form.show()
    app.exec_()
