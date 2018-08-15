#!/usr/bin/env python

import rospy

from PyQt4 import QtGui,QtCore

import sys
import ui_main
import numpy as np
import scipy as sp
from scipy import signal
import pyqtgraph
from hark_msgs.msg import HarkFFT, HarkFFTVal, HarkWave, HarkWaveVal




class qtSubAndPlot(QtGui.QMainWindow, ui_main.Ui_MainWindow):
    def __init__(self, parent=None):
        rospy.init_node('plotter')
        pyqtgraph.setConfigOption('background', 'w') #before loading widget
        super(qtSubAndPlot,self).__init__(parent)
        self.setupUi(self)
        self.grFFT.plotItem.showGrid(True, True, 0.5)
        self.grPCM.plotItem.showGrid(True, True, 0.5)
        self.maxFFT=0
        self.maxPCM=0
        self.fft=None
        self.fftlen = None
        self.audiowave=None
        self.freqs=None
        self.sample_rate = None
        self.chunk = None
        rospy.Subscriber("HarkFFT_throttle", HarkFFT, self.monitoringCallback)
        rospy.Subscriber("HarkWave_throttle", HarkWave, self.monitoringCallback)
        self.connect(self, QtCore.SIGNAL("changeUI(PyQt_PyObject)"), self.updatePlot)


    def updatePlot(self, msg):
        """
        updates the plot everytime it receives a signal from the subscriber
        """
        if type(msg) == HarkFFT:
            self.fft = np.asarray(msg.src[0].fftdata_real,dtype=np.float32)
            self.fftlen = int(msg.length)
            self.sample_rate = 48000
            self.freqs = sp.fftpack.rfftfreq(self.fftlen, 1.0/self.sample_rate)
        elif type(msg) == HarkWave:
            self.audiowave = np.asarray(msg.src[0].wavedata,dtype=np.int32)
            self.chunk = int(msg.length)
        if not self.audiowave is None and not self.fft is None:
            if not self.fft is None:
                pcmMax=np.max(np.abs(self.audiowave))
            if pcmMax>self.maxPCM:
                self.maxPCM=pcmMax
                self.grPCM.plotItem.setRange(yRange=[-pcmMax,pcmMax])
            if np.max(self.fft)>self.maxFFT:
                self.maxFFT=np.max(np.abs(self.fft))
                self.grFFT.plotItem.setRange(yRange=[0,1])
            self.pbLevel.setValue(1000*pcmMax/self.maxPCM)
            pen=pyqtgraph.mkPen(color='b')
            self.grPCM.plot(np.arange(self.chunk)/float(48000), self.audiowave,pen=pen,clear=True)
            pen=pyqtgraph.mkPen(color='r')
            self.grFFT.plot(self.freqs,2*self.fft/self.maxFFT,pen=pen,clear=True)
    
    def monitoringCallback(self,msg):
        self.emit(QtCore.SIGNAL("changeUI(PyQt_PyObject)"),msg)

    def monitoringCallbackWave(self,msgWAV):
        self.emit(QtCore.SIGNAL("changeUI(PyQt_PyObject)"),msg)

if __name__=="__main__":
    app = QtGui.QApplication(sys.argv)
    form = qtSubAndPlot()
    form.show()
    app.exec_()
