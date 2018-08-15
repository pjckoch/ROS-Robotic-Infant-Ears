#!/usr/bin/env python

import rospy
import numpy as np
from audio_proc.msg import AudioWav, FFTData
from rospy.numpy_msg import numpy_msg

MINTIME = 100000000
MAXTIME = 0
TIMES = []


def getFFT(data,srate):
      #global MINTIME, MAXTIME, TIMES
      #timestart = rospy.get_time()
      fft=data*np.hamming(len(data)) # apply Hamming window function
      fft=np.fft.fft(data)
      fft=np.abs(fft)
      freqs=np.fft.fftfreq(len(fft),1.0/srate)
      #timeend = rospy.get_time()
      #timediff = timeend-timestart
      """
      # timing test
      if timediff > 0:
          if timediff > MAXTIME:
              MAXTIME = timediff
          elif timediff < MINTIME:
              MINTIME = timediff
          TIMES.append(timediff)
      """
      return fft[:int((len(fft)/2)+1)], freqs[:int((len(freqs)/2)+1)] # symmetric part removed


class FourierTransform():
    def __init__(self):
        self.fft=None
        self.freqs=None  
        self.data=None # audio data
        self.srate=None # sample rate
        rospy.init_node('fft', disable_signals=True)
        rospy.loginfo("setting up fft node")
        self.pub=rospy.Publisher('fftData', FFTData, queue_size=1)
        self.rate=rospy.Rate(100) # in Hz
        self.subscriber()
 
    def publishFFT(self,msg):
        self.fft, self.freqs = getFFT(msg.data, msg.sample_rate)
        if not self.fft is None and not self.freqs is None:
            self.pub.publish(msg.data, self.fft, self.freqs)
    
    def subscriber(self):
        rospy.Subscriber("AudioStream", numpy_msg(AudioWav), self.publishFFT)
        rospy.spin()


if __name__=="__main__":
    FourierTransform()
    """timestart = rospy.get_time()
    while len(TIMES) < 1000:
        pass
    timeend = rospy.get_time()
    rospy.signal_shutdown('Numpy Test Over')
    timediff = timeend-timestart
    if len(TIMES) > 0:
        print("\n%s executions of numpy fft were recorded" %len(TIMES))
        print("Test duration: %s secs" %timediff)
        print("MIN numpy fft computation time: %s" %MINTIME)
        print("MAX numpy fft computation time: %s" %MAXTIME)
        print("AVERAGE numpy fft computation time: %s" %(sum(TIMES)/len(TIMES)))"""
