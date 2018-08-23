#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
import numpy as np
import scipy as sp
from scipy import signal
from audio_proc.msg import FFTData
from rospy.numpy_msg import numpy_msg
from audio_common_msgs.msg import AudioDataStampedRaw

#MINTIME = 10000000
#MAXTIME = 0
#TIMES = []


def getFFT(data,srate):
      #global MINTIME, MAXTIME, TIMES
      #timestart = rospy.get_time()
      
      # apply Hamming Window function
      fft = data*sp.signal.hamming(len(data))

      # our input is real, so use rfft to compute DFT
      fft = sp.fftpack.rfft(data) 

      # remove imaginary part
      fft=np.abs(fft)

      # get the DFT sample frequencies
      freqs=sp.fftpack.rfftfreq(len(fft),1.0/srate)

      """
      # the following block serves for timing analysis
      timeend = rospy.get_time()
      timediff = timeend-timestart
       
      # timing test
      if timediff > 0:
          if timediff > MAXTIME:
              MAXTIME = timediff
          elif timediff < MINTIME and timediff > 0:
              MINTIME = timediff
          TIMES.append(timediff)
      """
      # remove the symmetric part of the signal and return
      return fft, freqs

class FourierTransform():
    """the FourierTransform class provides a subscriber and publisher function to transform incoming time-domain signals to spectral-domain and publish both representations."""
    def __init__(self):

        self.fft=None
        self.freqs=None
        self.data=None # audio data
        self.srate=None # sample rate

        rospy.init_node('fft')
        rospy.loginfo("FFT node running")

        self.pub=rospy.Publisher('fftData', FFTData, queue_size=5)

        self.subscribe()
 

    def publishFFT(self,msg):
        """callback function that is called everytime a new audio chunk comes in.
        Calls the getFFT function and publishes the FFT data along with the time-domain signal.
        Note that there is no publishing rate specified since the rate is determined by the 
        incoming audio stream."""
        
        # create timestamp with time from callback start
        header = Header()
        header.stamp = rospy.Time.now()

        self.data = np.frombuffer(msg.data, dtype=np.int16)
        self.fft, self.freqs = getFFT(self.data, 48000)
        self.pub.publish(header, self.data, self.fft, self.freqs)


    def subscribe(self):
        """constantly checks for incoming audio chunks"""
        rospy.Subscriber("audio", AudioDataStampedRaw, self.publishFFT)
        rospy.spin()

 
if __name__=="__main__":
    FourierTransform()


    """
    timestart = rospy.get_time()
    while len(TIMES) < 1000:
        pass
    timeend = rospy.get_time()
    rospy.signal_shutdown('Scipy Test Over')
    timediff = timeend-timestart
    if len(TIMES) > 0:
        print("\n%s executions of scipy fft were recorded." %len(TIMES)) 
        print("Scipy test duration: %s secs" %timediff)
        print("MIN scipy fft computation time: %s" %MINTIME)
        print("MAX scipy fft computation time: %s" %MAXTIME)
        print("AVERAGE scipy fft computation time: %s" %(sum(TIMES)/len(TIMES)))
    """
