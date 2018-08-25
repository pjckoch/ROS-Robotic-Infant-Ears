#!/usr/bin/env python

import rospy
import numpy as np
import scipy as sp
from scipy import signal
from audio_proc.msg import FFTData, AudioWav
from rospy.numpy_msg import numpy_msg
from audio_common_msgs.msg import AudioDataStampedRaw
from std_msgs.msg import Float32MultiArray, Header


def getFFT(data,srate):
      # apply Hamming Window function
      fft = data*sp.signal.hamming(len(data))
      # our input is real, so use rfft to compute DFT
      fft = sp.fftpack.rfft(data) 
      # remove imaginary part
      fft=np.abs(fft)
      # get the DFT sample frequencies
      freqs=sp.fftpack.rfftfreq(len(fft),1.0/srate)
      # remove the symmetric part of the signal and return
      return fft, freqs


class FourierTransform():
    """the FourierTransform class provides a subscriber and publisher function to transform incoming time-domain signals to spectral-domain and publish both representations."""
    def __init__(self):

        self.fft = []
        self.freqs = []
        self.data = [] # audio data

        rospy.init_node('fft')
        rospy.loginfo("FFT node running")

        self.sample_rate = rospy.get_param("~sample_rate", 48000)
        self.msg_type_audio_common = rospy.get_param("~msg_type_audio_common", False)

        self.pub=rospy.Publisher('fftData', FFTData, queue_size=5)
        self.time_pub_ = rospy.Publisher('fftDuration', Float32MultiArray,
                                         queue_size=10)
        self.subscribe()
 

    def publishFFT(self,msg):
        """callback function that is called everytime a new audio chunk comes in.
        Calls the getFFT function and publishes the FFT data along with the time-domain signal.
        Note that there is no publishing rate specified since the rate is determined by the 
        incoming audio stream."""
        # timing analysis: begin
        callback_begin = rospy.Time.now()
        
        header = Header()
        # copy header of subscribed message
        header.stamp = msg.header.stamp

        self.data = np.frombuffer(msg.data, dtype=np.int16)
        if len(self.data) > 0:
            self.fft, self.freqs = getFFT(self.data, self.sample_rate)

        # timing analysis: end
        callback_end = rospy.Time.now()
        self.pub.publish(header, self.data, self.fft, self.freqs)

        elapsed_proc = callback_end - callback_begin
        elapsed = callback_end - msg.header.stamp

        timearray = [elapsed_proc.to_sec(), elapsed.to_sec()]
        timemsg = Float32MultiArray(data=timearray)


        self.time_pub_.publish(timemsg)


    def subscribe(self):
        """constantly checks for incoming audio chunks"""
        # since different audio drivers can be used, message types may vary
        if self.msg_type_audio_common:
            rospy.Subscriber("audio", AudioDataStampedRaw, self.publishFFT)
        else:
            rospy.Subscriber("audio", AudioWav, self.publishFFT)
        rospy.spin()

 
if __name__=="__main__":
    FourierTransform()
