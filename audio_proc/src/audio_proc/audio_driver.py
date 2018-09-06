#!/usr/bin/env python

"""
The following is a summary of the licenses involved in this project.
Please also refer to the LICENSE folder in this github repository
for full licensing information.

LICENSE SUMMARY:
------------------------------------------
           The MIT License (MIT)

applies to:
- AudioDriver class, Copyright (c) 2016 Scott W Harden
- PyAudio, Copyright (c) 2006 Hubert Pham
------------------------------------------
               BSD License

applies to:
- NumPy, Copyright (c) 2005-2018, NumPy Developers.
- rospy, Copyright (c) 2008, Willow Garage, Inc.
- std_msgs, Copyright (c) 2008, Willow Garage, Inc.
------------------------------------------
"""

import rospy
import pyaudio
import numpy as np
import threading
import math
from std_msgs.msg import Int32, Header
from audio_proc.msg import AudioWav

class AudioDriver():
    """The AudioDriver class provides access to continuously recorded
    (and mathematically processed) microphone data.


    ROS parameters:

        device - the index of the sound card input to use. Leave blank
        to automatically detect one.

        fs - sample rate to use. Defaults to something supported.

        buffer_size - also referred to as frames per buffer or chunk.
        Note that resolution is important for spectral analysis.
        (resolution = sample_rate/buffer_size)

    """

    def __init__(self):
        global lock
        lock = threading.Lock()

        rospy.init_node('audio_driver')
        self.pub = rospy.Publisher('audio', AudioWav, queue_size=1)
        self.p = pyaudio.PyAudio()
        self.device = rospy.get_param('~device', 3)
        self.fs = rospy.get_param('~sample_rate', 16000)
        self.chunk = rospy.get_param('~buffer_size', 2048)
        self.pubRate = rospy.get_param('~publish_rate', 100)
        self.cancel = False  # in case we can't find microphones
        self.stepsize = 0
        self.offset = 0
        
        self.buff0 = []
        self.buff1 = []
        self.buff2 = []
        self.twobuff = []
        
        self.get_stepsize()
        self.run()

    #  LOOK FOR VALID INPUT DEVICES

    def valid_rate(self, device, testrate):
        """Check whether the given sample rate is valid for the device.
        Otherwise try with device's devices default sample rate.
        """
        if self.valid_test(device, testrate):
            print("sample rate %d is valid for dev %d" % (testrate, device))
            return testrate
        rospy.loginfo("sample rate %d did not work for dev %d. "
                      "Switching to device's default rate."
                      % (testrate, device))
        if self.valid_test(device, "default"):
            testrate = int(self.info["defaultSampleRate"])
            return testrate
        rospy.loginfo("UNABLE TO FIND VALID SAMPLE RATE FOR DEVICE %d"
                      % (device))
        self.cancel = True
        return None

    def valid_test(self, device, testrate="default"):
        """ Given a device ID and a sample rate,
            return TRUE/False if it's valid.
        """
        try:
            self.info = self.p.get_device_info_by_index(device)
            if not self.info["maxInputChannels"] > 0:
                return False
            elif testrate == "default":
                testrate = int(self.info["defaultSampleRate"])
            stream = self.p.open(format=pyaudio.paInt16, channels=1,
                                 input_device_index=device,
                                 frames_per_buffer=self.chunk,
                                 rate=testrate, input=True)
            stream.close()
            return True
        except Exception as E:
            print(E)
            return False

    def valid_input_devices(self):
        """See which devices can be opened for microphone input.
        call this when no PyAudio object is loaded.
        """
        mics = []
        print("FOLLOWING INPUT DEVICES WERE FOUND:\n")
        for device in range(self.p.get_device_count()):
            if self.valid_test(device):
                mics.append(device)
                msg = ('[Dev: "%s", ' % self.info["name"])
                msg += ('Index: %d, ' % device)
                msg += ('defaultSampleRate: %s]\n'
                        % self.info["defaultSampleRate"])
                print(msg)
        if len(mics) == 0:
            print("no microphone devices found!")
            self.cancel = True
        else:
            print("found %d microphone devices: %s" % (len(mics), mics))
        return mics

    # SETUP AND SHUTDOWN

    def initiate(self):
        """Run this after changing settings (like sample rate) before recording
        """
        if self.device is None:
            mics = self.valid_input_devices()
            if not self.cancel:
                self.device = mics[0]  # pick the first one
        if not self.valid_test(self.device, self.fs):
            print("guessing a valid microphone device/sample rate...")
            mics = self.valid_input_devices()
            if not self.cancel:
                self.device = mics[0]  # pick the first one
                self.fs = self.valid_rate(self.device, self.fs)
        if not self.cancel:
            msg = ('recording from "%s" ' % self.info["name"])
            msg += ('(device %d) ' % self.device)
            msg += ('with %d frames per buffer and ' % self.chunk)
            msg += ('at %d Hz' % self.fs)
            print(msg)

    def close(self):
        """Gently detach from things."""
        rospy.loginfo(" -- sending stream termination command...")
        self.keepRecording = False  # the threads should self-close
        print("waiting for threads to close")
        rospy.sleep(1)
        self.stream.stop_stream()
        self.p.terminate()

    # STREAM HANDLING
    
    def callback(self, in_data, frame_count, time_info, status):
        """Reads audio buffer and gets re-called every time a buffer was returned.
        """
        global lock

        # ensure that publisher thread is currently not reading from twobuff
        lock.acquire()
        try:
            # FIFO: drop oldest buffer
            self.buff0 = self.buff1
            self.buff1 = np.fromstring(in_data,
                                       dtype=np.uint8)
            self.twobuff = np.concatenate([self.buff0, self.buff1])
        except IOError as io: 
            rospy.loginfo("-- exception! terminating audio driver...")
            print("\n\n%s\n\n" % io) 
            if io.strerror == -9981:
                print("Buffer overflow is likely caused by wrongly chosen "
                      "sample_rate and/or buffer_size."
                      "Sometimes even the device's default "
                      "sample_rate does not work. "
                      "e.g. if 44100 Hz was used, "
                      "you may want to try 48000 Hz.\n\n\n")
            self.keepRecording = False
        except Exception as E:
            rospy.loginfo(" -- exception! terminating audio driver...")
            print("\n\n%s\n\n" % E)
            self.keepRecording = False
        finally:
            lock.release()

        if self.keepRecording:
            callback_flag = pyaudio.paContinue

        else:
            self.stream.close()
            self.p.terminate()
            rospy.loginfo(" -- stream STOPPED")
            callback_flag = pyaudio.paComplete
        return (in_data, callback_flag)


    def stream_start(self):
        """Adds data to self.buff2 until termination signal
        """
        if not self.cancel:
            print(" -- starting stream")
            self.keepRecording = True
            self.buff0 = np.zeros(self.chunk)
            self.buff1 = np.zeros(self.chunk)
            self.buff2 = np.zeros(self.chunk)
            self.slideframe = np.ones(self.chunk)
            self.stream = self.p.open(format=pyaudio.paInt16, channels=1,
                                      rate=self.fs, input=True,
                                      input_device_index=self.device,
                                      frames_per_buffer=self.chunk,
                                      stream_callback=self.callback)
            self.stream.start_stream()
        else:
            print("Could not find any mics. "
                  "Please connect a mic and restart audio_driver.")


    def get_stepsize(self):
        """Instead of using a data-triggered publish rate,
           the publish rate shall be constant at e.g. 100.
           To ensure this, a frame slides along an array "twobuff"
           that contains the two latest buffers.
           That way,
           the data that is published is always new (no duplicates) and
           large chunk sizes can be used. The latter is important to
           obtain a high resolution in the spectral domain. This
           function computes the step-size by which the frame slides.
        """
        self.readrate = float(self.fs)/self.chunk
        self.stepsize = float(self.fs)/self.pubRate
        # round the stepsize -> always round down
        # why: otherwise frame could reach upper bound of twobuff before
        # new data is available --> would publish duplicate
        self.stepsize = int(math.floor(self.stepsize))
        print("Sliding Frame step size: %d" % self.stepsize)

    def publishFrame(self):
        """This function publishes the frame that slides along
        the two concatenating buffers.
        """
        global lock

        # to prevent the frame slider from reaching an index out of bounds,
        # we must set the offset index back to zero when it exceeds
        # 1/2 of the twobuff array. Due to the chosen step size, this will
        # also be the point at which a new buffer has been written to the array
        if self.offset >= self.chunk:
            self.offset = 0 
        else:
           self.offset = self.offset + self.stepsize

        # ensure that PyAudio is currently not writing to twobuff
        lock.acquire()
        try:
            # slide frame along array twobuff
            self.slideframe = self.twobuff[self.offset:(self.offset+self.chunk)]
        finally:
            lock.release()
        # fill message header with current system time
        header = Header()
        header.stamp = rospy.Time.now()
        # publish the audio message
        self.slideframe = np.asarray(self.slideframe, dtype=np.uint8)
        self.pub.publish(header, self.slideframe.tolist())
        # to prevent the frame slider from reaching an index out of bounds,
        # we must set the offset index back to zero when it exceeds
        # 1/2 of the twobuff array. Due to the chosen step size, this will
        # also be the point at which a new buffer has been written to the array
        if self.offset >= self.chunk:
            self.offset = 0 
        else:
           self.offset = self.offset + self.stepsize
 
    def run (self):
        """Start stream and publish audio data.
        """
        self.initiate()
        if not self.cancel:
            # start recording
            self.stream_start()
            r = rospy.Rate(110)
            while not rospy.is_shutdown() and self.stream.is_active():
                self.publishFrame()
                r.sleep()
            self.close()
        else:
            rospy.loginfo("Audio driver could not find working microphone.")


if __name__ == "__main__":

    AudioDriver()
