#!/usr/bin/env python

import rospy
import pyaudio
import numpy as np
import threading
from std_msgs.msg import Int32
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
        rospy.init_node('audio_driver')
        self.pub = rospy.Publisher('AudioStream', AudioWav, queue_size=8)
        self.p = pyaudio.PyAudio()
        self.device = rospy.get_param('~device', None)
        self.fs = rospy.get_param('~sample_rate', 48000)
        self.chunk = rospy.get_param('~buffer_size', 4096)
        self.cancel = False  # in case we can't find microphones
        self.run_and_publish()

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
        msg = ('recording from "%s" ' % self.info["name"])
        msg += ('(device %d) ' % self.device)
        msg += ('with %d frames per buffer and ' % self.chunk)
        msg += ('at %d Hz' % self.fs)
        print(msg)

    def close(self):
        """Gently detach from things."""
        rospy.loginfo(" -- sending stream termination command...")
        self.keepRecording = False  # the threads should self-close
        while(self.t.isAlive()):  # wait for all threads to close
            print("waiting for threads to close")
            rospy.sleep(.1)
        self.stream.stop_stream()
        self.p.terminate()

    # STREAM HANDLING

    def stream_readchunk(self):
        """Reads audio buffer and re-launches itself.
        """
        try:
            self.data = np.fromstring(self.stream.read(self.chunk),
                                      dtype=np.uint8)
            # fill message header with current system time
            header = rospy.Time.now()
            # publish the audio message
            self.pub.publish(header, self.data, self.fs, self.chunk)
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
        if self.keepRecording:
            self.stream_thread_new()
        else:
            self.stream.close()
            self.p.terminate()
            rospy.loginfo(" -- stream STOPPED")

    def stream_thread_new(self):
        """Starts a new thread
        """
        self.t = threading.Thread(target=self.stream_readchunk)
        self.t.start()

    def stream_start(self):
        """Adds data to self.data until termination signal
        """
        if not self.cancel:
            print(" -- starting stream")
            self.keepRecording = True
            self.data = np.zeros(self.chunk)
            self.stream = self.p.open(format=pyaudio.paInt16, channels=1,
                                      rate=self.fs, input=True,
                                      input_device_index=self.device,
                                      frames_per_buffer=self.chunk)
            self.stream_thread_new()
        else:
            print("Could not find any mics. "
                  "Please connect a mic and restart audio_driver.")

    def run_and_publish(self):
        """Start stream and publish audio data.
        """
        self.initiate()
        if not self.cancel:
            # start recording
            self.stream_start()
            # keep the node alive
            while not rospy.is_shutdown():
                pass
            self.close()
        else:
            rospy.loginfo("Audio driver could not find working microphone.")


if __name__ == "__main__":

    AudioDriver()
