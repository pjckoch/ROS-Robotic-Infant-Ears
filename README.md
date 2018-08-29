# ROS-Robotic-Infant-Ears

## Description
This repository contains the ROS catkin package(s) for the auditory system of a robotic infant which I was working on for my bachelor thesis. Licenses involved in this project are listed in the LICENSE directory.

The repository provides tools for capturing a live audio stream from a microphone, transforming the data from the time- to the spectral domain (FFT) and plotting both domains in a GUI.

The catkin package **audio_proc** consists of three executable python programs:

- [audio_driver.py](audio_proc/src/audio_proc/audio_driver.py): Captures audio stream from microphone and publishes it at 110 Hz.
- [audio_fft.py](audio_proc/src/audio_proc/audio_fft.py): Subscribes to an audio stream and performs the fast Fourier transform of the signal. It publishes the FFT along with the time-domain audio wave.
- [audio_plot.py](audio_proc/src/audio_proc/audio_plot.py): Subscribes to the FFT stream and plots the signal in time- and spectral domain.

Every step (capture, fft, plot) is implemented as a separate ROS node. This enables us to the spread the nodes across distributed system compontens. In my case, the driver runs on a Raspberry Pi Zero W, whereas the other two nodes run on a personal computer. Remember to use a common ROS_MASTER_URI on the different devices. The publish rate of the audio driver is set to 110 Hz to ensure that it is above 100 Hz (fluctuations are normal). The FFT node uses data-triggered publishing and has therefore the same publish rate as the audio driver.

## Prerequisites:
- Clone the [ROS-Timing](https://github.com/pjckoch/ROS-Timing.git) repository
- Install [PyQtGraph](http://pyqtgraph.org/documentation/installation.html)
- Install [NumPy and SciPy](https://www.scipy.org/install.html)
- Install [PyQt4](http://pyqt.sourceforge.net/Docs/PyQt4/installation.html) or [PyQt5](http://pyqt.sourceforge.net/Docs/PyQt5/installation.html)

## How to use
1. `roslaunch audio_proc piAudioStream.launch` on any machine connected to a microphone.
2. `roslaunch audio_proc pcAudioProcessing.launch` on any machine of your choice.

**Remark**: you can optionally pass parameters to the nodes when calling roslaunch. See below for a list of parameters.

## ROS parameters

### piAudioStream.launch:
- **audio_common**: Set this to `true` if you want to use the [audio_common](https://github.com/ros-drivers/audio_common.git) driver to capture sound. This is set to `false` by default, as the audio_common driver does not work on Raspberry Pi (see audio_common issue ticket #100). If you choose to use the audio_common driver, you will need to apply a patch to it, because it does not publish timestamped messages by default. The patch can be found in this repo in the directory "patch".
- **device**: If using the audio_proc driver, PyAudio will give an index to every sound device. If you set device to a valid index, the driver will capture from this device. If you do not set the device parameter or set it to an invalid index, the driver will print out all available input devices and choose the first one automatically. In case, you use the audio_common driver, the device parameter refers to the index that you get when running `arecord -l` from your command line.
- **sample_rate**: Sample rate in Hertz (Hz) with which you want to capture audio. It needs to be valid for the chosen device. If unsure which sample rate is supported, you can leave the device parameter empty and see what the driver prints to your terminal. Besides the device indices, it will print name and default sample rate for each input device.
- **buffer_size**: Only for audio_proc driver. The buffer size is also referred to as frames per buffer or chunk. It specifies how many frames are stored into one buffer. As the FFT is based on the Cooley-Tukey algorithm, performance is best if the buffer size is a power of two. Note that large values (>= 1024) result in higher spectral resolution. However if the value is very large (>= 8192), the plot might respond very slowly. A good trade-off is a value of 2048.
- **fft_at_robot**: Set this to `true` if you want to perform the FFT directly on your robot. Keep in mind that it might run slower than on a PC.
- **depth**: Only for audio_common driver. Specifies the bit depth.

- **channels**: Only for audio_common driver. Number of channels to use. The audio_proc driver supports only 1 channel.

### pcAudioProcessing.launch:
**Remark**: If you set parameters in piAudioStream.launch you might need to set them again when launching pcAudioProcessing.launch. This is due to the fact that roslaunch is unable to read existing parameters from the ROS parameter server. The parameters which need to be set again are italicized. Note that you are responsible for setting the parameter correctly. Otherwise errors may occur. If you know that you are going to permanently use a value different from the parameter's default value, it is recommendable to change the default value in the two launch-files.

- ***audio_common***
- ***fft_at_robot***
- ***sample_rate***
- **plot**: Set this parameter to `false` if you do not want to plot the signal with audio_plot.py.
- **bandwidth**: In case you want to plot the signal, the GUI will not be able to keep up with the publish rate of 110 Hz. Therefore a throttle node needs to be started which reduces the bandwidth for the plot node. The throttled topic will be separate from the original topic. Recommended bandwidth is 30 Hz.

## License

This project is licensed under the 3-Clause-BSD-License (see the [LICENSE.md](LICENSE/LICENSE.md) for details). For third-party licenses, see [LICENSE-3RD-PARTY.md](LICENSE/LICENSE-3RD-PARTY.md).
