# ROS-Robotic-Infant-Ears
## Description
This repository contains the ROS catkin package(s) for the auditory system of a robotic infant which I was working on for my bachelor thesis.

**Note**: the ROS nodes in this repo are also analyzed regarding their timing. Therefore, you will also need the ROS-Timing repo (https://github.com/pjckoch/ROS-Timing.git).

The package audio_proc provides tools for capturing a live audio stream from a microphone, transforming the data from the time- to the spectral domain (FFT) and plotting both domains in a GUI. The package consists of three executable python programs:

- **audio_driver.py**: captures audio stream from microphone
- **audio_fft.py**: performs fast Fourier transform on audio signal
- **audio_plot.py**: plots the signal in time- and spectral domain

Every step (capture, fft, plot) is implemented as a separate ROS node. This enables us to the spread the nodes across distributed system compontens. In my case, the driver runs on a Raspberry Pi Zero W, whereas the other two nodes run on a personal computer. Remember to use a common ROS_MASTER_URI on the different devices.

## How to use
1. `git clone`this repo and the ROS-Timing repo (https://github.com/pjckoch/ROS-Timing.git) into your catkin workspace.
2. `roslaunch audio_proc piAudioStream.launch`on any machine connected to a microphone.
3. `roslaunch audio_proc pcAudioProcessing.launch`on any machine.

**Remark: you can optionally pass parameters to the nodes when calling roslaunch. See below for a list of parameters.**

## ROS parameters
### piAudioStream.launch
- **audio_common**: Set this to `true`if you want to use the audio_common driver to capture sound. This is set to `false`by default, as the audio_common driver does not work on Raspberry Pi (see audio_common issue ticket #100). If you choose to use the audio_common driver, you will need to apply a patch to it, because it does not publish timestamped messages by default. The patch can be found in this repo in the folder "patch"
- **device**: If using the audio_proc driver, PyAudio will give an index to every sound device. If you set device to a valid index, the driver will capture from this device. If you do not set the device parameter or set it to an invalid index, the driver will print out all available input devices and choose the first one automatically. In case, you use the audio_common driver, the device parameter refers to the index that you get when running `arecord -l`from your command line.
- **sample_rate**: Set the sample rate with which you want to capture audio. 

