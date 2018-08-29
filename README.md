# ROS-Robotic-Infant-Ears
## Description
This repository contains the ROS catkin package(s) for the auditory system of a robotic infant which I was working on for my bachelor thesis.

The package audio_proc provides tools for capturing a live audio stream from a microphone, transforming the data from the time- to the spectral domain (FFT) and plotting both domains in a GUI. The package consists of three executable python programs:

- **audio_driver.py**: captures audio stream from microphone
- **audio_fft.py**: performs fast Fourier transform on audio signal
- **audio_plot.py**: plots the signal in time- and spectral domain

Every step (capture, fft, plot) is implemented as a separate ROS node. This enables us to the spread the nodes across distributed system compontens. In my case, the driver runs on a Raspberry Pi Zero W, whereas the other two nodes run on a personal computer.

## How to use

1. `roslaunch audio_proc piAudio.launch`with the parameters that you want to set (see below for a list of parameters)
2. 
