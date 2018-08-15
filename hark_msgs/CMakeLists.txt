
cmake_minimum_required(VERSION 2.8.3)
project(hark_msgs)

find_package(catkin REQUIRED COMPONENTS roscpp roslib rospy std_msgs message_generation)


add_message_files(
  FILES
  HarkFeature.msg
  HarkFeatureVal.msg
  HarkFFT.msg
  HarkFFTVal.msg
  HarkInt.msg
  HarkJulius.msg
  HarkJuliusSrc.msg
  HarkJuliusSrcVal.msg
  HarkSource.msg
  HarkSourceVal.msg
  HarkSrcFeature.msg
  HarkSrcFeatureMFM.msg
  HarkSrcFeatureMFMVal.msg
  HarkSrcFeatureVal.msg
  HarkSrcFFT.msg
  HarkSrcFFTVal.msg
  HarkSrcWave.msg
  HarkSrcWaveVal.msg
  HarkWave.msg
  HarkWaveVal.msg
)
#add_service_files(
#  FILES
#  HarkIntSrv.srv
#)

generate_messages(
    DEPENDENCIES std_msgs
)

catkin_package(
   DEPENDS roscpp roslib rospy message_runtime
   CATKIN_DEPENDS message_runtime
#    INCLUDE_DIRS # include
#    LIBRARIES
)

