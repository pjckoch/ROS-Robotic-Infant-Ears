//#include "/home/patrick/catkin_ws/src/timing_analysis/include/timing_analysis/timing_analysis.h"
#include <timing_analysis/timing_analysis.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>


void publishDuration(ros::Time stamp_sub, ros::Time process_begin, ros::Time callback_end, ros::Publisher pub) {
    std_msgs::Float32MultiArray msg;
    ros::Duration elapsed_proc = callback_end - process_begin;
    ros::Duration elapsed = callback_end - stamp_sub;
    msg.data.resize(2);
    msg.data[0] = elapsed_proc.toSec();
    msg.data[1] = elapsed.toSec();
    pub.publish(msg);
}
