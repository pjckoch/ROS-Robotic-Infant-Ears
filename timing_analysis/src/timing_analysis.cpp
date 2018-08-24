//#include "/home/patrick/catkin_ws/src/timing_analysis/include/timing_analysis/timing_analysis.h"
#include <timing_analysis/timing_analysis.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>


void publishDuration(ros::Time stamp_sub, ros::Time process_begin, ros::Time stamp_pub, ros::Publisher pub) {
    std_msgs::Float32MultiArray msg;
    ros::Duration elapsed_proc = stamp_pub - process_begin;
    ros::Duration elapsed = stamp_pub - stamp_sub;
    msg.data.resize(2);
    msg.data[0] = elapsed_proc.toSec();
    msg.data[1] = elapsed.toSec();
    pub.publish(msg);
}
