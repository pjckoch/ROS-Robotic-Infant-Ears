#include <timing_analysis/publish_duration.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

void publishDuration(ros::Time begin, ros::Time end, ros::Publisher pub) {
    std_msgs::Float32 msg;
    ros::Duration elapsed = end - begin;
    msg.data = elapsed.toSec();
    pub.publish(msg);
}
