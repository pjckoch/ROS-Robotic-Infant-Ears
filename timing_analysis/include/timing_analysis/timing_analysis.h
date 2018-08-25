#ifndef __TIMING_ANALYSIS_H_
#define __TIMING_ANALYSIS_H_

#include <ros/ros.h>


void publishDuration(ros::Time stamp_sub, ros::Time process_begin, ros::Time callback_end, ros::Publisher pub);


#endif /* __TIMING_ANALYSIS_H_ */
