**timing_analysis package:**

This package serves to analyze the timing of a system or node. It requires that all messages of interest contain a header (std_msgs/Header). It will then publish the time that the node took for bare processing, and the elapsed time between when the header of the message that the node subscribed to, was created. This is particularly useful in distributed systems, where you would like to know how fast your node is and how much impact the connection between the system components (e.g. Wifi) has on the overall run-time.

**Compilation of logfile_stats.cpp:** `g++ -std=c++11 logfile_stats.cpp -o logfile_stats`

**Remark:** For accurate measurements in distributed systems, you should make sure that the clocks of the machines are in sync. One possibility to ensure this is to install chrony on each machine (`apt-get install chrony`). After installation, the chrony daemon will start automatically and synchronize the clock of the machine with various internet servers.

**How to use:**
(Remark: variable names are arbitrary)


1. `#include <timing_analysis/timing_analysis.h>` in the node that you would like to analyze (don't forget to add timing_analysis to the build dependencies in the package.xml and to find it in the CMakeLists.txt of your node).
2. Create a `ros::Publisher time_pub_` that will publish the duration. Advertise it under a topic like "myNodeDuration".
3. At the beginning of your callback, obtain the current system time, e.g. `ros::Time callback_begin = ros::Time::now();`
4. At the end of the callback, obtain the current system time `ros::Time callback_end = ros::Time::now();`
5. Pass the timestamp of the message that the node received `msg->header.stamp`, `callback_begin`, `callback_end` and `time_pub_` to the function `publishDuration()` which is contained in timing_analysis.h.
6. The function `publishDuration()` will publish a `std_msgs::Float32MultiArray` which will contain the information:
[bare_processing_time, processing_time+network_latency].
7. You can the start the measurement by echoing the published duration messages into a txt- or log-file by running 
`rostopic echo -p /topic_name > data.txt` on your command line.
8. When you finished your measurement, run the logfile_stats (`./logfile_stats` on command line). You will be asked to specify the full filepath to your log-file (e.g. `/home/myusername/mylogfiles/mynodeduration.log`). Then, the results will be printed to your terminal.
