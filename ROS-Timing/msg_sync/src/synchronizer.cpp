#include <ros/ros.h>

//#include <boost/bind.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <audio_proc/FFTData.h>
#include <audio_proc/AudioWav.h>
#include <audio_common_msgs/AudioData.h>

#include <image_transport/image_transport.h>

#include <iostream>


using namespace sensor_msgs;
using namespace message_filters;

ros::Duration dur;
ros::Time begin;


class SyncMessages {

public:
  SyncMessages()
    {

    
      ros::NodeHandle nh; 
      image_transport::ImageTransport it_(nh);
      //std::string audio_top;
      //std::string fft_top;
      std::string img_top1;
      std::string img_top2;
      std::string img_top3;
      std::string img_top4;
      std::string img_top5;
      std::string img_top6;
      //std::string img_top7;
      std::string s;
    
      int queue_size_;

      // get parameters from ROS parameter server        
      nh.param<int>("queue_size", queue_size_, 50);

      //nh.param<std::string>("audio_topic", audio_top, "/audio");
      //nh.param<std::sting>("fft_topic", fft_top, "/fftData");
      nh.param<std::string>("img_topic1", img_top1, "/stereo/left/image_raw");
      nh.param<std::string>("img_topic2", img_top2, "/stereo/right/image_raw");
      nh.param<std::string>("img_topic3", img_top3, "/stereo/left/edge_map");
      nh.param<std::string>("img_topic4", img_top4, "/stereo/right/edge_map");
      nh.param<std::string>("img_topic5", img_top5, "/stereo/left/blob");
      nh.param<std::string>("img_topic6", img_top6, "/stereo/right/blob");
      //nh.param<std::string>("img_topic7", img_top7, "/stereo/depth");

      // subscribe to topics
      //audio_sub.subscribe(nh, audio_top, 1);
      //fft_sub.subscribe(nh, fft_top, 1);
      image1_sub.subscribe(nh, img_top1, 1); 
      image2_sub.subscribe(nh, img_top2, 1); 
      image3_sub.subscribe(nh, img_top3, 1); 
      image4_sub.subscribe(nh, img_top4, 1); 
      image5_sub.subscribe(nh, img_top5, 1); 
      image6_sub.subscribe(nh, img_top6, 1);  
      //image7_sub.subscribe(nh, img_top7, 1);

      s = "/sync";

      // advertise publisher topics
      //audio_pub = nh.advertise(audio_top + s, 1);
      //fft_pub = nh.advertise(fft_top + s, 1);
      image_pub1 = it_.advertise(img_top1 + s, 1);
      image_pub2 = it_.advertise(img_top2 + s, 1);
      image_pub3 = it_.advertise(img_top3 + s, 1);
      image_pub4 = it_.advertise(img_top4 + s, 1);
      image_pub5 = it_.advertise(img_top5 + s, 1);
      image_pub6 = it_.advertise(img_top6 + s, 1);
      //image_pub7 = it_.advertise(img_top7 + s, 1);

      // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
      sync_.reset(new Sync(MySyncPolicy(10), image1_sub, image2_sub, image3_sub, image4_sub, image5_sub, image6_sub));
      sync_->registerCallback(boost::bind(&SyncMessages::callback, this, _1, _2, _3, _4, _5, _6));

  }

  void callback(const ImageConstPtr& image1, const ImageConstPtr& image2,
                const ImageConstPtr& image3, const ImageConstPtr& image4,
                const ImageConstPtr& image5, const ImageConstPtr& image6) {   
        // Starts as soon as synchronized messages are available
        std::cout << "Image topics in sync" << std::endl;

        // republish everything in sync
        //audio_pub.publish(audio);
        //fft_pub.publish(fft);
        begin = ros::Time::now();
        image_pub1.publish(image1);
        image_pub2.publish(image2);
        image_pub3.publish(image3);
        image_pub4.publish(image4);
        image_pub5.publish(image5);
        image_pub6.publish(image6);
        //image_pub7.publish(image7);
        dur = ros::Time::now() - begin;
        std::cout << dur.toSec();

    }  

private:
     // image_transport::ImageTransport it_;
     // ros::NodeHandle nh;
      // subscribers
      //message_filters::Subscriber<audio_proc::AudioWav> audio_sub;
      //message_filters::Subscriber<audio_proc::FFTData> fft_sub;
      message_filters::Subscriber<Image> image1_sub; 
      message_filters::Subscriber<Image> image2_sub; 
      message_filters::Subscriber<Image> image3_sub; 
      message_filters::Subscriber<Image> image4_sub; 
      message_filters::Subscriber<Image> image5_sub; 
      message_filters::Subscriber<Image> image6_sub;
      //message_filters::Subscriber<Image> image7_sub;

      // publishers
      //ros::Publisher audio_pub;
      //ros::Publisher fft_pub;
      image_transport::Publisher image_pub1;
      image_transport::Publisher image_pub2;
      image_transport::Publisher image_pub3;
      image_transport::Publisher image_pub4;
      image_transport::Publisher image_pub5;
      image_transport::Publisher image_pub6;
      //image_transport::Publisher img_pub7;
      

      typedef sync_policies::ApproximateTime<Image, Image, Image, Image, Image, Image> MySyncPolicy;
      // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
      typedef Synchronizer<MySyncPolicy> Sync;
      boost::shared_ptr<Sync> sync_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "synchronizer");
  SyncMessages sm;
  ros::spin();
  return 0;
}
