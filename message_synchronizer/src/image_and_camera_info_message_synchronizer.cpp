#include <boost/bind.hpp>
#include <string>

#include <ros/console.h>
#include <ros/param.h>
#include <ros/ros.h>

#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

using namespace std;
using namespace message_filters;

// This node takes care of resynchronizing images and camera
// information to enforce time stamp matching.
// The main interest of this node is being able to realize stereo
// image processing when left and right camera image acquisition
// are not synchronized. This is required for Firewire cameras
// when using camera1394 (in CBoxTurtle release) or usb_cam.
//
// Here is how the can be used, it makes the assumption that a
// stereo camera pair topics are setup in the /stereo prefix and
// that you want to get a synchronized stereo pair in /stereo_sync.
//
// Input:
// - /stereo/left/image_raw
// - /stereo/left/camera_info
// - /stereo/right/image_raw
// - /stereo/right/camera_info
//
// Output:
// - /stereo_sync/left/image_raw
// - /stereo_sync/left/camera_info
// - /stereo_sync/right/image_raw
// - /stereo_sync/right/camera_info
//
// Launch the following command to start the synchronization mode:
// rosrun hueblob fake_camera_synchronizer_node _in:=/stereo _out:=/stereo_sync
//
// Then the stereo image processing node can be started:
// ROS_NAMESPACE=/stereo_sync rosrun stereo_image_proc stereo_image_proc


/// \brief Define policy type for Synchronizer filter.
///
/// This will matches image and camera information for
/// both left and right cameras.



string LEFT_STEREO_IMAGE_INPUT_TOPIC = "/rectify/left/image_rect"; //set topic to subscribe to for left stereo image 
string LEFT_STEREO_CAMERA_INFO_INPUT_TOPIC = "/agnostobot/left/camera_info";  //set topic to subscribe to for left stereo camera info
string RIGHT_STEREO_IMAGE_INPUT_TOPIC = "/rectify/right/image_rect";  //set topic to subscribe to for right stereo image 
string RIGHT_STEREO_CAMERA_INFO_INPUT_TOPIC = "/agnostobot/right/camera_info";  //set topic to subscribe to for left stereo camera info


string LEFT_STEREO_IMAGE_OUPUT_TOPIC = "/stereo/left/image_rect"; //set topic to publish left stereo compressed image 
string LEFT_STEREO_CAMERA_INFO_OUPUT_TOPIC = "/stereo/left/camera_info";  //set topic to publish left stereo camera info
string RIGHT_STEREO_IMAGE_OUPUT_TOPIC = "/stereo/right/image_rect";  //set topic to publish right stereo compressed image 
string RIGHT_STEREO_CAMERA_INFO_OUPUT_TOPIC = "/stereo/right/camera_info";  //set topic to publish left stereo camera info


typedef sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> policy_t;


void callback(image_transport::Publisher& left_stereo_image_publisher,
		   ros::Publisher& left_stereo_camera_info_publisher,
		   image_transport::Publisher& right_stereo_image_publisher,
		   ros::Publisher& right_stereo_camera_info_publisher,
		   const sensor_msgs::ImageConstPtr& left_stereo_image_msg,
		   const sensor_msgs::CameraInfoConstPtr& left_stereo_camera_info_msg,
		   const sensor_msgs::ImageConstPtr& right_stereo_image_msg,
		   const sensor_msgs::CameraInfoConstPtr& right_stereo_camera_info_msg
		   )
{

  left_stereo_image_publisher.publish(left_stereo_image_msg);
  right_stereo_image_publisher.publish(right_stereo_image_msg);
  left_stereo_camera_info_publisher.publish(left_stereo_camera_info_msg);
  right_stereo_camera_info_publisher.publish(right_stereo_camera_info_msg);
}


/// \brief Main entry point.
int main(int argc, char **argv)
{
  //initialize ros node
  ros::init(argc, argv, "image_and_camera_info_message_synchronizer");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  //initialize subscriber
  Subscriber<sensor_msgs::Image> left_stereo_image_subscriber(nh, LEFT_STEREO_IMAGE_INPUT_TOPIC, 100);
  Subscriber<sensor_msgs::CameraInfo> left_stereo_camera_info_subscriber(nh, LEFT_STEREO_CAMERA_INFO_INPUT_TOPIC, 100);
  Subscriber<sensor_msgs::Image> right_stereo_image_subscriber(nh, RIGHT_STEREO_IMAGE_INPUT_TOPIC, 100);
  Subscriber<sensor_msgs::CameraInfo> right_stereo_camera_info_subscriber(nh, RIGHT_STEREO_CAMERA_INFO_INPUT_TOPIC, 100);

  //initialize publishers
  image_transport::Publisher left_stereo_image_publisher = it.advertise(LEFT_STEREO_IMAGE_OUPUT_TOPIC, 100);
  ros::Publisher left_stereo_camera_info_publisher = nh.advertise<sensor_msgs::CameraInfo>(LEFT_STEREO_CAMERA_INFO_OUPUT_TOPIC, 100);
  image_transport::Publisher right_stereo_image_publisher = it.advertise(RIGHT_STEREO_IMAGE_OUPUT_TOPIC, 100);
  ros::Publisher right_stereo_camera_info_publisher = nh.advertise<sensor_msgs::CameraInfo>(RIGHT_STEREO_CAMERA_INFO_OUPUT_TOPIC, 100);


  // Message filter creation.
  Synchronizer<policy_t> sync
    (policy_t(100), left_stereo_image_subscriber, left_stereo_camera_info_subscriber, right_stereo_image_subscriber, right_stereo_camera_info_subscriber);
  
  sync.registerCallback
    (boost::bind(callback,
		 boost::ref(left_stereo_image_publisher), boost::ref(left_stereo_camera_info_publisher),
		 boost::ref(right_stereo_image_publisher), boost::ref(right_stereo_camera_info_publisher),
		 _1, _2, _3, _4));

  ros::spin();
}
