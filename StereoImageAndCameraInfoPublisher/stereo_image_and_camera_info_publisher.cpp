//Standard libraries
#include <iostream>
#include <algorithm>
#include <fstream>
#include <string>
#include <thread>
#include <cstdlib>
#include <mutex>
#include <atomic>
#include <unistd.h>
#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

//3rd party libraries
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <raspicam/raspicam_cv.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "yaml-cpp/yaml.h"

//DAPrototype source files
#include "pace_setter_class.cpp"

/*
usage
cd /opt/agnostobot/StereoImageAndCameraInfoPublisher
g++ stereo_image_and_camera_info_publisher.cpp -o  stereo_image_and_camera_info_publisher -I/usr/local/include/ -I/opt/ros/kinetic/include/ -L/opt/ros/kinetic/lib -L/opt/vc/lib -L/usr/local/lib -Wl,-rpath,/opt/ros/kinetic/lib -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util -lopencv_core -lopencv_highgui -lroscpp -lrosconsole -lrostime -lroscpp_serialization -limage_transport -lcv_bridge -lboost_date_time -lboost_system -lboost_thread -pthread -lactionlib -lyaml-cpp -std=c++11 
./stereo_image_and_camera_info_publisher

//force stop
ps -aux | grep stereo_image_and_camera_info_publisher
kill -9 pid

//convert ros image messaages to files
rosrun image_view extract_images image:=/stereo/left/image_raw
rosrun image_view extract_images image:=/stereo/right/image_raw
//or simply echo on console
rostopic echo /stereo/left/image_raw --noarr
rostopic echo /stereo/right/image_raw --noarr
rostopic echo /stereo/left/camera_info
rostopic echo /stereo/right/camera_info
*/

using namespace std;
using namespace cv;

string LEFT_STEREO_IMAGE_TOPIC = "/stereo/left/image_raw"; //set topic to publish left stereo image and camera info 
string RIGHT_STEREO_IMAGE_TOPIC = "/stereo/right/image_raw";  //set topic to publish right stereo image and camera info

int image_capture_fps=10; //set number of frames per second to capture and save to file
int publish_fps=10; //set number of frames per second to publish

int image_width=640; //set video width
int image_height=480; //set video height
int image_rotation_angle=180; //set rotation angle, valid values are 0, 90, 180 and 270
int camera_exposure=60; //set camera exposure, valid values 0-100 or -1

string left_pi_camera_calibration_file = "left.yaml"; //set the file that stores the calibration info for the left pi camera
string right_pi_camera_calibration_file = "right.yaml"; //set the file that stores the calibration info for the right pi camera


bool VERBOSE=false; //set to True if you want to enable logging
bool SAVE_TO_IMAGE_FILES=false; //set to True if you want to save images to file
bool PUBLISH_TO_ROS=true; //set to True if you want to publish video to ros TOPIC


image_transport::CameraPublisher left_stereo_image_publisher;
image_transport::CameraPublisher right_stereo_image_publisher;

//camera_info_manager::CameraInfoManager cameraInfoManager;


cv_bridge::CvImage left_stereo_cv_image_msg;
cv_bridge::CvImage right_stereo_cv_image_msg;
sensor_msgs::Image left_stereo_image_msg;
sensor_msgs::Image right_stereo_image_msg;
sensor_msgs::CameraInfo left_stereo_camera_info_msg;
sensor_msgs::CameraInfo right_stereo_camera_info_msg;

std_msgs::Header header;

raspicam::RaspiCam_Cv camera;


void Exit(int );
void rotate(Mat &src, int );
void readYamlFileToCameraInfo(string , sensor_msgs::CameraInfo& );
bool CheckIfFileExists(string ) ;

int main(int argc, char** argv)
{
	const int stereo_image_width = image_width * 2;
	const int stereo_image_height = image_height ;

	//Initialize camera
	cout<<"Initializing cameras"<<"\n";
	cout<<"resolution = "<<image_width<<"x"<<image_height<<"\n";
	cout<<"image_capture_fps = "<<image_capture_fps<<"\n";
	cout<<"publish_fps = "<<publish_fps<<"\n";

  	signal(SIGINT, Exit);

	//configure camera properties
	camera.set(CV_CAP_PROP_FRAME_WIDTH, stereo_image_width);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT, stereo_image_height);
	camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
	camera.set(CV_CAP_PROP_FPS, image_capture_fps);
	camera.set(CV_CAP_PROP_EXPOSURE, 60);

	//Set stereoscopic mode
	if (!camera.setStereoMode(1)) {
		cerr<<"Error setting stereoscopic mode"<<endl;
		Exit(-1);
	}

	//Validate open
	if (!camera.open()) {
		cerr<<"Error opening the cameras"<<endl;
		Exit(-1);
	}

	sleep(2); //wait for cameras to warm up;
	cout<<"cameras initialization complete\n";

	if(image_rotation_angle!=0 && image_rotation_angle!=90 && image_rotation_angle!=180 && image_rotation_angle!=270){
		cerr<<"image_rotation_angle = "<<image_rotation_angle<<" is invalid"<<endl;
		Exit(-1);
	}

	if(PUBLISH_TO_ROS)
	{
		//initialize ros node
		cout<<"setting up ros stereo_image_publisher node and image publishers\n";
		ros::init(argc, argv, "stereo_image_publisher", ros::init_options::NoSigintHandler);
		ros::NodeHandle nh;
		//cameraInfoManager = camera_info_manager::CameraInfoManager(nh);
	  	image_transport::ImageTransport it(nh);
		

		//initialize publishers
		left_stereo_image_publisher = it.advertiseCamera(LEFT_STEREO_IMAGE_TOPIC, 10);
		right_stereo_image_publisher = it.advertiseCamera(RIGHT_STEREO_IMAGE_TOPIC, 10);

		//init messages
		left_stereo_cv_image_msg.encoding = "bgr8";
		right_stereo_cv_image_msg.encoding = "bgr8";

		if(!CheckIfFileExists(left_pi_camera_calibration_file)) {
			cerr<<"failed to accecss left_pi_camera_calibration_file"<<endl;
			Exit(-1);
		}

		if(!CheckIfFileExists(right_pi_camera_calibration_file)) {
			cerr<<"failed to accecss right_pi_camera_calibration_file"<<endl;
			Exit(-1);
		}

		readYamlFileToCameraInfo(left_pi_camera_calibration_file, left_stereo_camera_info_msg);
		readYamlFileToCameraInfo(right_pi_camera_calibration_file, right_stereo_camera_info_msg);

	}	


	int drop_frames = floor(image_capture_fps/(float)publish_fps);
	if (drop_frames > image_capture_fps) //max fps to publish should not be more than image capture fps
		drop_frames = image_capture_fps;
	if (drop_frames == 0) //set to 1 if division caused value<1
		drop_frames = 1;

	int frame_id = 0;

	//Create pace setter to maintain FPS
	PaceSetter camerapacer(image_capture_fps, "Main thread");
	
	cout<<"stereo image capture started\n";

	//Loop till end of time
   	while (1) {
		frame_id += 1;

		//Grab stereo image
		camera.grab();

		//Copy to Mat
		Mat image;
		camera.retrieve(image);


		//split into left and right images
		Mat left_stereo_image{ Size(image.cols / 2, image.rows), image.type(), Scalar(0) };
		image( Rect(image.cols / 2, 0, image.cols / 2, image.rows) ).copyTo(
			left_stereo_image(Rect(0, 0, image.cols / 2, image.rows)));
		Mat right_stereo_image{ left_stereo_image.size(), left_stereo_image.type(), Scalar(0) };
		image( Rect(0, 0, image.cols / 2, image.rows) ).copyTo(
			right_stereo_image(Rect(0, 0, image.cols / 2, image.rows)));

		if(image_rotation_angle!=0)
		{
			rotate(left_stereo_image, image_rotation_angle);
			rotate(right_stereo_image, image_rotation_angle);
			//hconcat(left_stereo_image, right_stereo_image, image); //merge images again
		}

		//if(VERBOSE)
			//cout<<"\nWritten Frame "<<frame_id;

		//only publish frames that are not to be dropped
		if(PUBLISH_TO_ROS && frame_id % drop_frames == 0)
		{

			//set images to cv_bridge image messages
			left_stereo_cv_image_msg.image = left_stereo_image;
			right_stereo_cv_image_msg.image = right_stereo_image;

			//set header
			header.seq = frame_id;
			header.stamp = ros::Time::now();
			
			header.frame_id = "stereo";
			left_stereo_cv_image_msg.header = header;
			left_stereo_camera_info_msg.header = header;
			
			//header.frame_id = "right_pi_camera";
			right_stereo_cv_image_msg.header = header;
			right_stereo_camera_info_msg.header = header;

			//convert CvImages to Images
			left_stereo_cv_image_msg.toImageMsg(left_stereo_image_msg);
			right_stereo_cv_image_msg.toImageMsg(right_stereo_image_msg);

			//publish stereo image and cameraInfo messaages
			left_stereo_image_publisher.publish(left_stereo_image_msg, left_stereo_camera_info_msg);
			right_stereo_image_publisher.publish(right_stereo_image_msg, right_stereo_camera_info_msg);
 	 	
			if(VERBOSE){
				boost::posix_time::ptime my_posix_time = header.stamp.toBoost();
				cout<<"\nPublished Frame "<<frame_id<<" at "<<boost::posix_time::to_iso_extended_string(my_posix_time);
			}
		}

		if(SAVE_TO_IMAGE_FILES){

			string left_image_filename = "image-" + to_string(frame_id) + "-left.jpg";
			imwrite(left_image_filename, left_stereo_image );

			string right_image_filename = "image-" + to_string(frame_id) + "-right.jpg";
			imwrite(right_image_filename, right_stereo_image );
		}

		//Set pace
		camerapacer.SetPace();
	}


    return 0;
}

void Exit(int s){
	cout<<"\nstopping stereo_image_and_camera_info_publisher"<<endl;
	if(camera.isOpened())
		camera.release();
	if(!ros::isShuttingDown() && ros::ok())
		ros::shutdown();
	exit(1);
}

void rotate(Mat &src, int angle)
{
    if (angle == 90){
    transpose(src, src);  
		flip(src, src,1); //transpose+flip(1)=CW
  	} 
	else if (angle == 270) {
		transpose(src, src);  
		flip(src, src,0); //transpose+flip(0)=CCW     
  	} 
	else if (angle == 180){
		flip(src, src,-1);    //flip(-1)=180          
  } 
}

void readYamlFileToCameraInfo(string file, sensor_msgs::CameraInfo& camera_info_msg)
{
	YAML::Node camera_info_yaml = YAML::LoadFile(file);

	camera_info_msg.width = camera_info_yaml["image_width"].as<uint32_t>();
	camera_info_msg.height = camera_info_yaml["image_height"].as<uint32_t>();
	camera_info_msg.distortion_model = camera_info_yaml["distortion_model"].as<std::string>();
	camera_info_msg.D = camera_info_yaml["distortion_coefficients"]["data"].as<std::vector<double>>();
	
	vector<double> K_vector = camera_info_yaml["camera_matrix"]["data"].as<std::vector<double>>();
	for (int i=0;i!=K_vector.size();i++)
		camera_info_msg.K[i] = K_vector[i];
	
	vector<double> R_vector = camera_info_yaml["rectification_matrix"]["data"].as<std::vector<double>>();
	for (int i=0;i!=R_vector.size();i++)
		camera_info_msg.R[i] = R_vector[i];

	vector<double> P_vector = camera_info_yaml["projection_matrix"]["data"].as<std::vector<double>>();
	for (int i=0;i!=P_vector.size();i++)
		camera_info_msg.P[i] = P_vector[i];
}

bool CheckIfFileExists(string name) 
{
    if (FILE *file = fopen(name.c_str(), "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }   
}
