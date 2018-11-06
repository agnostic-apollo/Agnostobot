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
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "yaml-cpp/yaml.h"

//DAPrototype source files
#include "pace_setter_class.cpp"

/*
usage
export ROS_MASTER_URI=http://192.168.0.180:11311
export ROS_IP=192.168.0.190
cd /opt/agnostobot/StereoCompressedImageAndCameraInfoPublisher
g++ stereo_compressed_image_and_camera_info_publisher.cpp -o  stereo_compressed_image_and_camera_info_publisher -I/usr/local/include/ -I/opt/ros/kinetic/include/ -L/opt/ros/kinetic/lib -L/opt/vc/lib -L/usr/local/lib -Wl,-rpath,/opt/ros/kinetic/lib -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util -lopencv_core -lopencv_highgui -lopencv_imgproc -lroscpp -lrosconsole -lrostime -lroscpp_serialization -limage_geometry -lcv_bridge -lboost_date_time -lboost_system -lboost_thread -pthread -lactionlib -lyaml-cpp -std=c++11 
./stereo_compressed_image_and_camera_info_publisher

//force stop
ps -aux | grep stereo_compressed_image_and_camera_info_publisher
kill -9 pid

//convert ros image messaages to files
rosrun image_view extract_images image:=/agnostobot/left/image _image_transport:=compressed
rosrun image_view extract_images image:=/agnostobot/right/image _image_transport:=compressed
//or simply echo on console
rostopic echo /agnostobot/left/image/compressed --noarr
rostopic echo /agnostobot/right/image/compressed --noarr
rostopic echo /agnostobot/left/camera_info
rostopic echo /agnostobot/right/camera_info
*/

using namespace std;
using namespace cv;

string LEFT_STEREO_COMPRESSED_IMAGE_TOPIC = "/agnostobot/left/image/compressed"; //set topic to publish left stereo compressed image 
string RIGHT_STEREO_COMPRESSED_IMAGE_TOPIC = "/agnostobot/right/image/compressed";  //set topic to publish right stereo compressed image 
string LEFT_STEREO_CAMERA_INFO_TOPIC = "/agnostobot/left/camera_info";  //set topic to publish left stereo camera info
string RIGHT_STEREO_CAMERA_INFO_TOPIC = "/agnostobot/right/camera_info";  //set topic to publish left stereo camera info

string LEFT_STEREO_CAMERA_FRAME_ID = "/agnostobot_left_camera";
string RIGHT_STEREO_CAMERA_FRAME_ID = "/agnostobot_right_camera";


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
bool RECTIFY_IMAGES=false; //image rectification currently messes up the image, have to fix but it increases cpu load on pi drastically so might not be worth to do on pi


ros::Publisher left_stereo_compressed_image_publisher;
ros::Publisher right_stereo_compressed_image_publisher;
ros::Publisher left_stereo_camera_info_publisher;
ros::Publisher right_stereo_camera_info_publisher;
//image_geometry::PinholeCameraModel model;

//camera_info_manager::CameraInfoManager cameraInfoManager;


cv_bridge::CvImage left_stereo_cv_image_msg;
cv_bridge::CvImage right_stereo_cv_image_msg;

sensor_msgs::CameraInfo left_stereo_camera_info_msg;
sensor_msgs::CameraInfo right_stereo_camera_info_msg;

std_msgs::Header header;

raspicam::RaspiCam_Cv camera;

Mat left_stereo_rectified_image, left_stereo_camera_map_1, left_stereo_camera_map_2;
Mat right_stereo_rectified_image, right_stereo_camera_map_1, right_stereo_camera_map_2;
 
void Exit(int );
void rotate(Mat &src, int );
void readYamlFileToCameraInfo(string , sensor_msgs::CameraInfo& );
bool CheckIfFileExists(string ) ;
//Mat rectifyImage(Mat, const sensor_msgs::CameraInfo& );

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

		//initialize publishers
		left_stereo_compressed_image_publisher = nh.advertise<sensor_msgs::CompressedImage>(LEFT_STEREO_COMPRESSED_IMAGE_TOPIC, 100);
		right_stereo_compressed_image_publisher = nh.advertise<sensor_msgs::CompressedImage>(RIGHT_STEREO_COMPRESSED_IMAGE_TOPIC, 100);

		left_stereo_camera_info_publisher = nh.advertise<sensor_msgs::CameraInfo>(LEFT_STEREO_CAMERA_INFO_TOPIC, 100);
		right_stereo_camera_info_publisher = nh.advertise<sensor_msgs::CameraInfo>(RIGHT_STEREO_CAMERA_INFO_TOPIC, 100);

		//init messages

		//left_stereo_cv_image_msg.encoding = "bgr8";
		//right_stereo_cv_image_msg.encoding = "bgr8";

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

	
	//initiate maps for rectification
	if(RECTIFY_IMAGES){

	initUndistortRectifyMap(Mat(3, 3, CV_32FC1, &left_stereo_camera_info_msg.K[0]), 
								left_stereo_camera_info_msg.D, 
									Mat(3, 3, CV_32FC1, &left_stereo_camera_info_msg.R[0]),
										Mat(3, 4, CV_32FC1, &left_stereo_camera_info_msg.P[0]),
											Size(image_width,image_height) , CV_16SC2, left_stereo_camera_map_1, left_stereo_camera_map_2);
	initUndistortRectifyMap(Mat(3, 3, CV_32FC1, &right_stereo_camera_info_msg.K[0]), 
								right_stereo_camera_info_msg.D, 
									Mat(3, 3, CV_32FC1, &right_stereo_camera_info_msg.R[0]),
										Mat(3, 4, CV_32FC1, &right_stereo_camera_info_msg.P[0]),
											Size(image_width,image_height), CV_16SC2, right_stereo_camera_map_1, right_stereo_camera_map_2);
	}

	vector<int> param(2);
	param[0] = cv::IMWRITE_JPEG_QUALITY;
	param[1] = 95; //default(95) 0-100
	
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

		if(RECTIFY_IMAGES){
			remap(left_stereo_image, left_stereo_rectified_image, left_stereo_camera_map_1, left_stereo_camera_map_2, 0);
			remap(right_stereo_image, right_stereo_rectified_image, right_stereo_camera_map_1, right_stereo_camera_map_2, 0);
		}
		else
		{
			left_stereo_rectified_image = left_stereo_image;
			right_stereo_rectified_image = right_stereo_image;
		}

		//if(VERBOSE)
			//cout<<"\nWritten Frame "<<frame_id;

		//only publish frames that are not to be dropped
		if(PUBLISH_TO_ROS && frame_id % drop_frames == 0)
		{
			sensor_msgs::CompressedImage left_stereo_compressed_image_msg;
			sensor_msgs::CompressedImage right_stereo_compressed_image_msg;

			//set header
			header.seq = frame_id;
			header.stamp = ros::Time::now();

			header.frame_id = LEFT_STEREO_CAMERA_FRAME_ID;
			left_stereo_compressed_image_msg.header = header;
			left_stereo_camera_info_msg.header = header;
			
			header.frame_id = RIGHT_STEREO_CAMERA_FRAME_ID;
			right_stereo_compressed_image_msg.header = header;
			right_stereo_camera_info_msg.header = header;

			left_stereo_compressed_image_msg.format = "jpeg";
			right_stereo_compressed_image_msg.format = "jpeg";

			//left_stereo_cv_image_msg.toCompressedImageMsg(left_stereo_compressed_image_msg, cv_bridge::Format::JPG);
			//right_stereo_cv_image_msg.toCompressedImageMsg(right_stereo_compressed_image_msg, cv_bridge::Format::JPG);

			// encode image to jpeg
			if(!imencode(".jpg", left_stereo_rectified_image, left_stereo_compressed_image_msg.data, param)){
				cerr<<"failed to compress left_stereo_image"<<endl;
				Exit(-1);
			}
			if(!imencode(".jpg", right_stereo_rectified_image, right_stereo_compressed_image_msg.data, param)){
				cerr<<"failed to compress right_stereo_image"<<endl;
				Exit(-1);
			}	
			//publish stereo compressed images messaages
			left_stereo_compressed_image_publisher.publish(left_stereo_compressed_image_msg);
			left_stereo_camera_info_publisher.publish(left_stereo_camera_info_msg);

			//publish stereo cameras cameraInfo messaages
			right_stereo_compressed_image_publisher.publish(right_stereo_compressed_image_msg);
			right_stereo_camera_info_publisher.publish(right_stereo_camera_info_msg);
 	 	
			if(VERBOSE){
				boost::posix_time::ptime my_posix_time = header.stamp.toBoost();
				cout<<"\nPublished Frame "<<frame_id<<" at "<<boost::posix_time::to_iso_extended_string(my_posix_time);
			}
		}

		if(SAVE_TO_IMAGE_FILES){

			string left_image_filename = "image-" + to_string(frame_id) + "-left.jpg";
			imwrite(left_image_filename, left_stereo_rectified_image );

			string right_image_filename = "image-" + to_string(frame_id) + "-right.jpg";
			imwrite(right_image_filename, right_stereo_rectified_image );
		}

		//Set pace
		camerapacer.SetPace();
	}


    return 0;
}

void Exit(int s){
	cout<<"\nstopping stereo_compressed_image_and_camera_info_publisher"<<endl;
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
/*
Mat rectifyImage(Mat image, const sensor_msgs::CameraInfo& camera_info_msg)
{
	// Verify camera is actually calibrated
	if (camera_info_msg.K[0] == 0.0) {
		cerr<<"camera is uncalibrated"<<endl;
		Exit(-1);
	}

	// If zero distortion, just pass the message along
	bool zero_distortion = true;
	for (size_t i = 0; i < camera_info_msg.D.size(); ++i)
	{
		if (camera_info_msg.D[i] != 0.0)
		{
			zero_distortion = false;
			break;
		}
	}
	// This will be true if D is empty/zero sized
	if (zero_distortion)
	{
		return image;
	}

	// Update the camera model
	model.fromCameraInfo(camera_info_msg);

	Mat rect_image;
	//return image;

	//select interpolation method for rectification 
	//NN (0): Nearest-neighbor sampling
	//Linear (1): Bilinear interpolation
	//Cubic (2): Bicubic interpolation over 4x4 neighborhood 
	//Area (3): Resampling using pixel area relation
	//Lanczos4 (4): Lanczos interpolation over 8x8 neighborhood 
	int interpolation = 0;
	
	// Rectify image
	model.rectifyImage(image, rect_image, interpolation);

	return rect_image;
}

*/

















