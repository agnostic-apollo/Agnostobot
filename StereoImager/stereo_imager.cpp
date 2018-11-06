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
#include <ctime>

//3rd party libraries
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <raspicam/raspicam_cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//DAPrototype source files
#include "pace_setter_class.cpp"

/*usage
#install dependencies
sudo apt-get install ffmpeg x264 libx264-dev

cd /opt/agnostobot/StereoImager
g++ stereo_imager.cpp -o  stereo_imager -I/usr/local/include/ -L/opt/vc/lib -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util -lopencv_core -lopencv_highgui -lopencv_imgproc -lboost_date_time -lboost_system -lboost_thread -pthread -std=c++11 
./stereo_imager

#force stop
ps -aux | grep stereo_imager
kill -9 pid
*/

using namespace std;
using namespace cv;

//encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90] #set image quality after compression
//fourcc = cv2.VideoWriter_fourcc(*'X264') #set video codec
int fourcc = 0x00000021; //to fix ffmpeg error


int image_capture_fps=10; //set number of frames per second to capture and save to file
int image_width=640; //set video width
int image_height=480; //set video height
int image_rotation_angle=180; //set rotation angle, valid values are 0, 90, 180 and 270
int camera_exposure=60; //set camera exposure, valid values 0-100 or -1

bool VERBOSE=false; //set to True if you want to enable logging
bool SAVE_TO_VIDEO_FILE=false; //set to True if you want to save video to file
bool SAVE_TO_IMAGE_FILES=false; //set to True if you want to save image files
bool VIEW_IN_WINDOW=true; //set to True if you want to view images in opencv window

raspicam::RaspiCam_Cv camera;
VideoWriter video;

void mySigintHandler(int s);
void rotate(Mat &src, int angle);

int main(int argc, char** argv)
{
	const int stereo_image_width = image_width * 2;
	const int stereo_image_height = image_height ;

	//Initialize camera
	cout<<"Initializing cameras"<<"\n";
	cout<<"resolution = "<<image_width<<"x"<<image_height<<"\n";
	cout<<"image_capture_fps = "<<image_capture_fps<<"\n";

  	signal(SIGINT, mySigintHandler);

	//configure camera properties
	camera.set(CV_CAP_PROP_FRAME_WIDTH, stereo_image_width);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT, stereo_image_height);
	camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
	camera.set(CV_CAP_PROP_FPS, image_capture_fps);
	camera.set(CV_CAP_PROP_EXPOSURE, 60);


	//Set stereoscopic mode
	if (!camera.setStereoMode(1)) {
		cerr<<"Error setting stereoscopic mode"<<endl;
		exit(-1);
	}

	//Validate open
	if (!camera.open()) {
		cerr<<"Error opening the cameras"<<endl;
		exit(-1);
	}

	sleep(2); //wait for cameras to warm up;
	cout<<"cameras initialization complete\n";


	if(image_rotation_angle!=0 && image_rotation_angle!=90 && image_rotation_angle!=180 && image_rotation_angle!=270){
		cerr<<"image_rotation_angle = "<<image_rotation_angle<<" is invalid"<<endl;
		exit(-1);
	}

	if(SAVE_TO_VIDEO_FILE)
	{
		time_t rawtime;
		struct tm * timeinfo;
		char buffer[80];
		time (&rawtime);
		timeinfo = localtime(&rawtime);
		strftime(buffer,sizeof(buffer),"%d-%m-%Y_%H-%M-%S",timeinfo);
		string filename(buffer);
		filename = filename + ".avi";
		//string filename = "video.mp4"
		int video_width = stereo_image_width;
		int video_height = stereo_image_height;
		cout<<"video file = "<<filename<<endl;

		if(image_rotation_angle==90 || image_rotation_angle==270){
			video_width = stereo_image_height;
			video_height = stereo_image_width;
		}

  		video = VideoWriter(filename, CV_FOURCC('M','J','P','G'), image_capture_fps, Size(video_width, video_height), true); 
		
		if(!video.isOpened()){
			cerr<<"Error creating video writer"<<endl;
			exit(-1);
		}
	}

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
			hconcat(left_stereo_image, right_stereo_image, image); //merge images again
		}


		if(SAVE_TO_VIDEO_FILE){
    		video.write(image);
		}

		if(SAVE_TO_IMAGE_FILES){

			string left_image_filename = "image-" + to_string(frame_id) + "-left.jpg";
			imwrite(left_image_filename, left_stereo_image );

			string right_image_filename = "image-" + to_string(frame_id) + "-right.jpg";
			imwrite(right_image_filename, right_stereo_image );
		}

		if(VIEW_IN_WINDOW){
    		imshow("Stereo Image Viewer", image);
			waitKey(0);
		}

		//Set pace
		camerapacer.SetPace();
	}

    return 0;
}

void mySigintHandler(int s){
	cout<<"\nstopping stereo_image_publisher"<<endl;
	if(camera.isOpened())
		camera.release();
	if(video.isOpened())	 
		video.release();
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

