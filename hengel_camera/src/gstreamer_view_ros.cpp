/*
  Example code for displaying gstreamer video from the CSI port of the Nvidia Jetson in OpenCV.
  Created by Peter Moran on 7/29/17.
  https://gist.github.com/peter-moran/742998d893cd013edf6d0c86cc86ff7f
*/
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/videoio.hpp"

std::string get_tegra_pipeline(int width, int height, int fps, int cam_index) {
    return "nvcamerasrc sensor-id="+std::to_string(cam_index)+" ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(width) + ", height=(int)" +
           std::to_string(height) + ", format=(string)I420, framerate=(fraction)" + std::to_string(fps) +
           "/1 ! nvvidconv flip-method=2 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

int main(int argc, char** argv) {
    uint16_t WIDTH = 820;
    uint16_t HEIGHT = 616;
    uint8_t FPS = 30;
    // Define the gstream pipeline

    ros::init(argc, argv, "picamPublisher");
    ros::NodeHandle nh;

    std::string pipeline[6];
    std::vector<ros::Publisher> pub_vec;
    pub_vec.reserve(6);

    sensor_msgs::ImagePtr msg;
    
    std::vector<cv::VideoCapture> cap_vec;
    cap_vec.reserve(6);

    ros::Rate loop_rate(50);

    int i;
    for(i=0;i<5;i++){
	pub_vec.push_back(nh.advertise<sensor_msgs::Image>("Image"+std::to_string(i),10));
	pipeline[i]=get_tegra_pipeline(WIDTH, HEIGHT, FPS, i+1);
	cv::VideoCapture cap2(pipeline[i], cv::CAP_GSTREAMER);
	// Create OpenCV capture object, ensure it works.
	cap_vec.push_back(cap2);
	if (!cap_vec[i].isOpened()){
	    std::cout<<"Connection failed- Camera number"<< i;
	    return -1;
	}	
    }
    // View video
    cv::Mat frame;
    while (ros::ok()) {
	for(i=1; i<5; i++){
            cap_vec[i] >> frame;  // Get a new frame from camera
	    if(!frame.empty()){
		msg=cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
		pub_vec[i].publish(msg);
	    }
	    cv::waitKey(1);
	}
	ros::spinOnce();
	loop_rate.sleep();
    }
}
