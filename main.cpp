//#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "LaneDetector.h"
//#include "LaneDetector.cpp"
using namespace std;

/**
*@brief Function main that runs the main algorithm of the lane detection.
*@brief It will read a video of a car in the highway and it will output the
*@brief same video but with the plotted detected lane
*@param argv[] is a string to the full path of the demo video
*@return flag_plot tells if the demo has sucessfully finished
*/

void gammaProcessImage(cv::Mat& oriMat,double gamma,cv::Mat& outputMat){
   
    //伽马方法也是按照一个公式修改了每个像素值，我们可以通过LUT函数进行编写，它的公式是：
    //O=(I/255)的γ次方×255
    //代码如下
    cv::Mat lookupTable(1,256,CV_8U);
    uchar* p = lookupTable.ptr();
    for (int i =0 ; i < 256; i++) {
        p[i] =cv::saturate_cast<uchar>(pow(i/255.0, gamma) * 255.0);
    }
    LUT(oriMat,lookupTable,outputMat);
}

int main() {
	
	// The input argument is the location of the video
	std::cout<<"enter main"<<std::endl;
	//challenge   solidWhiteRight  solidYellowLeft
	cv::VideoCapture cap("/home/tang/tang/lane_detect_document/Lane-Lines-Detection/test_videos/challenge.mp4");

	if (!cap.isOpened())
		return -1;

	LaneDetector lanedetector;  // Create the class object
	cv::Mat frame;
	cv::Mat img_denoise;
	cv::Mat img_edges;
	cv::Mat img_edges_hsv;
	cv::Mat img_mask;
	cv::Mat img_mask_hsv;
	cv::Mat img_lines;
	cv::Mat dft;
	cv::Mat lab;
	std::vector<cv::Vec4i> lines;
	std::vector<cv::Vec4i> lines_hsv;
	std::vector<std::vector<cv::Vec4i> > left_right_lines;
	std::vector<cv::Point> lane;
	std::string turn;
	int flag_plot = -1;
	int i = 3652;
	cv::Mat hsvimg;
	cv::Mat hsv_channels[3];
	cv::Mat lab_channels[3];
	char name[100];

	// Main algorithm starts. Iterate through every frame of the video
	while (i < 4852) {
		// Capture frame
		//if (!cap.read(frame))
		//	break;
		sprintf(name,"/home/tang/tang/record/frame_%d.jpg",i);
		cout<<"NO. "<<i<<" picture"<<endl;
		frame = cv::imread(name);
		cv::imshow("raw picture",frame);
		//cv::imwrite(name,frame);
		//frame =cv::imread("/home/tang/tang/128.jpg");
		//frame = cv::imread("/home/tang/tang/lane_detect_document/Lane-Lines-Detection/test_images/lidar.jpg");
	
		dft= cv::Mat::zeros(frame.rows, frame.cols,CV_32F);
		gammaProcessImage(frame,3.5,dft); // 3.5
		//cout<<"type="<<dft.type()<<endl;
		//frame.convertTo(dft, -1, 1.5, 0);
		cv::imshow("covert image",dft);
		//frame = dft;
		// Denoise the image using a Gaussian filter
		img_denoise = lanedetector.deNoise(frame);
		cv::cvtColor(img_denoise, hsvimg, CV_BGR2HSV);
		cv::split(hsvimg,hsv_channels);
		cv::imshow("myhsv",hsv_channels[1]);
		//cv::waitKey(10000);
		//cv::cvtColor(img_denoise, lab, cv::COLOR_BGR2Lab);
		cv::cvtColor(img_denoise, lab, CV_BGR2HLS);
		cv::split(lab,lab_channels);
		cv::imshow("lab-L",lab_channels[1]);
		
		//
		// Detect edges in the image
		img_edges = lanedetector.edgeDetector(img_denoise);
		//img_edges_hsv= lanedetector.edgeDetector(hsv_channels[1]);
	    //img_edges+=img_edges_hsv;
		
		cv::imshow("all edges ",img_edges);
		// Mask the image so that we only get the ROI
		//img_mask_hsv =  lanedetector.mask(img_edges_hsv);
		img_mask = lanedetector.mask(img_edges);
	
		// Obtain Hough lines in the cropped image
		//lines_hsv = lanedetector.houghLines(img_mask_hsv);
		lines = lanedetector.houghLines(img_mask);

		if (!lines.empty())
		{
			// Separate lines into left and right lines
			left_right_lines = lanedetector.lineSeparation(lines, img_edges);

			// Apply regression to obtain only one line for each side of the lane
			lane = lanedetector.regression(left_right_lines, frame);
		
			// Predict the turn by determining the vanishing point of the the lines
			turn = lanedetector.predictTurn();
			
			// Plot lane detection
			flag_plot = lanedetector.plotLane(frame, lane, turn);

			i += 1;
			cv::waitKey(25);
		}
		else {
			flag_plot = -1;
			i += 1;
		}
		cv::waitKey(0);
	}
	return flag_plot;
}
