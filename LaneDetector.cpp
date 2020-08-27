/*@file LaneDetector.cpp
*@author Miguel Maestre Trueba
*@brief Definition of all the function that form part of the LaneDetector class.
*@brief The class will take RGB images as inputs and will output the same RGB image but
*@brief with the plot of the detected lanes and the turn prediction.
*/
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "LaneDetector.h"
#define CVX_RED CV_RGB(0xff, 0x00, 0x00)  
#define CVX_GREEN CV_RGB(0x00, 0xff, 0x00)  
#define CVX_BLUE CV_RGB(0x00, 0x00, 0xff)
using namespace std;
// IMAGE BLURRING
/**
*@brief Apply gaussian filter to the input image to denoise it
*@param inputImage is the frame of a video in which the
*@param lane is going to be detected
*@return Blurred and denoised image
*/
cv::Mat LaneDetector::deNoise(cv::Mat inputImage) {
	cv::Mat output;

	//cv::GaussianBlur(inputImage, output, cv::Size(3, 3), 0, 0);
    cv::GaussianBlur(inputImage, output, cv::Size(5, 5), 0, 0);
	return output;
}

// EDGE DETECTION
/**
*@brief Detect all the edges in the blurred frame by filtering the image
*@param img_noise is the previously blurred frame
*@return Binary image with only the edges represented in white
*/
cv::Mat LaneDetector::edgeDetector(cv::Mat img_noise) {
	cv::Mat output;
	cv::Mat kernel;
	cv::Point anchor;
	cv::Mat hsvimg;
	cv::Mat hsv_channels[3];
	cv::Mat rgb_channels[3];
	cv::Mat grayimg;
	cv::Mat sobelx;
	cv::Mat sobelimg;

	CvMemStorage* storage = cvCreateMemStorage();  
    //创建一个内存储存器 默认为 64K  
    CvSeq* first_contour = NULL;  
    //创建一个动态序列指针，用其指向第一个存储轮廓单元地址  
	IplImage* img_lpl;
	IplImage* draw_img;

	//cv::split(img_noise,rgb_channels);

	// Convert image from RGB to gray
	//cout<<"channels = "<<img_noise.channels()<<std::endl;
	if(img_noise.channels()>1)
	{
		cv::cvtColor(img_noise, output, cv::COLOR_RGB2GRAY);
		//output = RGBToGray(img_noise); // 自己的权值转灰度图
		cv::imshow("gray",output);
		cv::GaussianBlur(output, output, cv::Size(5, 5), 0, 0);
		cv::threshold(output, output, 160, 255, cv::THRESH_BINARY);
		////////////////////////////////////////////////////////////////////////////////////////////轮廓检测
		 img_lpl =(IplImage *) & IplImage(output); // Mat 转IplImage*

		int NC = cvFindContours(      
        //cvFindContours从二值图像中检索轮廓，并返回检测到的轮廓的个数  
        	img_lpl,         
            storage,                //存储轮廓元素的储存容器  
            &first_contour,         //指向第一个输出轮廓  
            sizeof (CvContour),       
            CV_RETR_LIST            //提取所有轮廓，并且放置在 list 中  
            );  
			cvDrawContours(  
                draw_img,   //用于绘制轮廓的图像  
                first_contour,          //指向目前轮廓所在地址空间  
                CVX_RED,    //外层轮廓颜色  
                CVX_BLUE,   //内层轮廓颜色  
                0,          //等级为0，绘制单独的轮廓  
                1,          //轮廓线条粗细  
                8           //线段类型为（8邻接)连接线  
                );  
				cv::imshow("img_contour", draw_img);  
		//////////////////////////////////////////////////////////////////////
		
		cv::Sobel(output, sobelx, CV_64F,1,0,3);  // 对灰度图进行sobel检测
		
		//cv::Scharr(output, sobelx, CV_16S, 1, 0, 3); //改进Sobel算子
		//cv::imshow("gray",sobelx);
		//cv::Canny(sobelx,sobelx,50,150,3);
		cv::imshow("sobel",sobelx);
		
	}
	else
	{
		output = img_noise;
	}
	
	
	//cv::cvtColor(img_noise, hsvimg, CV_BGR2HSV);
	//cv::split(hsvimg,hsv_channels);
	//cv::addWeighted(hsv_channels[1], 0.5, grayimg, 1.0 - 0.5, 0, output);// 将两个图像进行融合
	//output = grayimg;
	//cv::imshow("gray ",grayimg);
	//cv::imshow("output",grayimg);
	//equalizeHist( output, output ); // histogram 均衡化会使车道白线更不明显
	//output = hsv_channels[1];
	// Binarize gray image将灰度图二值化
	cv::GaussianBlur(output, output, cv::Size(5, 5), 0, 0);
	//cv::threshold(output, output, 160, 255, cv::THRESH_BINARY);

	//cv::threshold(output, output, 0, 255, cv::THRESH_BINARY | CV_THRESH_OTSU); // OSTU
	cv::imshow("binary image", output);

	// Create the kernel [-1 0 1]
	// This kernel is based on the one found in the
	// Lane Departure Warning System by Mathworks
	anchor = cv::Point(-1, -1);
	kernel = cv::Mat(1, 3, CV_32F);
	kernel.at<float>(0, 0) = -1;
	kernel.at<float>(0, 1) = 0;
	kernel.at<float>(0, 2) = 1;

	// Filter the binary image to obtain the edges
	//cv::filter2D(output, output, -1, kernel, anchor, 0, cv::BORDER_DEFAULT);



	cv::Canny(output,output,50,150,3);

	//cv::imshow("output", output);

	//cv::waitKey(10000);
	return output;
}

// MASK THE EDGE IMAGE
/**
*@brief Mask the image so that only the edges that form part of the lane are detected
*@param img_edges is the edges image from the previous function
*@return Binary image with only the desired edges being represented
*/
    cv::Mat LaneDetector::mask(cv::Mat img_edges) {
	cv::Mat output;
	cv::Mat mask = cv::Mat::zeros(img_edges.size(), img_edges.type());
	 
		/*cv::Point pts[4] = {
		cv::Point(100, 540),
		cv::Point(400, 350),
		cv::Point(600, 350),
		cv::Point(860, 540)
	};*/
    float row = img_edges.rows;
    float col = img_edges.cols;
	/*cv::Point pts[4] = {
		cv::Point(0.1*col, row),
		cv::Point(0.42*col, 0.7*row),
		cv::Point(0.625*col, 0.7*row),
		cv::Point(0.9*col, row)
	};*/

	    cv::Point pts[4] = {
		cv::Point(0.0*col, row),
		cv::Point(0.0*col, 0.5*row),
		cv::Point(1.0*col, 0.5*row),
		cv::Point(1.0*col, row)
	};


	// Create a binary polygon mask
	cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255, 0, 0));
	// Multiply the edges image and the mask to get the output
	cv::bitwise_and(img_edges, mask, output); // 将两个图像做与操作
	cv::imshow("masked image",output);

	return output;
}

// HOUGH LINES
/**
*@brief Obtain all the line segments in the masked images which are going to be part of the lane boundaries
*@param img_mask is the masked binary image from the previous function
*@return Vector that contains all the detected lines in the image
*/
std::vector<cv::Vec4i> LaneDetector::houghLines(cv::Mat img_mask) {
	std::vector<cv::Vec4i> line;

	// rho and theta are selected by trial and error
	//HoughLinesP(img_mask, line, 1, CV_PI / 180, 20, 20, 30);
	HoughLinesP(img_mask, line, 1, CV_PI / 180, 20, 30, 30);

	return line;
}

// SORT RIGHT AND LEFT LINES
/**
*@brief Sort all the detected Hough lines by slope.
*@brief The lines are classified into right or left depending
*@brief on the sign of their slope and their approximate location
*@param lines is the vector that contains all the detected lines
*@param img_edges is used for determining the image center
*@return The output is a vector(2) that contains all the classified lines
*/
//霍夫变换后，先将线段按照斜率和坐标 分到左 、右 两边
std::vector<std::vector<cv::Vec4i> > LaneDetector::lineSeparation(std::vector<cv::Vec4i> lines, cv::Mat img_edges) {
	std::vector<std::vector<cv::Vec4i> > output(2);
	size_t j = 0;
	cv::Point ini;
	cv::Point fini;
	double slope_thresh = 0.3;
	std::vector<double> slopes;
	std::vector<cv::Vec4i> selected_lines;
	std::vector<cv::Vec4i> right_lines, left_lines;

	// Calculate the slope of all the detected lines
	//lines 里的每一项存储的是两个点，起点和终点
	int times = 0 ;
	for (auto i : lines) {
		//cout<<"tiems = "<<times++<<endl;
		ini = cv::Point(i[0], i[1]);
		fini = cv::Point(i[2], i[3]);

		// Basic algebra: slope = (y1 - y0)/(x1 - x0)
		double slope = (static_cast<double>(fini.y) - static_cast<double>(ini.y)) / (static_cast<double>(fini.x) - static_cast<double>(ini.x) + 0.00001);

		// If the slope is too horizontal, discard the line
		// If not, save them  and their respective slope
		if (std::abs(slope) > slope_thresh) {
			slopes.push_back(slope);
			selected_lines.push_back(i);
		}
	}

	// Split the lines into right and left lines
	img_center = static_cast<double>((img_edges.cols / 2));
	while (j < selected_lines.size()) {
		ini = cv::Point(selected_lines[j][0], selected_lines[j][1]);
		fini = cv::Point(selected_lines[j][2], selected_lines[j][3]);

		// Condition to classify line as left side or right side
		if (slopes[j] > 0 && fini.x > img_center && ini.x > img_center) {
			right_lines.push_back(selected_lines[j]);
			right_flag = true;
		}
		else if (slopes[j] < 0 && fini.x < img_center && ini.x < img_center) {
			left_lines.push_back(selected_lines[j]);
			left_flag = true;
		}
		j++;
	}

	output[0] = right_lines;
	output[1] = left_lines;

	return output;
}

// REGRESSION FOR LEFT AND RIGHT LINES
/**
*@brief Regression takes all the classified line segments initial and final points and fits a new lines out of them using the method of least squares.
*@brief This is done for both sides, left and right.
*@param left_right_lines is the output of the lineSeparation function
*@param inputImage is used to select where do the lines will end
*@return output contains the initial and final points of both lane boundary lines
*/
// 分别将左侧、右侧的边缘点 进行线性回归
std::vector<cv::Point> LaneDetector::regression(std::vector<std::vector<cv::Vec4i> > left_right_lines, cv::Mat inputImage) {
	std::vector<cv::Point> output(4);
	cv::Point ini;
	cv::Point fini;
	cv::Point ini2;
	cv::Point fini2;
	cv::Vec4d right_line;
	cv::Vec4d left_line;
	std::vector<cv::Point> right_pts;
	std::vector<cv::Point> left_pts;

	// If right lines are being detected, fit a line using all the init and final points of the lines
	if (right_flag == true) {
		for (auto i : left_right_lines[0]) {
			ini = cv::Point(i[0], i[1]);
			fini = cv::Point(i[2], i[3]);

			right_pts.push_back(ini);
			right_pts.push_back(fini);
		}

		if (right_pts.size() > 0) {
			// The right line is formed here
			cv::fitLine(right_pts, right_line, CV_DIST_L2, 0, 0.01, 0.01);
			right_m = right_line[1] / right_line[0];
			right_b = cv::Point(right_line[2], right_line[3]);
		}
	}

	// If left lines are being detected, fit a line using all the init and final points of the lines
	if (left_flag == true) {
		for (auto j : left_right_lines[1]) {
			ini2 = cv::Point(j[0], j[1]);
			fini2 = cv::Point(j[2], j[3]);

			left_pts.push_back(ini2);
			left_pts.push_back(fini2);
		}

		if (left_pts.size() > 0) {
			// The left line is formed here
			cv::fitLine(left_pts, left_line, CV_DIST_L2, 0, 0.01, 0.01);
			left_m = left_line[1] / left_line[0];
			left_b = cv::Point(left_line[2], left_line[3]);
		}
	}

	// One the slope and offset points have been obtained, apply the line equation to obtain the line points
	int ini_y = inputImage.rows; //画直线所用的两个端点的y坐标的值
	//int fin_y = 470;
	int fin_y = 0.77*inputImage.rows;

	double right_ini_x = ((ini_y - right_b.y) / right_m) + right_b.x;
	double right_fin_x = ((fin_y - right_b.y) / right_m) + right_b.x;

	double left_ini_x = ((ini_y - left_b.y) / left_m) + left_b.x;
	double left_fin_x = ((fin_y - left_b.y) / left_m) + left_b.x;

	output[0] = cv::Point(right_ini_x, ini_y);//右侧直线的两个端点
	//output[0] =right_b;
	output[1] = cv::Point(right_fin_x, fin_y);
	output[2] = cv::Point(left_ini_x, ini_y);//左侧直线的两个端点
	//output[2] = left_b;
	output[3] = cv::Point(left_fin_x, fin_y);

	return output;
}

// TURN PREDICTION
/**
*@brief Predict if the lane is turning left, right or if it is going straight
*@brief It is done by seeing where the vanishing point is with respect to the center of the image
*@return String that says if there is left or right turn or if the road is straight
*/
std::string LaneDetector::predictTurn() {
	std::string output;
	double vanish_x;
	double thr_vp = 10;//10个像素
	// The vanishing point is the point where both lane boundary lines intersect 消失点是两条直线的交点
	vanish_x = static_cast<double>(((right_m*right_b.x) - (left_m*left_b.x) - right_b.y + left_b.y) / (right_m - left_m));
	
	// The vanishing points location determines where is the road turning
	if (vanish_x < (img_center - thr_vp)) //消失点在图像左侧，左转
		output = "Left Turn";
	else if (vanish_x >(img_center + thr_vp))//消失点在图像右侧，右转
		output = "Right Turn";
	else if (vanish_x >= (img_center - thr_vp) && vanish_x <= (img_center + thr_vp))// 消失点在图像中心部分，直行
		output = "Straight";

	return output;
}

// PLOT RESULTS
/**
*@brief This function plots both sides of the lane, the turn prediction message and a transparent polygon that covers the area inside the lane boundaries
*@param inputImage is the original captured frame
*@param lane is the vector containing the information of both lines
*@param turn is the output string containing the turn information
*@return The function returns a 0
*/
int LaneDetector::plotLane(cv::Mat inputImage, std::vector<cv::Point> lane, std::string turn) {
	std::vector<cv::Point> poly_points;
	cv::Mat output;//图片

	// Create the transparent polygon for a better visualization of the lane
	inputImage.copyTo(output);
	poly_points.push_back(lane[2]);
	poly_points.push_back(lane[0]);
	poly_points.push_back(lane[1]);
	poly_points.push_back(lane[3]);
	cv::fillConvexPoly(output, poly_points, cv::Scalar(0, 0, 255), CV_AA, 0);//在图片上画出一个梯形,并用红色填充，CV_AA=16
	cv::addWeighted(output, 0.3, inputImage, 1.0 - 0.3, 0, inputImage);// 将两个图像进行融合

	// Plot both lines of the lane boundary画线
	cv::line(inputImage, lane[0], lane[1], cv::Scalar(0, 255, 255), 5, CV_AA);
	cv::line(inputImage, lane[2], lane[3], cv::Scalar(0, 255, 255), 5, CV_AA);

	// Plot the turn message
	cv::putText(inputImage, turn, cv::Point(50, 90), cv::FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1, CV_AA);

	// Show the final output image
	cv::namedWindow("Lane", CV_WINDOW_AUTOSIZE);
	cv::imshow("Lane", inputImage);
	return 0;
}

cv::Mat LaneDetector::RGBToGray(cv::Mat src)
{
	if (src.channels()==1)
  {
   return src;
  }
  else if (src.channels() == 3 || src.channels() == 4)
  {
   
   int rows = src.rows;
   int cols = src.cols;
   cv::Mat mGray(rows, cols, CV_8UC1, cv::Scalar(100, 0, 0));
   int bValue, gValue, rValue;
   for (int i = 0; i < rows; i++)
   {
    for (int j = 0; j < cols; j++)
    {
     bValue = src.at<cv::Vec3b>(i, j)[0];
     gValue = src.at<cv::Vec3b>(i, j)[1];
     rValue = src.at<cv::Vec3b>(i, j)[2];
     mGray.at<uchar>(i, j) = cv::saturate_cast<uchar>(0.01*bValue + 0.01*gValue + 0.98*rValue);
    }
   }
   return mGray;
  }
  else
  {
   throw "error";
  }

}
