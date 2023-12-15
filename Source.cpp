#include <iostream>

#include "opencv2/opencv.hpp"

#include "Lian/Point.hpp"

#define PATH_IMG "resources/map.png"

int main() {

	cv::Mat img = cv::imread(PATH_IMG, cv::IMREAD_COLOR);

	if (img.empty()) {

		std::cerr << "Image not found!" << std::endl;

		return -1;
	}

	imshow("Display window", img);
	int k = cv::waitKey(0);

	return 0;
}