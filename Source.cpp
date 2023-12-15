#include <iostream>

#include <chrono>

#include "opencv2/opencv.hpp"

#include "Lian/Point.hpp"

#include "Lian/Lian.hpp"
#include "Lian/Detail/Geometry.hpp"
#include "Lian/Detail/Map.hpp"
#include "Lian/Detail/LianFunctions.hpp"

#define PATH_IMG "resources/map.png"

using namespace std;
using Algorithms::Graph::Lian::Lian;
using namespace Algorithms::Graph::Geometry;
using namespace Algorithms::Graph::Map;
using Algorithms::Graph::LianFunctions::Expand;

int main() {

	cv::Mat rawImg = cv::imread(PATH_IMG, cv::IMREAD_COLOR);

	if (rawImg.empty()) {

		std::cerr << "Image not found!" << std::endl;

		return -1;
	}

	cv::Mat img;
	cv::cvtColor(rawImg, img, cv::COLOR_BGR2GRAY);

	img.setTo(255, img > 200);
	img.setTo(0, img != 255);

	Point start = Point(165, 305);
	Point goal = Point(410, 420);

	cv::circle(img, cv::Point(start.x, start.y), 3, cv::Scalar(0, 0, 255), 1, cv::FILLED);
	cv::circle(img, cv::Point(goal.x, goal.y), 3, cv::Scalar(0, 255, 0), 1, cv::FILLED);

	Point point = Point(100, 100);

	// --- testing ---

	cout << "Test Geometry" << endl;

	cout << "Distance between vectors" << endl;
	cout << distanceBetweenPoints(start, goal) << endl;

	cout << "Angle between vectors" << endl;
	cout << angleBetweenVectors(start, point, point, goal) << endl;

	cout << "midpoint" << endl;
	auto points = midpoint(start, 2);

	for (auto&& point : points) {
		cout << "Point(" << point.x << ", " << point.y << ")" << endl;
	}

	cout << "line of sight" << endl;
	auto x = Point(410, 430);
	auto line = lineOfSight(goal, x);
	for (auto&& point : line) {
		cout << "Point(" << point.x << ", " << point.y << ")" << endl;
	}

	// test Map

	cout << "Test Map" << endl;

	Map m(img);
	cout << "isFree1" << m.isFree(start) << endl;
	cout << "isFree2" << m.isFree({ 70, 70 }) << endl;

	// test Expand

	cout << "Test Midpoint2" << endl;

	StagePoint sP(start, Point(0, 0), 0.0, 0.0);

	auto points2 = midpoint(sP.point, 15);
	for (auto&& point : points2) {
		cout << "Point(" << point.x << ", " << point.y << ")" << endl;
	}
	cout << points2.size() << endl;

	cout << "Test Expand1" << endl;

	std::vector<StagePoint> close;
	std::map<Point, StagePoint> mapPath;

	auto res = Expand(sP, m, sP, 15, 25, close, x, mapPath);

	for (auto&& point : res) {

		cout << "(" << point.point.x << ", " << point.point.y << ")" << " Dist: " << point.distance << "; simAngles: " << point.sumAngles << endl;
	}
	
	cout << "Expand2" << endl;

	StagePoint currentPoint { Point(151, 299), sP.point, 15.0, 0.0 };
	close.push_back(sP);
	mapPath.insert({ start, sP });
	auto res2 = Expand(sP, m, currentPoint, 15, 25, close, x, mapPath);
	for (auto&&  point : res2) {

		cout << "(" << point.point.x << ", " << point.point.y << ")" << " Dist: " << point.distance << "; simAngles: " << point.sumAngles << endl;
	}

	cout << "Test Lian" << endl;

	auto timer = std::chrono::steady_clock::now();
	auto resPath = Lian(start, goal, m, 30, 25);

	for (auto&& point : resPath) {

		cv::circle(m.getMap(), cv::Point(point.x, point.y), 3, cv::Scalar(100, 100, 100), 2);
	}
	std::cout << "Time code -> " << std::chrono::duration <double, std::milli>(std::chrono::steady_clock::now() - timer).count() << std::endl;

	// --- end testing----

	imshow("Display window", img);
	int k = cv::waitKey(0);

	return 0;
}