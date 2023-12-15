#include <iostream>

#include "opencv2/opencv.hpp"

#include "Lian/Point.hpp"

#include "Lian/Detail/Geometry.hpp"
#include "Lian/Detail/Map.hpp"
#include "Lian/Detail/LianFunctions.hpp"

#define PATH_IMG "resources/map.png"

using namespace std;
using namespace Algorithms::Graph::Geometry;
using namespace Algorithms::Graph::Map;
using Algorithms::Graph::LianFunctions::Expand;

int main() {

	cv::Mat rawImg = cv::imread(PATH_IMG, cv::IMREAD_COLOR);
	cv::Mat img;
	cv::cvtColor(rawImg, img, cv::COLOR_BGR2GRAY);

	img.setTo(255, img > 200);
	img.setTo(0, img != 255);

	Point start = Point(165, 305);
	Point goal = Point(410, 420);

	Point point = Point(100, 100);

	// --- testing ---

	cout << "Test Geometry" << endl;

	cout << "Distance between vectors" << endl;
	cout << distanceBetweenPoints(start, goal) << endl;

	cout << "Angle between vectors" << endl;
	cout << angleBetweenVectors(start, point, point, goal) << endl;

	cout << "midpoint" << endl;
	auto points = midpoint(start, 2);

	for (auto point : points) {
		cout << "Point(" << point.x << ", " << point.y << ")" << endl;
	}

	cout << "line of sight" << endl;
	auto x = Point(410, 430);
	auto line = lineOfSight(goal, x);
	for (auto point : line) {
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
	for (auto point : points2) {
		cout << "Point(" << point.x << ", " << point.y << ")" << endl;
	}
	cout << points2.size() << endl;

	cout << "Test Expand1" << endl;

	std::vector<StagePoint> close;
	std::map<Point, StagePoint> mapPath;

	//151, 299
	auto res = Expand(sP, m, sP, 15, 25, close, x, mapPath);

	for (auto point : res) {

		cout << "(" << point.point.x << ", " << point.point.y << ")" << " Dist: " << point.distance << "; simAngles: " << point.sumAngles << endl;
	}
	
	cout << "Expand2" << endl;

	StagePoint currentPoint { Point(151, 299), sP.point, 15.0, 0.0 };
	close.push_back(sP);
	mapPath.insert({ start, sP });
	auto res2 = Expand(sP, m, currentPoint, 15, 25, close, x, mapPath);
	for (auto point : res2) {

		cout << "(" << point.point.x << ", " << point.point.y << ")" << " Dist: " << point.distance << "; simAngles: " << point.sumAngles << endl;
	}

	// --- end testing----

	if (img.empty()) {

		std::cerr << "Image not found!" << std::endl;

		return -1;
	}

	imshow("Display window", img);
	int k = cv::waitKey(0);

	return 0;
}