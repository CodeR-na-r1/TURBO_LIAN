#pragma once

#include "opencv2/opencv.hpp"

#include <iostream>
#include <algorithm>
#include <chrono>

#include <vector>
#include <float.h>

#include "Point.hpp"
#include "Detail/Map.hpp"
#include "Detail/StagePoint.hpp"

#include "detail/Geometry.hpp"
#include "Detail/LianFunctions.hpp"

namespace Algorithms {

	namespace Graph {

		namespace Lian {

			using std::vector;
			using Algorithms::Graph::Map::Map;
			using namespace Geometry;
			using LianFunctions::Expand;
			using LianFunctions::unwindingPath;
			using LianFunctions::showImageThread;

			vector<Point> Lian(Point start_, Point goal_, Map<cv::Mat> img, int deltaDist, int deltaAngle) {

				vector<StagePoint> OPEN, CLOSE;
				OPEN.reserve(100000);
				CLOSE.reserve(100000);

				StagePoint start(start_, Point(0, 0), 0.0, 0.0),
					goal(goal_, Point(0, 0), DBL_MAX, DBL_MAX);
				OPEN.push_back(start);

				std::map<Point, StagePoint> mapPath;

				int itCounter{ 0 };
				auto itCurrent = OPEN.begin();	// delete it and replace on obj downer
				StagePoint currentSPoint = start;

				vector<StagePoint> res;

				bool isAction{ true };
				//std::thread t([&isAction, start_, goal_, &currentSPoint, img, &OPEN, &CLOSE, &mapPath]() {
					//showImageThread(isAction, start_, goal_, currentSPoint.point, img, OPEN, CLOSE, mapPath);
					//});
				auto timer = std::chrono::steady_clock::now();

				while (!OPEN.empty()) {

					//itCurrent = std::min_element(OPEN.begin(), OPEN.end(), [goal_](auto left, auto right) { return (*left).distance + distanceBetweenPoints((*left).point, goal_) < (*right).distance + distanceBetweenPoints((*right).point, goal_); });
					itCurrent = std::min_element(OPEN.begin(), OPEN.end(), [goal_](auto left, auto right) { return left.distance + distanceBetweenPoints(left.point, goal_) < right.distance + distanceBetweenPoints(right.point, goal_); });
					currentSPoint = *itCurrent;
					OPEN.erase(std::remove(OPEN.begin(), OPEN.end(), *itCurrent), OPEN.end());

					if (currentSPoint.point == goal.point) {

						isAction = false;
						auto path = unwindingPath(mapPath, start_, goal_);
						std::cout << "Points: " << path.size() << std::endl;
						std::cout << "LEnght: " << goal.distance << std::endl;
						//t.join();
						return path;
					}

					res = Expand(start, img, currentSPoint, deltaDist, deltaAngle, CLOSE, goal_, mapPath);

					for (auto&& sPoint: res) {
					
						OPEN.push_back(sPoint);
					}
					CLOSE.push_back(currentSPoint);

					if (std::chrono::duration <double, std::milli>
						(std::chrono::steady_clock::now() - timer).count() > 200) {

						showImageThread(isAction, start_, goal_, currentSPoint.point, img, OPEN, CLOSE, mapPath);
						timer = std::chrono::steady_clock::now();
					}

					std::cout << "Iteration -> " << itCounter << std::endl;
					++itCounter;

				}

				return {};
			}

		}

	}

}