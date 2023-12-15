#pragma once

#include "opencv2/opencv.hpp"

#include <iostream>
#include <fstream>

#include <chrono>
#include <thread>

#include <string>
#include <vector>
#include <map>

#include "../Point.hpp"
#include "StagePoint.hpp"
#include "Path.hpp"
#include "Geometry.hpp"
#include "Map.hpp"

using std::vector;
using std::map;

namespace Algorithms {

	namespace Graph {

		namespace LianFunctions {

			using Algorithms::Graph::Map::Map;
			using Algorithms::Graph::Geometry::Point;
			using Algorithms::Graph::Geometry::StagePoint;
			using Algorithms::Graph::Geometry::Path;
			using Algorithms::Graph::Geometry::distanceBetweenPoints;
			using Algorithms::Graph::Geometry::angleBetweenVectors;

			bool validPath(vector<Point> points, Map<cv::Mat> image) {

				for (auto&& point : points) {

					if (!image.isFree(point))
						return false;
				}
				return true;
			}

			std::vector<StagePoint> Expand(StagePoint start, Map<cv::Mat> image, StagePoint point, int deltaDist, int deltaAngle, std::vector<StagePoint> CLOSE, Point goal, std::map<Point, StagePoint>& mapPath) {

				std::vector<StagePoint> points;

				std::vector<Point> midpoints = Algorithms::Graph::Geometry::midpoint(point.point, deltaDist);

				if (distanceBetweenPoints(point.point, goal) < deltaDist) {
					midpoints.push_back(goal);
				}

				for (auto midpoint : midpoints) {
					if (!image.isFree(midpoint)) {
						continue;
					}

					double angle = 0.0;
					if (point.point != start.point) {
						angle = angleBetweenVectors(point.parent, point.point, point.point, midpoint);
					}
					if (angle > deltaAngle)
						continue;

					// CHECK THIS
					auto points_ = Algorithms::Graph::Geometry::lineOfSight(point.point, midpoint);
					if (!validPath(points_, image))
						continue;

					bool inClose = false;
					for (auto&& cl : CLOSE) {
						if (midpoint == cl.point && point.point == cl.parent) {
							inClose = true;
							break;
						}
					}
					if (inClose) {
						continue;
					}
					//if (mapPath.contains(midpoint) && angle + point.sumAngles <= mapPath.at(midpoint).sumAngles) {
					if (mapPath.contains(midpoint) && distanceBetweenPoints(point.point, midpoint) + point.distance < mapPath.at(midpoint).distance) {

						mapPath.at(midpoint) = StagePoint(midpoint,
							point.point,
							distanceBetweenPoints(point.point, midpoint) + point.distance,
							point.sumAngles + angle);
						points.push_back(mapPath.at(midpoint));
					}
					if (!mapPath.contains(midpoint)) {
						mapPath.insert({ midpoint, StagePoint(midpoint,
							point.point,
							distanceBetweenPoints(point.point, midpoint) + point.distance,
							point.sumAngles + angle) });
						points.push_back(mapPath.at(midpoint));

					}

				}
				return points;

			}

			vector<Point> unwindingPath(const map<Point, StagePoint> mapPath, Point start, Point end) {

				vector<Point> path;
				path.push_back(end);

				Point current = end;

				while (mapPath.contains(current) && mapPath.at(current).point != start) {

					current = mapPath.at(current).parent;

					path.push_back(current);
				}

				return { path.rbegin(), path.rend() };
			}

			// functions for image processing

			void saveImage(std::string nameFile, Map<cv::Mat> image) {

				cv::imwrite(nameFile, image.getMap());
			}

			cv::Mat drawStateOnImage(Point start, Point goal, Point current, Map<cv::Mat> img, const vector<StagePoint>& OPEN, const std::vector<StagePoint>& CLOSE, const std::map<Point, StagePoint>& mapPath) {

				int RADIUS{ 1 };
				int THICKNESS = -1;

				cv::Mat imgCopy;
				cv::cvtColor(img.getMap(), imgCopy, cv::COLOR_GRAY2RGB);

				for (auto&& point : OPEN) {

					cv::circle(imgCopy, cv::Point(point.point.x, point.point.y), RADIUS, cv::Scalar(0, 0, 255), THICKNESS);
				}

				for (auto&& point : CLOSE) {

					cv::circle(imgCopy, cv::Point(point.point.x, point.point.y), RADIUS, cv::Scalar(100, 100, 100), THICKNESS);
				}

				auto path = unwindingPath(mapPath, start, current);
				for (auto&& point : path) {

					cv::circle(imgCopy, cv::Point(point.x, point.y), RADIUS, cv::Scalar(0, 255, 0), THICKNESS);
				}

				cv::circle(imgCopy, cv::Point(goal.x, goal.y), RADIUS, cv::Scalar(), THICKNESS);

				return imgCopy;
			}

			void showImage(Map<cv::Mat> image) {

				cv::imshow("Lian processing...", image.getMap());
				cv::waitKey(1);
			}

			void showImageThread(bool& isAction, Point start, Point goal, Point current, Map<cv::Mat> img, vector<StagePoint>& OPEN, std::vector<StagePoint>& CLOSE, std::map<Point, StagePoint>& mapPath) {

				//while (isAction) {

					showImage(drawStateOnImage(start, goal, current, img, OPEN, CLOSE, mapPath));
					//std::this_thread::sleep_for(std::chrono::milliseconds(200));
				//}
			}
			// functions for log

			void logConsole(const Path& path) {

				std::cout << "Points: " << path.points.size() << std::endl;	// log in console
				std::cout << "Length: " << path.distance << std::endl;
				std::cout << "SumAngles: " << path.sumAngles << std::endl;
			}

			void logFile(const std::string& fileName, const Path& path, int deltaDist, int deltaAngle, double codeTime) {

				std::ofstream outLogFile(fileName);

				if (!outLogFile.is_open()) {
					return;
				}

				outLogFile << "Path Search Parameters:\n";
				outLogFile << "Distance delta -> " << deltaDist << "\n";
				outLogFile << "Angle delta -> " << deltaAngle << "°\n";

				outLogFile << "----- ----- -----" << "\n";

				outLogFile << "Path characteristics:\n";
				outLogFile << "Distance -> " << std::to_string(path.distance) << "px\n";
				outLogFile << "Sum angles -> " << std::to_string(path.sumAngles) << "°\n";

				outLogFile << "----- ----- -----" << "\n";

				outLogFile << "Pathfinding time -> " << codeTime << " seconds\n";

				outLogFile << "----- ----- -----" << "\n";

				outLogFile << "Points of path:\n";
				for (auto&& point : path.points) {

					outLogFile << point.x << ", " << point.y << "\n";
				}
				outLogFile << "----- ----- -----" << "\n";
			}
		}
	}
}