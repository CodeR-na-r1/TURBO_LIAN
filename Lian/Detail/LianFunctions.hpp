#pragma once

#include "opencv2/opencv.hpp"

#include <vector>
#include <map>

#include "../Point.hpp"
#include "StagePoint.hpp"
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
			using Algorithms::Graph::Geometry::distanceBetweenPoints;
			using Algorithms::Graph::Geometry::angleBetweenVectors;

			bool validPath(vector<Point> points, Map<cv::Mat> image) {

				for (auto point : points) {

					if (!image.isFree(point))
						return false;
				}
				return true;
			}

			std::vector<StagePoint> Expand(StagePoint start, Map<cv::Mat> image, StagePoint point, int deltaDist, int deltaAngle, std::vector<StagePoint> CLOSE, Point goal, std::map<Point, StagePoint> mapPath) {

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
					for (auto cl : CLOSE) {
						if (midpoint == cl.point && point.point == cl.parent) {
							inClose = true;
							break;
						}
					}
					if (inClose) {
						continue;
					}

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

		}
	}
}