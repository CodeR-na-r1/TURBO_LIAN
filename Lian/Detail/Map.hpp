#pragma once

#include "../Point.hpp"

namespace Algorithms {

	namespace Graph {

		namespace Map {

			using Algorithms::Graph::Geometry::Point;

			template <typename T>
			class Map {
				T image;

			public:

				Map(T img) :image(img) {}

				bool isFree(Point point) {

					return ((int)image.at<uchar>(cv::Point(point.x, point.y))) != 0;
				}

			};
		}
	}
}