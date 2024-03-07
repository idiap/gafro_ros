/*
    Copyright (c) Idiap Research Institute, http://www.idiap.ch/
    Written by Tobias Löw <https://tobiloew.ch>

    This file is part of gafro_ros.

    gafro is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 3 as
    published by the Free Software Foundation.

    gafro is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with gafro. If not, see <http://www.gnu.org/licenses/>.
*/

#include <geometry_msgs/Point.h>

#include <gafro/algebra.hpp>
//
#include <gafro_ros/conversion/Point.hpp>
#include <gafro_ros/conversion/PointPair.hpp>

namespace gafro_ros
{
    std::vector<geometry_msgs::Point> convertToPointMsgs(const gafro::PointPair<double> &point_pair)
    {
        geometry_msgs::Point p1 = convertToPointMsg(point_pair.getPoint1());
        geometry_msgs::Point p2 = convertToPointMsg(point_pair.getPoint2());

        std::vector<geometry_msgs::Point> points;

        points.push_back(p1);
        points.push_back(p2);

        return points;
    }
}  // namespace gafro_ros