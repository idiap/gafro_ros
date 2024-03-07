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

#include <gafro/algebra.hpp>
//
#include <gafro_ros/conversion/Point.hpp>
//
#include <gafro_ros/conversion/Plane.hpp>

namespace gafro_ros
{

    visualization_msgs::Marker convertToMarker(const gafro::Plane<double> &plane, const std::string &frame, const int &id,  //
                                               const double &scale, const double &r, const double &g, const double &b, const double &a)
    {
        visualization_msgs::Marker plane_msg;

        gafro::Point<double> p0 = gafro::Scalar<double>(-1.0) * (plane | gafro::Point<double>()) * plane.inverse();
        gafro::Point<double> p1 = gafro::Scalar<double>(-1.0) * (plane | gafro::Point<double>(scale, 0.0, 0.0)) * plane.inverse();
        gafro::Point<double> p2 = gafro::Scalar<double>(-1.0) * (plane | gafro::Point<double>(0.0, scale, 0.0)) * plane.inverse();
        gafro::Point<double> p3 = gafro::Scalar<double>(-1.0) * (plane | gafro::Point<double>(-scale, 0.0, 0.0)) * plane.inverse();
        gafro::Point<double> p4 = gafro::Scalar<double>(-1.0) * (plane | gafro::Point<double>(0.0, -scale, 0.0)) * plane.inverse();

        plane_msg.points.push_back(convertToPointMsg(p1));
        plane_msg.points.push_back(convertToPointMsg(p2));
        plane_msg.points.push_back(convertToPointMsg(p0));

        plane_msg.points.push_back(convertToPointMsg(p2));
        plane_msg.points.push_back(convertToPointMsg(p3));
        plane_msg.points.push_back(convertToPointMsg(p0));

        plane_msg.points.push_back(convertToPointMsg(p3));
        plane_msg.points.push_back(convertToPointMsg(p4));
        plane_msg.points.push_back(convertToPointMsg(p0));

        plane_msg.points.push_back(convertToPointMsg(p4));
        plane_msg.points.push_back(convertToPointMsg(p1));
        plane_msg.points.push_back(convertToPointMsg(p0));

        plane_msg.header.frame_id = frame;
        plane_msg.header.stamp = ros::Time::now();

        plane_msg.id = id;

        plane_msg.pose.orientation.w = 1.0;

        plane_msg.type = visualization_msgs::Marker::TRIANGLE_LIST;
        plane_msg.scale.x = 1.0;
        plane_msg.scale.y = 1.0;
        plane_msg.scale.z = 1.0;
        plane_msg.color.r = static_cast<float>(r);
        plane_msg.color.g = static_cast<float>(g);
        plane_msg.color.b = static_cast<float>(b);
        plane_msg.color.a = static_cast<float>(a);

        return plane_msg;
    }

}  // namespace gafro_ros