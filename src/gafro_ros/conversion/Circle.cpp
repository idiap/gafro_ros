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
#include <Eigen/Geometry>
//
#include <gafro_ros/conversion/Circle.hpp>

namespace gafro_ros
{

    visualization_msgs::Marker convertToMarker(const gafro::Circle<double> &circle, const std::string &frame, const int &id,  //
                                               const double &r, const double &g, const double &b, const double &a)
    {
        visualization_msgs::Marker circle_msg;

        circle_msg.header.frame_id = frame;
        circle_msg.header.stamp = ros::Time::now();
        circle_msg.id = id;

        Eigen::Vector3d center = circle.getCenter().vector().topRows(3);
        double diameter = 2.0 * circle.getRadius();

        circle_msg.pose.position.x = center.x();
        circle_msg.pose.position.y = center.y();
        circle_msg.pose.position.z = center.z();

        gafro::Plane<double> plane = circle.getPlane();
        plane.normalize();

        Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d({ 0.0, 0.0, 1.0 }), plane.getNormal().vector()).normalized();

        circle_msg.pose.orientation.x = q.x();
        circle_msg.pose.orientation.y = q.y();
        circle_msg.pose.orientation.z = q.z();
        circle_msg.pose.orientation.w = q.w();

        circle_msg.type = visualization_msgs::Marker::CYLINDER;
        circle_msg.scale.x = diameter;
        circle_msg.scale.y = diameter;
        circle_msg.scale.z = 0.001;
        circle_msg.color.r = static_cast<float>(r);
        circle_msg.color.g = static_cast<float>(g);
        circle_msg.color.b = static_cast<float>(b);
        circle_msg.color.a = static_cast<float>(a);

        return circle_msg;
    }

}  // namespace gafro_ros