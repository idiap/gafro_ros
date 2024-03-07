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
#include <gafro_ros/conversion/Line.hpp>

namespace gafro_ros
{

    visualization_msgs::Marker convertToMarker(const gafro::Line<double> &line, const std::string &frame, const int &id, const double &scale,  //
                                               const double &r, const double &g, const double &b, const double &a)
    {
        visualization_msgs::Marker line_msg;

        gafro::PointPair<double> pp = (gafro::Scalar<double>(-1.0) * (gafro::Line<double>(line) | gafro::E0<double>(1.0)).evaluate() +
                                       gafro::E0i<double>(0.5 * line.norm()) + gafro::Multivector<double, 9, 10, 11>({ 0.0, 0.0, 0.0 }))
                                        .evaluate();

        Eigen::Vector3d p1 = pp.getPoint1().vector().topRows(3);
        Eigen::Vector3d p2 = pp.getPoint2().vector().topRows(3);

        Eigen::Vector3d direction = (p2 - p1).normalized();

        if (direction.norm() > 1e-8)
        {
            line_msg.type = visualization_msgs::Marker::LINE_STRIP;

            geometry_msgs::Point p_msg_1;
            p_msg_1.x = p1.x() + 100.0 * direction.x();
            p_msg_1.y = p1.y() + 100.0 * direction.y();
            p_msg_1.z = p1.z() + 100.0 * direction.z();
            line_msg.points.push_back(p_msg_1);

            geometry_msgs::Point p_msg_2;
            p_msg_2.x = p1.x() - 100.0 * direction.x();
            p_msg_2.y = p1.y() - 100.0 * direction.y();
            p_msg_2.z = p1.z() - 100.0 * direction.z();
            line_msg.points.push_back(p_msg_2);
        }
        else if (!direction.hasNaN())
        {
            line_msg.type = visualization_msgs::Marker::POINTS;

            geometry_msgs::Point p_msg;

            p1 = p1.normalized();

            p_msg.x = p1.x();
            p_msg.y = p1.y();
            p_msg.z = p1.z();
            line_msg.points.push_back(p_msg);
        }

        line_msg.header.frame_id = frame;
        line_msg.header.stamp = ros::Time::now();

        line_msg.id = id;

        line_msg.pose.orientation.w = 1.0;

        line_msg.scale.x = scale;
        line_msg.color.r = static_cast<float>(r);
        line_msg.color.g = static_cast<float>(g);
        line_msg.color.b = static_cast<float>(b);
        line_msg.color.a = static_cast<float>(a);

        return line_msg;
    }

}  // namespace gafro_ros