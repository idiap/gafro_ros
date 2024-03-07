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
#include <gafro_ros/conversion/Sphere.hpp>

namespace gafro_ros
{

    visualization_msgs::Marker convertToMarker(const gafro::Sphere<double> &sphere, const std::string &frame, const int &id,  //
                                               const double &r, const double &g, const double &b, const double &a)
    {
        visualization_msgs::Marker sphere_msg;

        Eigen::Vector3d center = sphere.getCenter().vector().topRows(3);

        sphere_msg.pose.position.x = center.x();
        sphere_msg.pose.position.y = center.y();
        sphere_msg.pose.position.z = center.z();

        double diameter = 2.0 * sphere.getRadius();

        sphere_msg.header.frame_id = frame;
        sphere_msg.header.stamp = ros::Time::now();

        sphere_msg.id = id;

        sphere_msg.pose.orientation.w = 1.0;

        sphere_msg.type = visualization_msgs::Marker::SPHERE;
        sphere_msg.scale.x = diameter;
        sphere_msg.scale.y = diameter;
        sphere_msg.scale.z = diameter;
        sphere_msg.color.r = static_cast<float>(r);
        sphere_msg.color.g = static_cast<float>(g);
        sphere_msg.color.b = static_cast<float>(b);
        sphere_msg.color.a = static_cast<float>(a);

        return sphere_msg;
    }

}  // namespace gafro_ros