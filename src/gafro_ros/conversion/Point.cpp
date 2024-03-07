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
#include <gafro_ros/conversion/Point.hpp>

namespace gafro_ros
{

    geometry_msgs::Point convertToPointMsg(const gafro::Point<double> &point)
    {
        geometry_msgs::Point point_msg;

        Eigen::Vector3d p = point.vector().topRows(3);

        point_msg.x = p[0];
        point_msg.y = p[1];
        point_msg.z = p[2];

        return point_msg;
    }

}  // namespace gafro_ros