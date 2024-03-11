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

#pragma once

#include <gafro_robot_descriptions/SystemVisual.hpp>
//
#include <visualization_msgs/MarkerArray.h>

namespace gafro_ros
{

    visualization_msgs::MarkerArray convertToMarker(const gafro::SystemVisual *system,       //
                                                    const Eigen::VectorXd &joint_positions,  //
                                                    gafro::Motor<double> base_motor,         //
                                                    const std::string &frame);

}  // namespace gafro_ros