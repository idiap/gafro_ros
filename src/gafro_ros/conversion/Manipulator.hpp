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

#include <visualization_msgs/MarkerArray.h>

#include <gafro/robot.hpp>
//
#include <gafro_ros/conversion/Motor.hpp>
#include <gafro_ros/conversion/Visual.hpp>

namespace gafro_ros
{

    template <int dof>
    visualization_msgs::MarkerArray convertToMarker(const gafro::Manipulator<double, dof> &manipulator,  //
                                                    const Eigen::Vector<double, dof> &joint_positions,   //
                                                    gafro::Motor<double> base_motor,                     //
                                                    const std::string &frame)
    {
        visualization_msgs::MarkerArray marker_array;

        gafro::Motor<double> motor = base_motor;

        gafro::Link<double> *link = manipulator.getLinks().front().get();

        for (unsigned j = 0; j < dof; ++j)
        {
            visualization_msgs::Marker visual = convertToMarker(link->getVisual());

            visual.pose = convertToPose(motor);

            visual.header.frame_id = frame;
            visual.header.stamp = ros::Time::now();

            motor *= manipulator.getJoints()[j]->getMotor(joint_positions[j]);
        }

        return marker_array;
    }

}  // namespace gafro_ros