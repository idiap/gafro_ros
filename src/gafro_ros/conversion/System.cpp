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
#include <gafro_ros/conversion/System.hpp>
#include <gafro_ros/conversion/Visual.hpp>

namespace gafro_ros
{

    void addLinkVisual(const gafro::SystemVisual *system,               //
                       visualization_msgs::MarkerArray &system_visual,  //
                       const gafro::Link<double> *link,                 //
                       const gafro::Motor<double> &motor,               //
                       int &id,                                         //
                       const std::string &frame,                        //
                       const Eigen::VectorXd &joint_positions)
    {
        if (system->hasVisual(link->getName()))
        {
            const gafro::Visual *visual = system->getVisual(link->getName());

            visualization_msgs::Marker visual_marker = convertToMarker(visual);

            visual_marker.pose = convertToPose(motor * visual->getTransform());

            visual_marker.header.frame_id = frame;
            visual_marker.header.stamp = ros::Time::now();
            visual_marker.ns = link->getName();
            visual_marker.id = static_cast<int>(system_visual.markers.size());

            system_visual.markers.push_back(visual_marker);
        }

        for (const auto *child_joint : link->getChildJoints())
        {
            if (child_joint->getChildLink())
            {
                if (child_joint->isActuated())
                {
                    double joint_position = 0.0;

                    if (id < joint_positions.rows())
                    {
                        joint_position = joint_positions[id];
                    }

                    addLinkVisual(system,                                         //
                                  system_visual,                                  //
                                  child_joint->getChildLink(),                    //
                                  motor * child_joint->getMotor(joint_position),  //
                                  ++id,                                           //
                                  frame,                                          //
                                  joint_positions);
                }
                else
                {
                    addLinkVisual(system,                              //
                                  system_visual,                       //
                                  child_joint->getChildLink(),         //
                                  motor * child_joint->getMotor(0.0),  //
                                  id,                                  //
                                  frame,                               //
                                  joint_positions);
                }
            }
        }
    }

    visualization_msgs::MarkerArray convertToMarker(const gafro::SystemVisual *system,       //
                                                    const Eigen::VectorXd &joint_positions,  //
                                                    gafro::Motor<double> base_motor,         //
                                                    const std::string &frame)
    {
        visualization_msgs::MarkerArray system_visual;

        int id = 0;
        addLinkVisual(system,                            //
                      system_visual,                     //
                      system->getLinks().front().get(),  //
                      base_motor,                        //
                      id,                                //
                      frame,                             //
                      joint_positions);

        return system_visual;
    }

}  // namespace gafro_ros