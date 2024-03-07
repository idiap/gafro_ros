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

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
//
#include <gafro_ros/visualisation/Cube.hpp>

namespace gafro_ros::visualisation
{
    Cube::Cube(const std::string &name, sackmesser_ros::Interface *interface)
      : sackmesser_ros::Publisher<visualization_msgs::Marker, Eigen::Vector3d, Eigen::Vector3d>(name, interface)
    {
        config_ = interface->getConfigurations()->load<Configuration>("publisher/" + name + "/");
    }

    Cube::~Cube() {}

    bool Cube::Configuration::load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server)
    {
        return server->loadParameter(ns + "frame", &frame) &&            //
               server->loadParameter(ns + "color/r", &color_r, true) &&  //
               server->loadParameter(ns + "color/g", &color_g, true) &&  //
               server->loadParameter(ns + "color/b", &color_b, true) &&  //
               server->loadParameter(ns + "color/a", &color_a, true);
    }

    visualization_msgs::Marker Cube::createMessage(const Eigen::Vector3d &center, const Eigen::Vector3d &sides) const
    {
        visualization_msgs::Marker cube_msg;

        cube_msg.header.frame_id = config_.frame;
        cube_msg.header.stamp = ros::Time::now();

        cube_msg.pose.position.x = center.x();
        cube_msg.pose.position.y = center.y();
        cube_msg.pose.position.z = center.z();

        cube_msg.pose.orientation.x = 0.0;
        cube_msg.pose.orientation.y = 0.0;
        cube_msg.pose.orientation.z = 0.0;
        cube_msg.pose.orientation.w = 1.0;

        cube_msg.type = visualization_msgs::Marker::CUBE;
        cube_msg.scale.x = sides.x();
        cube_msg.scale.y = sides.y();
        cube_msg.scale.z = sides.z();
        cube_msg.color.r = static_cast<float>(config_.color_r);
        cube_msg.color.g = static_cast<float>(config_.color_g);
        cube_msg.color.b = static_cast<float>(config_.color_b);
        cube_msg.color.a = static_cast<float>(config_.color_a);

        return cube_msg;
    }

    REGISTER_CLASS(sackmesser_ros::PublisherFactory, Cube, "gafro_cube");

}  // namespace gafro_ros::visualisation