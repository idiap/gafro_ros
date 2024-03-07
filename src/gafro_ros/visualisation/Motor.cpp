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

#include <gafro_ros/conversion/Motor.hpp>
//
#include <gafro_ros/visualisation/Motor.hpp>

namespace gafro_ros::visualisation
{
    Motor::Motor(const std::string &name, sackmesser_ros::Interface *interface)
      : sackmesser_ros::Publisher<visualization_msgs::MarkerArray, gafro::Motor<double>>(name, interface)
    {
        config_ = interface->getConfigurations()->load<Configuration>("publisher/" + name + "/");
    }

    Motor::~Motor() {}

    bool Motor::Configuration::load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server)
    {
        return server->loadParameter(ns + "scale", &scale, true) &&      //
               server->loadParameter(ns + "opacity", &opacity, true) &&  //
               server->loadParameter(ns + "frame", &frame);
    }

    visualization_msgs::MarkerArray Motor::createMessage(const gafro::Motor<double> &motor) const
    {
        return convertToMarkerArray(motor, config_.frame, 0, config_.scale, config_.opacity);
    }

    REGISTER_CLASS(sackmesser_ros::PublisherFactory, Motor, "gafro_motor");

}  // namespace gafro_ros::visualisation