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
#include <gafro_ros/visualisation/MotorVector.hpp>

namespace gafro_ros::visualisation
{
    MotorVector::MotorVector(const std::string &name, sackmesser_ros::Interface *interface)
      : sackmesser_ros::Publisher<visualization_msgs::MarkerArray, std::vector<gafro::Motor<double>>>(name, interface)
    {
        config_ = interface->getConfigurations()->load<Configuration>("publisher/" + name + "/");
    }

    MotorVector::~MotorVector() {}

    bool MotorVector::Configuration::load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server)
    {
        return server->loadParameter(ns + "scale", &scale, true) &&  //
               server->loadParameter(ns + "frame", &frame) &&        //
               server->loadParameter(ns + "opacity", &opacity, true);
    }

    visualization_msgs::MarkerArray MotorVector::createMessage(const std::vector<gafro::Motor<double>> &motors) const
    {
        visualization_msgs::MarkerArray frame_msg;

        int id = 0;

        for (const gafro::Motor<double> &motor : motors)
        {
            for (const visualization_msgs::Marker &marker : convertToMarkerArray(motor, config_.frame, id, config_.scale, config_.opacity).markers)
            {
                frame_msg.markers.push_back(marker);

                id += 3;
            }
        }

        return frame_msg;
    }

    REGISTER_CLASS(sackmesser_ros::PublisherFactory, MotorVector, "gafro_motor_vector");

}  // namespace gafro_ros::visualisation