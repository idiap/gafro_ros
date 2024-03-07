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

#include <gafro_ros/conversion/Line.hpp>
//
#include <gafro_ros/visualisation/LineVector.hpp>

namespace gafro_ros::visualisation
{
    LineVector::LineVector(const std::string &name, sackmesser_ros::Interface *interface)
      : sackmesser_ros::Publisher<visualization_msgs::MarkerArray, std::vector<gafro::Line<double>>>(name, interface)
    {
        config_ = interface->getConfigurations()->load<Configuration>("publisher/" + name + "/");
    }

    LineVector::~LineVector() {}

    bool LineVector::Configuration::load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server)
    {
        return server->loadParameter(ns + "frame", &frame) &&            //
               server->loadParameter(ns + "scale", &scale, true) &&      //
               server->loadParameter(ns + "color/r", &color_r, true) &&  //
               server->loadParameter(ns + "color/g", &color_g, true) &&  //
               server->loadParameter(ns + "color/b", &color_b, true) &&  //
               server->loadParameter(ns + "color/a", &color_a, true);
    }

    visualization_msgs::MarkerArray LineVector::createMessage(const std::vector<gafro::Line<double>> &lines) const
    {
        int i = 0;

        visualization_msgs::MarkerArray lines_msg;

        for (const gafro::Line<double> &line : lines)
        {
            lines_msg.markers.push_back(
              convertToMarker(line, config_.frame, i++, config_.scale, config_.color_r, config_.color_g, config_.color_b, config_.color_a));
        }

        return lines_msg;
    }

    REGISTER_CLASS(sackmesser_ros::PublisherFactory, LineVector, "gafro_line_vector");

}  // namespace gafro_ros::visualisation