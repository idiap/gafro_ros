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

#include <gafro_robot_descriptions/SystemVisual.hpp>
#include <gafro_ros/conversion/System.hpp>
#include <gafro_ros/visualisation/System.hpp>

namespace gafro_ros::visualisation
{
    bool System::Configuration::load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server)
    {
        return server->loadParameter("publisher/" + ns + "/description", &description) &&  //
               server->loadParameter("publisher/" + ns + "/frame", &frame) &&              //
               server->loadParameter("publisher/" + ns + "/color/r", &color_r, true) &&    //
               server->loadParameter("publisher/" + ns + "/color/g", &color_g, true) &&    //
               server->loadParameter("publisher/" + ns + "/color/b", &color_b, true) &&    //
               server->loadParameter("publisher/" + ns + "/color/a", &color_a, true);
    }

    System::System(const std::string &name, sackmesser_ros::Interface *interface)
      : sackmesser_ros::Publisher<visualization_msgs::MarkerArray, Eigen::MatrixXd, gafro::Motor<double>>(name, interface)
    {
        config_ = interface->getConfigurations()->load<Configuration>(name);
        system_ = std::make_unique<gafro::SystemVisual>(config_.description);
    }

    System::~System() = default;

    visualization_msgs::MarkerArray System::createMessage(const Eigen::MatrixXd &position, const gafro::Motor<double> &base) const
    {
        visualization_msgs::MarkerArray all_markers;

        int id = 0;

        for (unsigned j = 0; j < position.cols(); ++j)
        {
            visualization_msgs::MarkerArray markers = convertToMarker(system_.get(), position.col(j), base, config_.frame);

            for (visualization_msgs::Marker &marker : markers.markers)
            {
                marker.color.r = static_cast<float>(config_.color_r);
                marker.color.g = static_cast<float>(config_.color_g);
                marker.color.b = static_cast<float>(config_.color_b);
                marker.color.a = static_cast<float>(config_.color_a);

                marker.mesh_use_embedded_materials = true;

                marker.id = id + marker.id;

                all_markers.markers.push_back(marker);
            }

            id += position.rows();
        }

        return all_markers;
    }

    REGISTER_CLASS(sackmesser_ros::PublisherFactory, System, "gafro_system");

}  // namespace gafro_ros::visualisation