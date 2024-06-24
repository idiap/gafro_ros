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

#include <gafro_ros/conversion/Point.hpp>
#include <gafro_ros/visualisation/PointVector.hpp>

namespace gafro_ros::visualisation
{
    PointVector::PointVector(const std::string &name, sackmesser_ros::Interface *interface)
      : sackmesser_ros::Publisher<visualization_msgs::Marker, std::vector<gafro::Point<double>>>(name, interface)
    {
        config_ = interface->getConfigurations()->load<Configuration>("publisher/" + name + "/");
    }

    PointVector::~PointVector() {}

    bool PointVector::Configuration::load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server)
    {
        return server->loadParameter(ns + "frame", &frame) &&            //
               server->loadParameter(ns + "radius", &radius, true) &&    //
               server->loadParameter(ns + "color/r", &color_r, true) &&  //
               server->loadParameter(ns + "color/g", &color_g, true) &&  //
               server->loadParameter(ns + "color/b", &color_b, true) &&  //
               server->loadParameter(ns + "color/a", &color_a, true);
    }

    visualization_msgs::Marker PointVector::createMessage(const std::vector<gafro::Point<double>> &values) const
    {
        int i = 0;

        visualization_msgs::Marker point_msg;

        point_msg.header.frame_id = config_.frame;
        point_msg.header.stamp = ros::Time::now();
        point_msg.id = i++;

        point_msg.type = visualization_msgs::Marker::SPHERE_LIST;
        point_msg.pose.orientation.w = 1.0;
        point_msg.scale.x = 2.0 * config_.radius;
        point_msg.scale.y = 2.0 * config_.radius;
        point_msg.scale.z = 2.0 * config_.radius;
        point_msg.color.r = static_cast<float>(config_.color_r);
        point_msg.color.g = static_cast<float>(config_.color_g);
        point_msg.color.b = static_cast<float>(config_.color_b);
        point_msg.color.a = static_cast<float>(config_.color_a);

        for (const gafro::Point<double> &point : values)
        {
            point_msg.points.push_back(convertToPointMsg(point));
        }

        return point_msg;
    }

    REGISTER_CLASS(sackmesser_ros::PublisherFactory, PointVector, "gafro_point_vector");

}  // namespace gafro_ros::visualisation