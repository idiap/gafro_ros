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

#include <gafro_ros/conversion/PointPair.hpp>
//
#include <gafro_ros/visualisation/PointPair.hpp>

namespace gafro_ros::visualisation
{
    PointPair::PointPair(const std::string &name, sackmesser_ros::Interface *interface)
      : sackmesser_ros::Publisher<visualization_msgs::Marker, gafro::PointPair<double>>(name, interface)
    {
        config_ = interface->getConfigurations()->load<Configuration>("publisher/" + name + "/");
    }

    PointPair::~PointPair() {}

    bool PointPair::Configuration::load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server)
    {
        return server->loadParameter(ns + "frame", &frame) &&            //
               server->loadParameter(ns + "radius", &radius, true) &&    //
               server->loadParameter(ns + "color/r", &color_r, true) &&  //
               server->loadParameter(ns + "color/g", &color_g, true) &&  //
               server->loadParameter(ns + "color/b", &color_b, true) &&  //
               server->loadParameter(ns + "color/a", &color_a, true);
    }

    visualization_msgs::Marker PointPair::createMessage(const gafro::PointPair<double> &point_pair) const
    {
        visualization_msgs::Marker pp_marker;

        pp_marker.header.frame_id = config_.frame;
        pp_marker.header.stamp = ros::Time::now();
        pp_marker.pose.orientation.w = 1.0;
        pp_marker.scale.x = 2.0 * config_.radius;
        pp_marker.scale.y = 2.0 * config_.radius;
        pp_marker.scale.z = 2.0 * config_.radius;
        pp_marker.color.r = static_cast<float>(config_.color_r);
        pp_marker.color.g = static_cast<float>(config_.color_g);
        pp_marker.color.b = static_cast<float>(config_.color_b);
        pp_marker.color.a = static_cast<float>(config_.color_a);

        pp_marker.type = visualization_msgs::Marker::SPHERE_LIST;

        pp_marker.points = convertToPointMsgs(point_pair);

        return pp_marker;
    }

    REGISTER_CLASS(sackmesser_ros::PublisherFactory, PointPair, "gafro_pointpair");

}  // namespace gafro_ros::visualisation