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

#include <gafro/algebra.hpp>
#include <gafro_ros/visualisation/Vector.hpp>

namespace gafro_ros::visualisation
{
    Vector::Vector(const std::string &name, sackmesser_ros::Interface *interface)
      : sackmesser_ros::Publisher<visualization_msgs::Marker, gafro::Vector<double>, gafro::Vector<double>>(name, interface)
    {
        config_ = interface->getConfigurations()->load<Configuration>("publisher/" + name + "/");
    }

    Vector::~Vector() {}

    bool Vector::Configuration::load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server)
    {
        return server->loadParameter(ns + "frame", &frame) &&            //
               server->loadParameter(ns + "radius", &radius, true) &&    //
               server->loadParameter(ns + "color/r", &color_r, true) &&  //
               server->loadParameter(ns + "color/g", &color_g, true) &&  //
               server->loadParameter(ns + "color/b", &color_b, true) &&  //
               server->loadParameter(ns + "color/a", &color_a, true);
    }

    visualization_msgs::Marker Vector::createMessage(const gafro::Vector<double> &v1, const gafro::Vector<double> &v2) const
    {
        visualization_msgs::Marker vector_msg;

        vector_msg.header.frame_id = config_.frame;
        vector_msg.header.stamp = ros::Time::now();
        vector_msg.pose.orientation.w = 1.0;

        vector_msg.type = visualization_msgs::Marker::ARROW;
        vector_msg.scale.x = 2.0 * config_.radius;
        vector_msg.scale.y = 2.0 * config_.radius;
        vector_msg.scale.z = 2.0 * config_.radius;
        vector_msg.color.r = static_cast<float>(config_.color_r);
        vector_msg.color.g = static_cast<float>(config_.color_g);
        vector_msg.color.b = static_cast<float>(config_.color_b);
        vector_msg.color.a = static_cast<float>(config_.color_a);

        geometry_msgs::Point p1;
        p1.x = v1.get<gafro::blades::e1>();
        p1.y = v1.get<gafro::blades::e2>();
        p1.z = v1.get<gafro::blades::e3>();

        geometry_msgs::Point p2;
        p2.x = v2.get<gafro::blades::e1>();
        p2.y = v2.get<gafro::blades::e2>();
        p2.z = v2.get<gafro::blades::e3>();

        vector_msg.points.push_back(p1);
        vector_msg.points.push_back(p2);

        return vector_msg;
    }

    REGISTER_CLASS(sackmesser_ros::PublisherFactory, Vector, "gafro_vector");

}  // namespace gafro_ros::visualisation