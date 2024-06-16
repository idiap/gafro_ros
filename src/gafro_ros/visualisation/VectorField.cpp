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
#include <gafro_ros/conversion/Point.hpp>
#include <gafro_ros/visualisation/VectorField.hpp>

namespace gafro_ros::visualisation
{
    VectorField::VectorField(const std::string &name, sackmesser_ros::Interface *interface)
      : sackmesser_ros::Publisher<visualization_msgs::MarkerArray, std::vector<gafro::Point<double>>, gafro::VectorField>(name, interface)
    {
        config_ = interface->getConfigurations()->load<Configuration>("publisher/" + name + "/");
    }

    VectorField::~VectorField() {}

    bool VectorField::Configuration::load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server)
    {
        return server->loadParameter(ns + "frame", &frame) &&            //
               server->loadParameter(ns + "radius", &radius, true) &&    //
               server->loadParameter(ns + "color/r", &color_r, true) &&  //
               server->loadParameter(ns + "color/g", &color_g, true) &&  //
               server->loadParameter(ns + "color/b", &color_b, true) &&  //
               server->loadParameter(ns + "color/a", &color_a, true);
    }

    visualization_msgs::MarkerArray VectorField::createMessage(const std::vector<gafro::Point<double>> &points, const gafro::VectorField &field) const
    {
        visualization_msgs::MarkerArray vector_field_msg;

        int id = 0;

        for (const auto &point : points)
        {
            visualization_msgs::Marker vector_msg;

            vector_msg.header.frame_id = config_.frame;
            vector_msg.header.stamp = ros::Time::now();
            vector_msg.pose.orientation.w = 1.0;
            vector_msg.id = id++;

            vector_msg.type = visualization_msgs::Marker::ARROW;
            vector_msg.scale.x = 0.05;
            vector_msg.scale.y = 0.1;
            vector_msg.scale.z = 0.1;
            vector_msg.color.r = static_cast<float>(config_.color_r);
            vector_msg.color.g = static_cast<float>(config_.color_g);
            vector_msg.color.b = static_cast<float>(config_.color_b);
            vector_msg.color.a = static_cast<float>(config_.color_a);

            // double x = -25.0 + 50.0 * double(i) / 25.0;
            // double y = -25.0 + 50.0 * double(j) / 25.0;
            // double z = 0.0;  //-5.0 + 10.0 * double(k) / 25.0;

            geometry_msgs::Point p1 = convertToPointMsg(point);

            gafro::Vector<double> vector = field(point);

            geometry_msgs::Point p2;
            p2.x = p1.x + vector.get<gafro::blades::e1>();
            p2.y = p1.y + vector.get<gafro::blades::e2>();
            p2.z = p1.z + vector.get<gafro::blades::e3>();

            vector_msg.points.push_back(p1);
            vector_msg.points.push_back(p2);

            vector_field_msg.markers.push_back(vector_msg);
        }

        return vector_field_msg;
    }

    REGISTER_CLASS(sackmesser_ros::PublisherFactory, VectorField, "gafro_vector_field");

}  // namespace gafro_ros::visualisation