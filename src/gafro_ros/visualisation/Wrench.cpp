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

#include <Eigen/Geometry>
//
#include <gafro/algebra.hpp>
#include <gafro/physics/Wrench.hxx>
#include <gafro_ros/visualisation/Wrench.hpp>

namespace gafro_ros::visualisation
{
    Wrench::Wrench(const std::string &name, sackmesser_ros::Interface *interface)  //
      : sackmesser_ros::Publisher<geometry_msgs::WrenchStamped, gafro::Wrench<double>>(name, interface)
    {
        config_ = interface->getConfigurations()->load<Configuration>("publisher/" + name + "/");
    }

    Wrench::~Wrench() {}

    bool Wrench::Configuration::load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server)
    {
        return server->loadParameter(ns + "frame", &frame);
    }

    geometry_msgs::WrenchStamped Wrench::createMessage(const gafro::Wrench<double> &wrench) const
    {
        geometry_msgs::WrenchStamped wrench_msg;

        wrench_msg.header.stamp = ros::Time::now();
        wrench_msg.header.frame_id = config_.frame;

        wrench_msg.wrench.force.x = wrench.multivector().get<gafro::blades::e01>();
        wrench_msg.wrench.force.y = wrench.multivector().get<gafro::blades::e02>();
        wrench_msg.wrench.force.z = wrench.multivector().get<gafro::blades::e03>();

        wrench_msg.wrench.torque.x = wrench.multivector().get<gafro::blades::e23>();
        wrench_msg.wrench.torque.y = -wrench.multivector().get<gafro::blades::e13>();
        wrench_msg.wrench.torque.z = wrench.multivector().get<gafro::blades::e12>();

        return wrench_msg;
    }

    REGISTER_CLASS(sackmesser_ros::PublisherFactory, Wrench, "gafro_wrench");

}  // namespace gafro_ros::visualisation